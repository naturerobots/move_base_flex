#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mbf_abstract_core/abstract_plan_refiner.h>
#include <mbf_abstract_nav/abstract_plan_refiner_execution.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

using geometry_msgs::msg::PoseStamped;
using mbf_abstract_core::AbstractPlanRefiner;

struct AbstractPlanRefinerMock : public AbstractPlanRefiner
{
  MOCK_METHOD6(
    applyRefinement,
    uint32_t(
      const std::vector<PoseStamped> &, std::vector<PoseStamped> &, float &, float &, float &,
      std::string &));

  MOCK_METHOD0(cancel, bool());
};

using mbf_abstract_nav::AbstractPlanRefinerExecution;
using testing::_;
using testing::DoAll;
using testing::InSequence;
using testing::Return;
using testing::SetArgReferee;
using testing::Test;

struct AbstractPlanRefinerExecutionFixture : public Test
{
  void initRosNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
  {
    node_ptr_ = std::make_shared<rclcpp::Node>(
      "abstract_plan_refiner_execution_test", "",
      node_options);
    node_ptr_->get_logger().set_level(rclcpp::Logger::Level::Fatal);

    tf_ptr_ = std::make_shared<TF>(node_ptr_->get_clock());
    tf_ptr_->setUsingDedicatedThread(true);
    robot_info_ptr_ = std::make_shared<mbf_utility::RobotInformation>(
      node_ptr_, tf_ptr_, "global_frame", "odom_frame", "robot_frame", rclcpp::Duration::from_seconds(1.0), "");

    mock_refiner_ptr_ = std::make_shared<AbstractPlanRefinerMock>();
    plan_refiner_execution_ptr_ = std::make_unique<AbstractPlanRefinerExecution>(
      "foo", mock_refiner_ptr_, robot_info_ptr_, node_ptr_);
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    plan_refiner_execution_ptr_->join();

    plan_refiner_execution_ptr_.reset();
    mock_refiner_ptr_.reset();

    robot_info_ptr_.reset();
    tf_ptr_.reset();
    node_ptr_.reset();

    rclcpp::shutdown();
  }

protected:
  std::vector<PoseStamped> plan_{PoseStamped{}};

  rclcpp::Node::SharedPtr node_ptr_;
  TFPtr tf_ptr_;
  mbf_utility::RobotInformation::Ptr robot_info_ptr_;

  std::shared_ptr<AbstractPlanRefinerMock> mock_refiner_ptr_;
  std::unique_ptr<AbstractPlanRefinerExecution> plan_refiner_execution_ptr_;
};

TEST_F(AbstractPlanRefinerExecutionFixture, success)
{
  initRosNode();

  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _)).WillOnce(Return(0));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::REFINED_PLAN);
}

TEST_F(AbstractPlanRefinerExecutionFixture, cancel)
{
  initRosNode();

  ON_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _)).WillByDefault(Return(11));
  EXPECT_CALL(*mock_refiner_ptr_, cancel()).Times(1).WillOnce(Return(true));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  ASSERT_TRUE(plan_refiner_execution_ptr_->cancel());

  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::CANCELED);
}

TEST_F(AbstractPlanRefinerExecutionFixture, max_retries)
{
  const int max_retries = 5;
  initRosNode(rclcpp::NodeOptions().append_parameter_override("refiner_max_retries", max_retries));

  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _))
  .Times(1 + max_retries)
  .WillRepeatedly(Return(11));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::MAX_RETRIES);
}

TEST_F(AbstractPlanRefinerExecutionFixture, success_after_retries)
{
  const int max_retries = 5;
  initRosNode(rclcpp::NodeOptions().append_parameter_override("refiner_max_retries", max_retries));

  InSequence seq;
  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _))
  .Times(max_retries)
  .WillRepeatedly(Return(11));
  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _)).Times(1).WillOnce(Return(1));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::REFINED_PLAN);
}

TEST_F(AbstractPlanRefinerExecutionFixture, no_plan_found_zero_retries)
{
  initRosNode(rclcpp::NodeOptions().append_parameter_override("refiner_max_retries", 0));

  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _)).Times(1).WillOnce(Return(11));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::NO_PLAN_FOUND);
}

TEST_F(AbstractPlanRefinerExecutionFixture, sets_refined_plan_and_error_outputs)
{
  initRosNode();

  std::vector<PoseStamped> refined_plan(4);
  refined_plan[0].pose.position.x = 0.0;
  refined_plan[1].pose.position.x = 1.0;
  refined_plan[2].pose.position.x = 2.0;
  refined_plan[3].pose.position.x = 3.0;
  const float position_error = 0.1f;
  const float orientation_error = 0.2f;
  const float path_length_ratio = 0.9f;

  EXPECT_CALL(*mock_refiner_ptr_, applyRefinement(_, _, _, _, _, _))
  .WillOnce(
    DoAll(
      SetArgReferee<1>(refined_plan),
      SetArgReferee<2>(position_error),
      SetArgReferee<3>(orientation_error),
      SetArgReferee<4>(path_length_ratio),
      Return(0)));

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::REFINED_PLAN);
  ASSERT_EQ(plan_refiner_execution_ptr_->getPlan().size(), refined_plan.size());
  ASSERT_FLOAT_EQ(plan_refiner_execution_ptr_->getPositionError(), position_error);
  ASSERT_FLOAT_EQ(plan_refiner_execution_ptr_->getOrientationError(), orientation_error);
  ASSERT_FLOAT_EQ(plan_refiner_execution_ptr_->getPathLengthRatio(), path_length_ratio);
}

ACTION(ThrowException)
{
  throw std::runtime_error("bad plan refiner");
}

TEST_F(AbstractPlanRefinerExecutionFixture, exception)
{
  initRosNode();

  EXPECT_CALL(
    *mock_refiner_ptr_,
    applyRefinement(_, _, _, _, _, _)).Times(1).WillOnce(ThrowException());

  ASSERT_TRUE(plan_refiner_execution_ptr_->start(plan_));
  plan_refiner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));

  ASSERT_EQ(plan_refiner_execution_ptr_->getState(), AbstractPlanRefinerExecution::INTERNAL_ERROR);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
