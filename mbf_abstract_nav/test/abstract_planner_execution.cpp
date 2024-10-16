#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_abstract_nav/abstract_planner_execution.h>

// too long namespaces...
using geometry_msgs::msg::PoseStamped;
using mbf_abstract_core::AbstractPlanner;

// mocked version of a planner
// we will control the output of it
struct AbstractPlannerMock : public AbstractPlanner
{
  MOCK_METHOD6(makePlan, uint32_t(const PoseStamped&, const PoseStamped&, double, std::vector<PoseStamped>&, double&,
                                  std::string&));

  MOCK_METHOD0(cancel, bool());
};

using mbf_abstract_nav::AbstractPlannerExecution;
using testing::_;
using testing::AtLeast;
using testing::InSequence;
using testing::Return;
using testing::Test;

// setup the test-fixture
struct AbstractPlannerExecutionFixture : public Test
{
  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions, e.g. for global and robot frame.
  void initRosNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
  {
    node_ptr_ = std::make_shared<rclcpp::Node>("abstract_planner_execution_test", "", node_options);
    // suppress the logging since we don't want warnings to pollute the test-outcome
    node_ptr_->get_logger().set_level(rclcpp::Logger::Level::Fatal);
    tf_ptr_ = std::make_shared<TF>(node_ptr_->get_clock());
    tf_ptr_->setUsingDedicatedThread(true);
    robot_info_ptr_ = std::make_shared<mbf_utility::RobotInformation>(node_ptr_, tf_ptr_, "global_frame",
                                                                "robot_frame", rclcpp::Duration::from_seconds(1.0), "");

    mock_planner_ptr_ = std::make_shared<AbstractPlannerMock>();
    planner_execution_ptr_ = std::make_unique<AbstractPlannerExecution>("foo", mock_planner_ptr_,
                               robot_info_ptr_, node_ptr_);

  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    // we have to stop the thread when the test is done
    planner_execution_ptr_->join();

    planner_execution_ptr_.reset();
    mock_planner_ptr_.reset();

    robot_info_ptr_.reset();
    tf_ptr_.reset();
    node_ptr_.reset();

    rclcpp::shutdown();
  }

protected:
  PoseStamped pose_;  // dummy pose_ to call start

  rclcpp::Node::SharedPtr node_ptr_;
  TFPtr tf_ptr_;
  mbf_utility::RobotInformation::Ptr robot_info_ptr_;

  std::shared_ptr<AbstractPlannerMock> mock_planner_ptr_;
  std::unique_ptr<AbstractPlannerExecution> planner_execution_ptr_;
};

TEST_F(AbstractPlannerExecutionFixture, success)
{
  initRosNode();

  // the good case - we succeed
  // setup the expectation
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).WillOnce(Return(0));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::FOUND_PLAN);
}

ACTION_P(Wait, cv)
{
  std::mutex m;
  std::unique_lock<std::mutex> lock(m);
  cv->wait(lock);
  return 11;
}

TEST_F(AbstractPlannerExecutionFixture, cancel)
{
  initRosNode();

  // the cancel case. we simulate that we cancel the execution
  // setup the expectation
  ON_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).WillByDefault(Return(11));
  EXPECT_CALL(*mock_planner_ptr_, cancel()).Times(1).WillOnce(Return(true));

  // now call the method
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));
  ASSERT_TRUE(planner_execution_ptr_->cancel());

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::CANCELED);
}

TEST_F(AbstractPlannerExecutionFixture, reconfigure)
{
  initRosNode();

  ASSERT_EQ(planner_execution_ptr_->getFrequency(), 0.0); // default value, after param declaration
  const auto set_param_result = node_ptr_->set_parameter(rclcpp::Parameter("planner_frequency", 1.0));
  EXPECT_TRUE(set_param_result.successful);
  EXPECT_EQ(planner_execution_ptr_->getFrequency(), 1.0); // changed value, should be set via the reconfigure callback that is called by ROS 2 when params change
}

TEST_F(AbstractPlannerExecutionFixture, max_retries)
{
  initRosNode();

  // we expect that if the planner fails for 1 + max_retries times, that
  // the class returns MAX_RETRIES

  // configure the class
  const int max_retries = 5;
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", max_retries)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 100.0)).successful);

  // setup the expectations
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1 + max_retries).WillRepeatedly(Return(11));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::MAX_RETRIES);
}

TEST_F(AbstractPlannerExecutionFixture, success_after_retries)
{
  initRosNode();

  // we expect that if the planner fails for 1 + (max_retries - 1) times and then succeeds, that
  // the class returns FOUND_PLAN

  // configure the class
  const int max_retries = 5;
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", max_retries)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 100.0)).successful);

  // setup the expectations
  InSequence seq;
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(max_retries).WillRepeatedly(Return(11));
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Return(1));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // wait for the patience to elapse and check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::FOUND_PLAN);
}

TEST_F(AbstractPlannerExecutionFixture, no_plan_found_zero_patience)
{
  initRosNode();

  // if no retries and no patience are configured, we return NO_PLAN_FOUND on
  // planner failure

  // configure the class
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", 0)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 0.0)).successful);

  // setup the expectations
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Return(11));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::NO_PLAN_FOUND);
}

TEST_F(AbstractPlannerExecutionFixture, no_plan_found_non_zero_patience)
{
  initRosNode();

  // if no retries and a large patience are configured, we return NO_PLAN_FOUND on
  // planner failure

  // configure the class
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", 0)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 1.0)).successful);

  // setup the expectations
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Return(11));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::NO_PLAN_FOUND);
}

using testing::DoAll;
using testing::SetArgReferee;

TEST_F(AbstractPlannerExecutionFixture, sumDist)
{
  initRosNode();

  // simulate the case when the planner returns zero cost
  std::vector<PoseStamped> plan(4);
  for (size_t ii = 0; ii != plan.size(); ++ii)
    plan.at(ii).pose.position.x = ii;
  double cost = 0;

  // call the planner
  // the good case - we succeed
  // setup the expectation
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _))
      .WillOnce(DoAll(SetArgReferee<3>(plan), SetArgReferee<4>(cost), Return(0)));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::FOUND_PLAN);
  ASSERT_EQ(planner_execution_ptr_->getCost(), 3);
}

TEST_F(AbstractPlannerExecutionFixture, patience_exceeded_waiting_for_planner_response)
{
  initRosNode();

  // if makePlan does not return before the patience times out, we return PAT_EXCEEDED

  // configure the class
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", 0)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 0.1)).successful);

  // setup the expectations
  std::condition_variable cv;
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Wait(&cv));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // wait for the patience to elapse
  std::this_thread::sleep_for(std::chrono::milliseconds{ 200 });
  cv.notify_all();

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::PAT_EXCEEDED);
}

TEST_F(AbstractPlannerExecutionFixture, patience_exceeded_infinite_retries)
{
  initRosNode();

  // if negative retries are configured, we expect makePlan to repeatedly get called, and PAT_EXCEEDED to be returned
  // once the patience is exceeded

  // configure the class
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_max_retries", -1)).successful);
  ASSERT_TRUE(node_ptr_->set_parameter(rclcpp::Parameter("planner_patience", 0.5)).successful);

  // setup the expectations
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(AtLeast(10)).WillRepeatedly(Return(11));

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // wait for the patience to elapse and check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::PAT_EXCEEDED);
}

ACTION(ThrowException)
{
  throw std::runtime_error("bad planner");
}

TEST_F(AbstractPlannerExecutionFixture, exception)
{
  initRosNode();

  // if we throw an exception, we expect that we can recover from it
  // setup the expectations
  EXPECT_CALL(*mock_planner_ptr_, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(ThrowException());

  // call and wait
  ASSERT_TRUE(planner_execution_ptr_->start(pose_, pose_, 0));

  // check result
  ASSERT_EQ(planner_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1)), std::cv_status::no_timeout);
  ASSERT_EQ(planner_execution_ptr_->getState(), AbstractPlannerExecution::INTERNAL_ERROR);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
