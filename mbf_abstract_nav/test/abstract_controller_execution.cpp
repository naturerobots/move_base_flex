#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_abstract_nav/abstract_controller_execution.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>
#include <memory>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using mbf_abstract_core::AbstractController;
using mbf_abstract_nav::AbstractControllerExecution;
using testing::_;
using testing::Return;
using testing::Test;

// the plan as a vector of poses
typedef std::vector<PoseStamped> plan_t;

// mocked planner so we can control its output
struct AbstractControllerMock : public AbstractController
{
  // the mocked pure virtual members
  MOCK_METHOD4(computeVelocityCommands, uint32_t(const PoseStamped&, const TwistStamped&, TwistStamped&, std::string&));
  MOCK_METHOD2(isGoalReached, bool(double, double));
  MOCK_METHOD1(setPlan, bool(const plan_t&));
  MOCK_METHOD0(cancel, bool());
};

// fixture for our tests
struct AbstractControllerExecutionFixture : public Test
{
  AbstractControllerExecutionFixture() {};

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }


  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions, e.g. for global and robot frame.
  void initRosNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
  {
    node_ptr_ = std::make_shared<rclcpp::Node>("abstract_controller_execution_test", "", node_options);
    // suppress the logging since we don't want warnings to pollute the test-outcome
    node_ptr_->get_logger().set_level(rclcpp::Logger::Level::Fatal);
    vel_pub_ptr_ = node_ptr_->create_publisher<TwistStamped>("vel", 1);
    goal_pub_ptr_ = node_ptr_->create_publisher<PoseStamped>("pose", 1);
    tf_ptr_ = std::make_shared<TF>(node_ptr_->get_clock());
    tf_ptr_->setUsingDedicatedThread(true);
    robot_info_ptr_ = std::make_shared<mbf_utility::RobotInformation>(node_ptr_, tf_ptr_, "global_frame",
                                                                "robot_frame", rclcpp::Duration::from_seconds(1.0), "");

    mock_controller_ptr_ = std::make_shared<AbstractControllerMock>();
    controller_execution_ptr_ = std::make_unique<AbstractControllerExecution>("a name", AbstractController::Ptr(mock_controller_ptr_),
                                                                              robot_info_ptr_, vel_pub_ptr_, goal_pub_ptr_, node_ptr_);
  }


  void TearDown() override
  {
    // after every test we expect that moving_ is set to false
    // we don't expect this for the case NO_LOCAL_CMD
    if (controller_execution_ptr_->getState() != AbstractControllerExecution::NO_LOCAL_CMD) {
      EXPECT_FALSE(controller_execution_ptr_->isMoving());
    }

    // we have to stop the thread when the test is done
    controller_execution_ptr_->join();

    controller_execution_ptr_.reset();
    mock_controller_ptr_.reset();

    robot_info_ptr_.reset();
    tf_ptr_.reset();
    goal_pub_ptr_.reset();
    vel_pub_ptr_.reset();
    node_ptr_.reset();
  
    rclcpp::shutdown();
  }

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<TwistStamped>::SharedPtr vel_pub_ptr_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_ptr_;
  TFPtr tf_ptr_;
  mbf_utility::RobotInformation::Ptr robot_info_ptr_;

  std::shared_ptr<AbstractControllerMock> mock_controller_ptr_;
  std::unique_ptr<AbstractControllerExecution> controller_execution_ptr_;
};

TEST_F(AbstractControllerExecutionFixture, noPlan)
{
  // test checks the case where we call start() without setting a plan.
  // the expected output is NO_PLAN.
  initRosNode();

  // start the controller
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::NO_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, emptyPlan)
{
  // test checks the case where we pass an empty path to the controller.
  // the expected output is EMPTY_PLAN
  initRosNode();

  // set an empty plan
  controller_execution_ptr_->setNewPlan(plan_t{}, true, 1, 1);

  // start the controller
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::EMPTY_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, invalidPlan)
{
  // test checks the case where the controller recjets the plan.
  // the expected output is INVALID_PLAN
  initRosNode();

  // setup the expectation: the controller rejects the plan
  EXPECT_CALL(*mock_controller_ptr_, setPlan(_)).WillOnce(Return(false));
  // set a plan
  plan_t plan(10);
  controller_execution_ptr_->setNewPlan(plan, true, 1, 1);

  // start the controller
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::INVALID_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, internalError)
{
  // test checks the case where we cannot compute the current robot pose
  // the expected output is INTERNAL_ERROR

  // set the robot frame to some thing else then the global frame (for our test case)
  initRosNode(rclcpp::NodeOptions()
    .append_parameter_override("global_frame", "global_frame")
    .append_parameter_override("robot_frame", "not_global_frame")
  );

  // setup the expectation: the controller accepts the plan
  EXPECT_CALL(*mock_controller_ptr_, setPlan(_)).WillOnce(Return(true));

  // set a plan
  plan_t plan(10);
  controller_execution_ptr_->setNewPlan(plan, true, 1, 1);

  // start the controller
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  // note: this timeout must be longer than the default tf-timeout
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(2));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::INTERNAL_ERROR);
}

// fixture making us pass computeRobotPose()
struct ComputeRobotPoseFixture : public AbstractControllerExecutionFixture
{
  ComputeRobotPoseFixture () 
  : global_frame_("global_frame")
  , robot_frame_("robot_frame")
  {}

  void initRosNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
  {

    node_options.append_parameter_override("global_frame", global_frame_)
                .append_parameter_override("robot_frame", robot_frame_);
    AbstractControllerExecutionFixture::initRosNode(node_options);
    // setup the transform.
    const auto t_now = node_ptr_->now();
    TransformStamped transform;
    transform.header.stamp = t_now;
    transform.header.frame_id = global_frame_;
    transform.child_frame_id = robot_frame_;
    transform.transform.rotation.w = 1;
    // add transforms to the buffer such that move base flex can transform between global and robot frame for one second, starting now
    tf_ptr_->setTransform(transform, "test_tf_authority");
    transform.header.stamp = t_now + rclcpp::Duration::from_seconds(1.0);
    tf_ptr_->setTransform(transform, "test_tf_authority");
  }
protected: 
  const std::string global_frame_;
  const std::string robot_frame_;
};

TEST_F(ComputeRobotPoseFixture, arrivedGoal)
{
  // test checks the case where we reach the goal.
  // the expected output is ARRIVED_GOAL
  initRosNode();

  // setup the expectation: the controller accepts the plan and says we are arrived
  EXPECT_CALL(*mock_controller_ptr_, setPlan(_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_controller_ptr_, isGoalReached(_, _)).WillOnce(Return(true));

  // we compare against the back-pose of the plan
  plan_t plan(10);
  plan.back().header.frame_id = global_frame_;
  plan.back().pose.orientation.w = 1;

  // make the tolerances small
  controller_execution_ptr_->setNewPlan(plan, true, 1e-3, 1e-3);

  // call start
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::ARRIVED_GOAL);
}

ACTION(ControllerException)
{
  throw std::runtime_error("Oh no! Controller throws an Exception");
}

TEST_F(ComputeRobotPoseFixture, controllerException)
{
  initRosNode();
  // setup the expectation: the controller accepts the plan and says we are not arrived.
  // the controller throws then an exception
  EXPECT_CALL(*mock_controller_ptr_, setPlan(_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_controller_ptr_, isGoalReached(_, _)).WillOnce(Return(false));
  EXPECT_CALL(*mock_controller_ptr_, computeVelocityCommands(_, _, _, _)).WillOnce(ControllerException());

  // setup the plan
  plan_t plan(10);
  controller_execution_ptr_->setNewPlan(plan, true, 1e-3, 1e-3);

  // call start
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::INTERNAL_ERROR);
}

// fixture which will setup the mock such that we generate a controller failure
struct FailureFixture : public ComputeRobotPoseFixture
{
  void initRosNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
  {
    // call the parent method for the computeRobotPose call
    ComputeRobotPoseFixture::initRosNode(node_options);

    // setup the expectation: the controller accepts the plan and says we are not arrived.
    // it furhermore returns an error code
    EXPECT_CALL(*mock_controller_ptr_, setPlan(_)).WillOnce(Return(true));
    EXPECT_CALL(*mock_controller_ptr_, isGoalReached(_, _)).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock_controller_ptr_, computeVelocityCommands(_, _, _, _)).WillRepeatedly(Return(11));

    // setup the plan
    plan_t plan(10);
    controller_execution_ptr_->setNewPlan(plan, true, 1e-3, 1e-3);
  }
};

TEST_F(FailureFixture, maxRetries)
{
  // test verifies the case where we exceed the max-retries.
  // the expected output is MAX_RETRIES

  // enable the retries logic (max_retries > 0)
  initRosNode(rclcpp::NodeOptions().append_parameter_override("controller_max_retries", 1));

  // call start
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update: in first iteration NO_LOCAL_CMD
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::NO_LOCAL_CMD);

  // wait for the status update: in second iteration MAX_RETRIES
  // bcs max_retries_ > 0 && ++retries > max_retries_
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::MAX_RETRIES);
}

TEST_F(FailureFixture, noValidCmd)
{
  // test verifies the case where we don't exceed the patience or max-retries conditions
  // the expected output is NO_VALID_CMD

  // disable the retries logic
  initRosNode(rclcpp::NodeOptions().append_parameter_override("controller_max_retries", -1));

  // call start
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::NO_LOCAL_CMD);
}

TEST_F(FailureFixture, patExceeded)
{
  // test verifies the case where we exceed the patience
  // the expected output is PAT_EXCEEDED

  // disable the retries logic and enable the patience logic: we cheat by setting it to a negative duration.
  initRosNode(rclcpp::NodeOptions()
    .append_parameter_override("controller_max_retries", -1)
    .append_parameter_override("controller_patience", -1e-3)
  );

  // call start
  ASSERT_TRUE(controller_execution_ptr_->start());

  // wait for the status update
  controller_execution_ptr_->waitForStateUpdate(std::chrono::seconds(1));
  ASSERT_EQ(controller_execution_ptr_->getState(), AbstractControllerExecution::PAT_EXCEEDED);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
