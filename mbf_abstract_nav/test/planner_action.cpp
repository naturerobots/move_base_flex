#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_msgs/action/get_path.hpp>

#include <mbf_abstract_nav/planner_action.h>
#include <mbf_abstract_nav/abstract_planner_execution.h>

#include <string>

using namespace mbf_abstract_nav;

// in this test we will use an action server and an action client, since
// we cannot mock the goal-handle.

using geometry_msgs::msg::PoseStamped;
using mbf_abstract_core::AbstractPlanner;

// a mocked planner, allowing to control the outcome
struct MockPlanner : public AbstractPlanner
{
  MOCK_METHOD6(makePlan, uint32_t(const PoseStamped&, const PoseStamped&, double, std::vector<PoseStamped>&, double&,
                                  std::string&));

  // the cancel call is for now uninteresting. if this changes, we can also mock it.
  bool cancel()
  {
    return true;
  }
};

using mbf_abstract_nav::AbstractPlannerExecution;
using mbf_msgs::action::GetPath;

using testing::_;
using testing::DoAll;
using testing::Eq;
using testing::Field;
using testing::Return;
using testing::SetArgReferee;
using testing::Test;
using testing::MatcherCast;
using testing::Pointee;
using testing::SafeMatcherCast;

// a mocked action server
struct MockedActionServer : public rclcpp_action::Server<GetPath>
{
  // define the types we are using
  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<GetPath>> GoalHandlePtr;

  MockedActionServer(const rclcpp::Node::SharedPtr& node, GoalCallback goal_cb, CancelCallback cancel_cb, AcceptedCallback call_action_cb)
    : rclcpp_action::Server<GetPath>(
        node->get_node_base_interface(),
        node->get_node_clock_interface(),
        node->get_node_logging_interface(),
        "mocked_test_server", rcl_action_server_get_default_options(),
        goal_cb, cancel_cb, call_action_cb)
  { }

  // the mocked method
  MOCK_METHOD2(publish_result, void(const rclcpp_action::GoalUUID &, std::shared_ptr<void>));
};

// action which will trigger our condition variable, so we can wait until the runImpl thread reaches makePlan
ACTION_P(Notify, cv)
{
  cv->notify_all();
}

// test-fixture
struct PlannerActionFixture : public Test
{
  PlannerActionFixture()
    : node_(new rclcpp::Node("planner_action_test"))
    , tf_(new TF(node_->get_clock()))
    , planner_(new MockPlanner())
    , global_frame_("global_frame")
    , local_frame_("local_frame")
    , robot_info_(node_, tf_, global_frame_, local_frame_, rclcpp::Duration::from_seconds(0.0))
    , planner_execution_(new AbstractPlannerExecution("plugin", planner_, robot_info_, node_))
    , planner_action_(node_, "action_name", robot_info_)
    , action_server_(new MockedActionServer(node_, 
                    std::bind(&PlannerActionFixture::acceptGoal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&PlannerActionFixture::cancelAction, this, std::placeholders::_1),
                    std::bind(&PlannerActionFixture::callAction, this, std::placeholders::_1)))
    , goal_(new mbf_msgs::action::GetPath::Goal())
  {
    node_->get_node_waitables_interface()->add_waitable(action_server_, nullptr);
    tf_->setUsingDedicatedThread(true);
    node_->set_parameter(rclcpp::Parameter("planner_patience", 0.0));
  }

  void TearDown()
  {
  }

  rclcpp_action::GoalResponse acceptGoal(const rclcpp_action::GoalUUID& uuid, mbf_msgs::action::GetPath::Goal::ConstSharedPtr goal) 
  {
   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelAction(MockedActionServer::GoalHandlePtr goal_handle)
  {
    planner_action_.cancel(goal_handle);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void callAction(MockedActionServer::GoalHandlePtr goal_handle)
  {
    planner_action_.start(goal_handle, planner_execution_);
  }

  rclcpp::Node::SharedPtr node_;
  TFPtr tf_;
  std::shared_ptr<MockPlanner> planner_;     ///< the mocked planner
  std::condition_variable done_condition_;  ///< cv triggered on makePlan
  std::string global_frame_;
  std::string local_frame_;
  mbf_utility::RobotInformation robot_info_;
  AbstractPlannerExecution::Ptr planner_execution_;
  PlannerAction planner_action_;
  std::shared_ptr<MockedActionServer> action_server_;
  mbf_msgs::action::GetPath::Goal::SharedPtr goal_;
};

MATCHER_P(GetPathOutcomeIs, expected_outcome, "") 
{
  // we get std::shared_ptr<void> from get_result
  std::shared_ptr<mbf_msgs::action::GetPath::Result> msgPtr = std::reinterpret_pointer_cast<mbf_msgs::action::GetPath::Result>(arg);
  return msgPtr->outcome == expected_outcome;
}

TEST_F(PlannerActionFixture, emptyPath)
{
  // goal with frames, so we can pass tf-lookup
  goal_->use_start_pose = true;
  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;

  // setup the expectation
  // we dont return anything here, so the outcome should be empty path
  EXPECT_CALL(*planner_, makePlan(_, _, _, _, _, _)).WillOnce(Return(0));

  EXPECT_CALL(*action_server_,
    publish_result(_, GetPathOutcomeIs(mbf_msgs::action::GetPath::Result::EMPTY_PATH))
    ).Times(1).WillOnce(Notify(&done_condition_));

  // tear down 
  // call the server - this is where the actual test call happens
  std::shared_ptr<void> goal = goal_;
  action_server_->execute_goal_request_received(goal);

  // wait until the cv gets triggered
  std::mutex m;
  std::unique_lock<std::mutex> lock(m);
  done_condition_.wait_for(lock, std::chrono::seconds(1));
}

//TEST_F(PlannerActionFixture, success)
//{
//  // create a dummy path
//  std::vector<geometry_msgs::msg::PoseStamped> path(10);
//  // set the frame such that we can skip tf
//  for (size_t ii = 0; ii != path.size(); ++ii)
//  {
//    path[ii].header.frame_id = global_frame_;
//    path[ii].pose.orientation.w = 1;
//  }
//
//  // goal with frames, so we can pass tf-lookup
//  goal_->use_start_pose = true;
//  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;
//
//  // setup the expectation
//  EXPECT_CALL(*planner_, makePlan(_, _, _, _, _, _)).WillOnce(DoAll(SetArgReferee<3>(path), Return(0)));
//  EXPECT_CALL(action_server_,
//              publish_result(_, Field(&mbf_msgs::action::GetPath::Result::outcome, Eq(mbf_msgs::action::GetPath::Result::SUCCESS))))
//      .Times(1)
//      .WillOnce(Notify(&done_condition_));
//}
//
//TEST_F(PlannerActionFixture, tfError)
//{
//  // create a dummy path
//  std::vector<geometry_msgs::msg::PoseStamped> path(10);
//  // set the frame such that we fail at the tf
//  for (size_t ii = 0; ii != path.size(); ++ii)
//    path[ii].header.frame_id = "unknown";
//
//  // setup the expectation - we succeed here
//  EXPECT_CALL(*planner_, makePlan(_, _, _, _, _, _)).WillOnce(DoAll(SetArgReferee<3>(path), Return(0)));
//  EXPECT_CALL(action_server_,
//              publish_result(_, Field(&mbf_msgs::action::GetPath::Result::outcome, Eq(mbf_msgs::action::GetPath::Result::TF_ERROR))))
//      .Times(1)
//      .WillOnce(Notify(&done_condition_));
//
//  // goal with frames, so we can pass tf-lookup
//  goal_->use_start_pose = true;
//  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;
//}
//
//TEST_F(PlannerActionFixture, noRobotPose)
//{
//  // test case where we fail to get a valid robot pose.
//  // in this case we will receive a TF_ERROR from the server.
//
//  // make us use the robot pose
//  goal_->use_start_pose = false;
//  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;
//
//  // set the expectation
//  EXPECT_CALL(action_server_,
//              publish_result(_, Field(&mbf_msgs::action::GetPath::Result::outcome, Eq(mbf_msgs::action::GetPath::Result::TF_ERROR))))
//      .Times(1)
//      .WillOnce(Notify(&done_condition_));
//}
//
//TEST_F(PlannerActionFixture, noPathFound)
//{
//  // test case where the planner fails.
//  // in this case we will receive NO_PLAN_FOUND from the server.
//  node_->set_parameter(rclcpp::Parameter("planner_max_retries", 0));
//
//  // valid goal
//  goal_->use_start_pose = true;
//  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;
//
//  // set the expectation: the planner returns a failure
//  EXPECT_CALL(*planner_, makePlan(_, _, _, _, _, _)).WillOnce(Return(mbf_msgs::action::GetPath::Result::NO_PATH_FOUND));
//  EXPECT_CALL(action_server_,
//              publish_result(_, Field(&mbf_msgs::action::GetPath::Result::outcome, Eq(mbf_msgs::action::GetPath::Result::NO_PATH_FOUND))))
//      .Times(1)
//      .WillOnce(Notify(&done_condition_));
//}
//
//ACTION(SleepAndFail)
//{
//  std::this_thread::sleep_for(std::chrono::milliseconds(10));
//  return 11;
//}
//
//TEST_F(PlannerActionFixture, patExceeded)
//{
//  // test case where the planner fails multiple times and we are out of patience
//
//  node_->set_parameter(rclcpp::Parameter("planner_max_retries", 5));
//  node_->set_parameter(rclcpp::Parameter("planner_patience", 0.001));
//
//  // valid goal
//  goal_->use_start_pose = true;
//  goal_->start_pose.header.frame_id = goal_->target_pose.header.frame_id = global_frame_;
//
//  // set the expectation: the planner returns a failure
//  EXPECT_CALL(*planner_, makePlan(_, _, _, _, _, _)).WillRepeatedly(SleepAndFail());
//  EXPECT_CALL(action_server_,
//              publish_result(_, Field(&mbf_msgs::action::GetPath::Result::outcome, Eq(mbf_msgs::action::GetPath::Result::PAT_EXCEEDED))))
//      .Times(1)
//      .WillOnce(Notify(&done_condition_));
//}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
