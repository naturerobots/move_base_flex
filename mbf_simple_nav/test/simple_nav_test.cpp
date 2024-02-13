#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mbf_simple_nav/simple_navigation_server.h>
#include <mbf_test_utility/robot_simulator.hpp>
#include <optional>
#include <rclcpp_action/client.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace ::testing;

struct SimpleNavTest : public Test
{
protected:
  SimpleNavTest()
  : default_node_options_(rclcpp::NodeOptions()
      .append_parameter_override("global_frame", "odom")
      .append_parameter_override("planners", std::vector<std::string> {"test_planner"})
      .append_parameter_override("test_planner.type", "mbf_simple_nav/TestPlanner")
      .append_parameter_override("controllers", std::vector<std::string> {"test_controller"})
      .append_parameter_override("test_controller.type", "mbf_simple_nav/TestController")
      .append_parameter_override("recovery_behaviors", std::vector<std::string> {"test_recovery"})
      .append_parameter_override("test_recovery.type", "mbf_simple_nav/TestRecovery"))
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ptr_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    robot_sim_node_ptr_ = std::make_shared<mbf_test_utility::RobotSimulator>();
    executor_ptr_->add_node(robot_sim_node_ptr_);
    // node with simple navigation server will be added later, in a method called from the individual test functions (to allow for setting parameter overrides)

    get_path_goal_.planner = "test_planner";
    get_path_goal_.use_start_pose = true;
    get_path_goal_.start_pose.header.frame_id = "odom";
    // start pose starts at position [0,0,0]
    get_path_goal_.target_pose.header.frame_id = "odom";
    get_path_goal_.target_pose.pose.position.x = 10;
    get_path_goal_.target_pose.pose.position.y = -3;

    exe_path_goal_.controller = "test_controller";
    exe_path_goal_.angle_tolerance = 0.1;
    exe_path_goal_.dist_tolerance = 0.01;
    exe_path_goal_.path.header.frame_id = "odom";
    exe_path_goal_.path.poses.push_back(get_path_goal_.start_pose);
    exe_path_goal_.path.poses.push_back(get_path_goal_.target_pose);

    recovery_goal_.behavior = "test_recovery";
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via
  // launch file for example)
  void initRosNode(rclcpp::NodeOptions node_options)
  {
    nav_server_node_ptr_ =
      std::make_shared<rclcpp::Node>("simple_nav", "", node_options);
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(nav_server_node_ptr_->get_clock());
    tf_buffer_ptr_->setUsingDedicatedThread(true);
    nav_server_ptr_ = std::make_shared<mbf_simple_nav::SimpleNavigationServer>(
      tf_buffer_ptr_,
      nav_server_node_ptr_);
    action_client_get_path_ptr_ = rclcpp_action::create_client<mbf_msgs::action::GetPath>(
      nav_server_node_ptr_,
      "simple_nav/get_path");
    action_client_exe_path_ptr_ = rclcpp_action::create_client<mbf_msgs::action::ExePath>(
      nav_server_node_ptr_,
      "simple_nav/exe_path");
    action_client_recovery_ptr_ = rclcpp_action::create_client<mbf_msgs::action::Recovery>(
      nav_server_node_ptr_,
      "simple_nav/recovery");
    executor_ptr_->add_node(nav_server_node_ptr_);
  }

  // Helper function that runs execution until given future completes. Test will abort if future doesn't complete until timeout.
  template<typename FutureT>
  void spin_until_future_complete(const FutureT & future)
  {
    ASSERT_EQ(
      executor_ptr_->spin_until_future_complete(
        future, std::chrono::seconds(
          5)), rclcpp::FutureReturnCode::SUCCESS);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    nav_server_ptr_.reset();
    tf_buffer_ptr_.reset();
    nav_server_node_ptr_.reset();
    robot_sim_node_ptr_.reset();
  }

  std::shared_ptr<mbf_simple_nav::SimpleNavigationServer> nav_server_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr nav_server_node_ptr_;
  rclcpp::Node::SharedPtr robot_sim_node_ptr_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_ptr_;

  // default parameterization for the tests
  const rclcpp::NodeOptions default_node_options_;

  // action clients and goals:
  std::shared_ptr<rclcpp_action::Client<mbf_msgs::action::GetPath>> action_client_get_path_ptr_;
  mbf_msgs::action::GetPath::Goal get_path_goal_;
  std::shared_ptr<rclcpp_action::Client<mbf_msgs::action::ExePath>> action_client_exe_path_ptr_;
  mbf_msgs::action::ExePath::Goal exe_path_goal_;
  std::shared_ptr<rclcpp_action::Client<mbf_msgs::action::Recovery>> action_client_recovery_ptr_;
  mbf_msgs::action::Recovery::Goal recovery_goal_;
  std::shared_ptr<rclcpp_action::Client<mbf_msgs::action::MoveBase>> action_client_move_base_ptr_;
};

TEST_F(SimpleNavTest, rejectsGetPathGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions()); // default node options without any parameter -> no plugins configured
  const auto goal_handle = action_client_get_path_ptr_->async_send_goal(get_path_goal_);
  spin_until_future_complete(goal_handle);
  EXPECT_THAT(goal_handle.get(), IsNull()); // goal rejection is expressed by returning a nullptr (by rclcpp_action client)
}

TEST_F(SimpleNavTest, rejectsExePathGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions());
  const auto goal_handle = action_client_exe_path_ptr_->async_send_goal(exe_path_goal_);
  spin_until_future_complete(goal_handle);
  EXPECT_THAT(goal_handle.get(), IsNull());
}

TEST_F(SimpleNavTest, rejectsRecoveryGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions());
  const auto goal_handle = action_client_recovery_ptr_->async_send_goal(recovery_goal_);
  spin_until_future_complete(goal_handle);
  EXPECT_THAT(goal_handle.get(), IsNull());
}

// TODO re-enable test
// TEST_F(SimpleNavTest, acceptsGoalsAfterLoadingTestPlugins)
// {
//   initRosNode(default_node_options_);
//   mbf_msgs::action::GetPath::Goal planner_goal;
//   planner_goal.planner = "test_planner";
//   const auto goal_handle = action_client_get_path_ptr_->async_send_goal(planner_goal);
//   ASSERT_EQ(
//     executor_ptr_->spin_until_future_complet(
//       goal_handle,
//       std::chrono::milliseconds(100)), rclcpp::FutureReturnCode::SUCCESS);
//   EXPECT_THAT(goal_handle.get(), NotNull()); // goal was not rejected
// }
// TODO add analogous test for exe path and recovery

TEST_F(SimpleNavTest, getPathReturnsPlan)
{
  initRosNode(default_node_options_);
  const auto goal_handle = action_client_get_path_ptr_->async_send_goal(get_path_goal_);
  spin_until_future_complete(goal_handle);
  const auto future_result = action_client_get_path_ptr_->async_get_result(goal_handle.get());
  spin_until_future_complete(future_result);
  const mbf_msgs::action::GetPath::Result::SharedPtr result_ptr = future_result.get().result;
  EXPECT_EQ(result_ptr->outcome, mbf_msgs::action::GetPath::Result::SUCCESS);
  EXPECT_EQ(result_ptr->path.poses[0], get_path_goal_.start_pose);
  EXPECT_EQ(result_ptr->path.poses[result_ptr->path.poses.size() - 1], get_path_goal_.target_pose);
}

TEST_F(SimpleNavTest, exePathMovesRobotToGoal)
{
  initRosNode(default_node_options_);
  exe_path_goal_.path.header.stamp = nav_server_node_ptr_->now();
  exe_path_goal_.path.poses[0].header.stamp = exe_path_goal_.path.header.stamp;
  exe_path_goal_.path.poses[1].header.stamp = exe_path_goal_.path.header.stamp;
  const auto goal_handle = action_client_exe_path_ptr_->async_send_goal(exe_path_goal_);
  spin_until_future_complete(goal_handle);
  const auto future_result = action_client_exe_path_ptr_->async_get_result(goal_handle.get());
  spin_until_future_complete(future_result);
  const mbf_msgs::action::ExePath::Result::SharedPtr result_ptr = future_result.get().result;
  EXPECT_EQ(result_ptr->outcome, mbf_msgs::action::ExePath::Result::SUCCESS);
  EXPECT_LE(result_ptr->dist_to_goal, exe_path_goal_.dist_tolerance);
}