#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mbf_simple_nav/simple_navigation_server.h>
#include <mbf_test_utility/robot_simulator.hpp>
#include <optional>
#include <rclcpp_action/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

using namespace ::testing;

/*
 * Tests integration of move base simple nav modules (and, by extension, the abstract nav modules).
 *
 * The test consists of two nodes:
 *   - The simple nav server node, to which we send action goals during the tests (just like users would do)
 *   - The robot simulator node, which publishes TFs and receives cmd_vel data to move it around
 *
 * In addition, the test uses test plugins that have a very basic implementation of the respective action.
 */
struct SimpleNavIntegrationTest : public Test
{
protected:
  SimpleNavIntegrationTest()
  : default_node_options_(rclcpp::NodeOptions()
      .append_parameter_override("global_frame", "odom")
      .append_parameter_override("odom_topic", "") // disable warning
      .append_parameter_override("planners", std::vector<std::string> {"test_planner"})
      .append_parameter_override("test_planner.type", "mbf_simple_nav/TestPlanner")
      .append_parameter_override("controllers", std::vector<std::string> {"test_controller"})
      .append_parameter_override("test_controller.type", "mbf_simple_nav/TestController")
      .append_parameter_override("recovery_behaviors", std::vector<std::string> {"test_recovery"})
      .append_parameter_override("test_recovery.type", "mbf_simple_nav/TestRecovery")
      .append_parameter_override("test_recovery.sim_server", "/robot_simulator"))
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ptr_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    robot_sim_node_ptr_ = std::make_shared<mbf_test_utility::RobotSimulator>(
      "robot_simulator",
      rclcpp::NodeOptions().arguments({"--ros-args", "-r", "~/cmd_vel:=/simple_nav/cmd_vel"})); // connect cmd_vel to simple_nav publisher
    executor_ptr_->add_node(robot_sim_node_ptr_);
    // node with simple navigation server will be added later, in a method called from the individual test functions (to allow for setting parameter overrides)

    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(robot_sim_node_ptr_->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

    get_path_goal_.planner = "test_planner";
    get_path_goal_.use_start_pose = true;
    get_path_goal_.start_pose.header.frame_id = "odom";
    // start pose starts at position [0,0,0]
    get_path_goal_.target_pose.header.frame_id = "odom";
    get_path_goal_.target_pose.pose.position.x = 0.7;
    get_path_goal_.target_pose.pose.position.y = -1.3;

    exe_path_goal_.controller = "test_controller";
    exe_path_goal_.angle_tolerance = 0.1;
    exe_path_goal_.dist_tolerance = 0.01;
    exe_path_goal_.tolerance_from_action = true;
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

  // Helper function that runs execution until given future completes. Returns false if future didn't return in time.
  // Suggested use: ASSERT_TRUE(spin_until_future_complete(...)) to abort the test if something goes wrong with completing the future.
  template<typename FutureT>
  bool spin_until_future_complete(const FutureT & future)
  {
    return executor_ptr_->spin_until_future_complete(
      future,
      std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS;
  }

  void TearDown() override
  {
    // nav_server needs to be destructed before we can call rclcpp::shutdown.
    // Otherwise, we might still have goal handles that crash when they reach their terminal state, because the goal doesn't exist at the lower rcl layer.
    nav_server_ptr_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<mbf_simple_nav::SimpleNavigationServer> nav_server_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
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

TEST_F(SimpleNavIntegrationTest, rejectsGetPathGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions()); // node options without any parameter -> no plugins configured
  const auto goal_handle = action_client_get_path_ptr_->async_send_goal(get_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), IsNull()); // goal rejection is expressed by returning a nullptr (by rclcpp_action client)
}

TEST_F(SimpleNavIntegrationTest, rejectsExePathGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions());
  const auto goal_handle = action_client_exe_path_ptr_->async_send_goal(exe_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), IsNull());
}

TEST_F(SimpleNavIntegrationTest, rejectsRecoveryGoalWhenNoPluginIsLoaded)
{
  initRosNode(rclcpp::NodeOptions());
  const auto goal_handle = action_client_recovery_ptr_->async_send_goal(recovery_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), IsNull());
}

TEST_F(SimpleNavIntegrationTest, acceptsGetPathGoalAfterLoadingTestPlugins)
{
  initRosNode(default_node_options_);
  const auto goal_handle = action_client_get_path_ptr_->async_send_goal(get_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), NotNull()); // goal was not rejected
}

TEST_F(SimpleNavIntegrationTest, acceptsExePathGoalAfterLoadingTestPlugins)
{
  initRosNode(default_node_options_);
  const auto goal_handle = action_client_exe_path_ptr_->async_send_goal(exe_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), NotNull());
  // wait for goal completion before destructing the test fixture to save time during testing, even though it's not part of this test:
  // otherwise, depending on the execution order, the exe_path action execution thread might wait 3sec for a tf that won't come, because the robot sim node is already gone.
  spin_until_future_complete(action_client_exe_path_ptr_->async_get_result(goal_handle.get()));
}

TEST_F(SimpleNavIntegrationTest, acceptsRecoveryGoalAfterLoadingTestPlugins)
{
  initRosNode(default_node_options_);
  const auto goal_handle = action_client_recovery_ptr_->async_send_goal(recovery_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  EXPECT_THAT(goal_handle.get(), NotNull());
  spin_until_future_complete(action_client_recovery_ptr_->async_get_result(goal_handle.get()));
}

TEST_F(SimpleNavIntegrationTest, getPathReturnsPlan)
{
  initRosNode(default_node_options_);
  const auto goal_handle = action_client_get_path_ptr_->async_send_goal(get_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  const auto future_result = action_client_get_path_ptr_->async_get_result(goal_handle.get());
  ASSERT_TRUE(spin_until_future_complete(future_result));
  const mbf_msgs::action::GetPath::Result::SharedPtr result_ptr = future_result.get().result;
  EXPECT_EQ(result_ptr->outcome, mbf_msgs::action::GetPath::Result::SUCCESS);
  EXPECT_EQ(result_ptr->path.poses[0], get_path_goal_.start_pose);
  EXPECT_EQ(result_ptr->path.poses[result_ptr->path.poses.size() - 1], get_path_goal_.target_pose);
}

TEST_F(SimpleNavIntegrationTest, exePathReachesTheGoal)
{
  initRosNode(default_node_options_);

  // start exe path action, then wait until it finishes
  const auto goal_handle = action_client_exe_path_ptr_->async_send_goal(exe_path_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  const auto future_result = action_client_exe_path_ptr_->async_get_result(goal_handle.get());
  ASSERT_TRUE(spin_until_future_complete(future_result));
  const mbf_msgs::action::ExePath::Result::SharedPtr result_ptr = future_result.get().result;

  // check that the action succeeded and that the robot actually arrived at the chosen goal location
  EXPECT_EQ(result_ptr->outcome, mbf_msgs::action::ExePath::Result::SUCCESS);
  EXPECT_LE(result_ptr->dist_to_goal, exe_path_goal_.dist_tolerance);
  const geometry_msgs::msg::TransformStamped trf_odom_baseLink = tf_buffer_ptr_->lookupTransform(
    "odom",
    "base_link",
    tf2::TimePointZero);
  const auto & robot_position = trf_odom_baseLink.transform.translation;
  const auto & goal_position = exe_path_goal_.path.poses.back().pose.position;
  EXPECT_NEAR(robot_position.x, goal_position.x, exe_path_goal_.dist_tolerance);
  EXPECT_NEAR(robot_position.y, goal_position.y, exe_path_goal_.dist_tolerance);
}

TEST_F(SimpleNavIntegrationTest, recoveryTriggersBehavior)
{
  initRosNode(default_node_options_);

  robot_sim_node_ptr_->set_parameter(rclcpp::Parameter("is_robot_stuck", true));
  ASSERT_TRUE(robot_sim_node_ptr_->get_parameter("is_robot_stuck").as_bool());

  // start recovery action, then wait until it finishes
  const auto goal_handle = action_client_recovery_ptr_->async_send_goal(recovery_goal_);
  ASSERT_TRUE(spin_until_future_complete(goal_handle));
  const auto future_result = action_client_recovery_ptr_->async_get_result(goal_handle.get());
  ASSERT_TRUE(spin_until_future_complete(future_result));
  const mbf_msgs::action::Recovery::Result::SharedPtr result_ptr = future_result.get().result;

  // check that the action succeeded and that the robot actually got unstuck
  EXPECT_EQ(result_ptr->outcome, mbf_msgs::action::Recovery::Result::SUCCESS);
  EXPECT_FALSE(robot_sim_node_ptr_->get_parameter("is_robot_stuck").as_bool());
}
