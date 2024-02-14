#include <mbf_simple_core/simple_controller.h>
#include <mbf_msgs/action/exe_path.hpp>

namespace mbf_simple_nav
{

//! Planner plugin for testing move base flex
class TestController : public mbf_simple_core::SimpleController
{
public:
  TestController() = default;
  virtual ~TestController() = default;

  virtual void initialize(
    const std::string name, ::TF * tf,
    const rclcpp::Node::SharedPtr & node_handle) override {}
  virtual bool cancel() override {}

  virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped> & plan) override
  {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger(
        "test_controller"),
      "Got plan in frame " << plan[0].header.frame_id);
    plan_ = plan;
    return true;
  }

  virtual uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & velocity,
    geometry_msgs::msg::TwistStamped & cmd_vel, std::string & message)
  {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger(
        "test_controller"),
      "Got new robot pose for t= " << pose.header.stamp.sec << "s, in frame " <<
        pose.header.frame_id);
    latest_robot_pose_ = pose;

    const auto & goal_position = plan_.back().pose.position;
    const auto & robot_position = latest_robot_pose_->pose.position;

    const double dist_x = abs(goal_position.x - robot_position.x);
    const double direction_x = robot_position.x < goal_position.x ? 1 : -1;

    const double dist_y = abs(goal_position.y - robot_position.y);
    const double direction_y = robot_position.y < goal_position.y ? 1 : -1;

    constexpr double speed_multiplier = 10;
    cmd_vel.twist.linear.x = direction_x * speed_multiplier * dist_x;
    cmd_vel.twist.linear.y = direction_y * speed_multiplier * dist_y;

    return mbf_msgs::action::ExePath::Result::SUCCESS;
  }

  virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance)
  {
    if (!latest_robot_pose_.has_value()) {
      return false;
    }
    const auto & goal_position = plan_.back().pose.position;
    const auto & robot_position = latest_robot_pose_->pose.position;
    const double xy_goal_distance_squared =
      (robot_position.x - goal_position.x) *
      (robot_position.x - goal_position.x) +
      (robot_position.y - goal_position.y) *
      (robot_position.y - goal_position.y);
    return xy_goal_distance_squared <= xy_tolerance * xy_tolerance;
  }

protected:
  std::vector<geometry_msgs::msg::PoseStamped> plan_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_robot_pose_;
};
}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestController, mbf_simple_core::SimpleController);
