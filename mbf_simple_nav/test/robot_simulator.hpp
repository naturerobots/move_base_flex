#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace mbf_simple_nav
{

/**
 * Tiny robot simulator: Takes cmd_vel msgs and updates a TF accordingly.
 * Useful for simulating a robot's movement in unit tests.
 */
class RobotSimulator : public rclcpp::Node
{
public:
  RobotSimulator()
    : Node("robot_simulator")
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
    , tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
    , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*tf_buffer_))
    , t_last_update_(now())
  {
    trf_parent_robot_.header.frame_id = config_.parent_frame_id;
    trf_parent_robot_.child_frame_id = config_.robot_frame_id;
    startUpdateRobotPoseTimer();
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& vel)
  {
    // TODO trf
    if (vel->header.frame_id != config_.parent_frame_id)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Dropping velocity msg, expecting frame id "
                                            << config_.parent_frame_id << ", got " << vel->header.frame_id);
    }
    current_velocity_ = vel->twist;
  };

  void startUpdateRobotPoseTimer()
  {
    update_robot_pose_timer_ = create_wall_timer(10ms, std::bind(&RobotSimulator::update_robot_pose_timer_, this));
  }

  void updateRobotPose()
  {
    const auto t_now = now();
    const rclcpp::Duration duration_since_last_update = t_now - t_last_update_;
    startUpdateRobotPoseTimer();  // restart timer for continuous updates
  };

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist current_velocity_;
  rclcpp::Time t_last_update_;
  rclcpp::TimerBase::UniquePtr update_robot_pose_timer_;
  geometry_msgs::msg::TransformStamped trf_parent_robot_;

  struct
  {
    std::string robot_frame_id = "base_link";
    std::string parent_frame_id = "odom";
  } config_;
};

}  // namespace mbf_simple_nav