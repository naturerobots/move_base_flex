#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel)
  {
    if (vel->header.frame_id != config_.robot_frame_id)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Dropping velocity msg. Node expects velocities in robot frame ("
                                            << config_.robot_frame_id << "(, but got frame " << vel->header.frame_id);
    }
    current_velocity_ = vel->twist;
  }

  RobotSimulator()
    : Node("robot_simulator")
    , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    trf_parent_robot_.header.stamp = now();
    trf_parent_robot_.header.frame_id = config_.parent_frame_id;
    trf_parent_robot_.child_frame_id = config_.robot_frame_id;
    startUpdateRobotPoseTimer();
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, std::bind(&RobotSimulator::velocityCallback, this, std::placeholders::_1));
  }

  void startUpdateRobotPoseTimer()
  {
    update_robot_pose_timer_ = create_wall_timer(10ms, std::bind(&RobotSimulator::updateRobotPose, this));
  }

  void updateRobotPose()
  {
    const auto t_now = now();

    // Calculate how much the has robot moved, based on current_velocity_ (assuming constant velocity)
    const double seconds_since_last_update = (t_now - trf_parent_robot_.header.stamp).seconds();
    tf2::Quaternion orientationTrf_robotTLastUpdate_robotTNow;
    orientationTrf_robotTLastUpdate_robotTNow.setRPY(current_velocity_.angular.x * seconds_since_last_update,
                                                     current_velocity_.angular.y * seconds_since_last_update,
                                                     current_velocity_.angular.z * seconds_since_last_update);
    const tf2::Transform trf_robotTLastUpdate_robotTNow(
        orientationTrf_robotTLastUpdate_robotTNow,
        tf2::Vector3(current_velocity_.linear.x * seconds_since_last_update,
                     current_velocity_.linear.y * seconds_since_last_update,
                     current_velocity_.linear.z * seconds_since_last_update));

    // Updated robot pose member and publish tf
    tf2::Transform trf_parent_robotTLastUpdate;
    tf2::fromMsg(trf_parent_robot_.transform, trf_parent_robotTLastUpdate);
    const tf2::Transform trf_parent_robotTNow = trf_parent_robotTLastUpdate * trf_robotTLastUpdate_robotTNow;
    tf2::toMsg(trf_parent_robotTNow, trf_parent_robot_.transform);
    trf_parent_robot_.header.stamp = t_now;
    tf_broadcaster_->sendTransform(trf_parent_robot_);

    startUpdateRobotPoseTimer();  // restart timer for continuous updates
  };

protected:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;

  geometry_msgs::msg::Twist current_velocity_;
  rclcpp::TimerBase::SharedPtr update_robot_pose_timer_;
  geometry_msgs::msg::TransformStamped trf_parent_robot_;

  struct
  {
    std::string robot_frame_id = "base_link";
    std::string parent_frame_id = "odom";
  } config_;
};

}  // namespace mbf_simple_nav

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  mbf_simple_nav::RobotSimulator::SharedPtr sim_node = std::make_shared<mbf_simple_nav::RobotSimulator>();
  rclcpp::spin(sim_node);
  return EXIT_SUCCESS;
}