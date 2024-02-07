#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mbf_simple_nav
{

/**
 * Tiny robot simulator: Takes cmd_vel msgs and updates a TF accordingly.
 * Useful for simulating a robot's movement in unit tests.
 * The frames in the config_ member define the TF's frame_id and child_frame_id.
 */
  class RobotSimulator: public rclcpp::Node
  {
public:
    //! Initialises the simulator and starts with publishing an identity TF
    RobotSimulator();

protected:
    //! Handle new command velocities. Incoming msgs need to be in the robot's frame.
    void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel);
    //! Regularly (via timer) updates the robot's pose based on current_velocity and publishes it via tf2.
    void continuouslyUpdateRobotPose();

    std::unique_ptr < tf2_ros::TransformBroadcaster > tf_broadcaster_;
    rclcpp::Subscription < geometry_msgs::msg::TwistStamped > ::SharedPtr cmd_vel_subscription_;

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
