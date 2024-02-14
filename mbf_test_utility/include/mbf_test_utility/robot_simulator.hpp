// Copyright 2024 Nature Robots GmbH
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Nature Robots GmbH nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mbf_test_utility
{

/**
 * Tiny robot simulator: Takes cmd_vel msgs and updates a TF accordingly.
 * Useful for simulating a robot's movement in unit tests.
 * The frames in the config_ member define the TF's frame_id and child_frame_id.
 */
class RobotSimulator : public rclcpp::Node
{
public:
  //! Initialises the simulator and starts with publishing an identity TF
  RobotSimulator(
    const std::string & node_name = "robot_simulator",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  //! Handle new command velocities. Incoming msgs need to be in the robot's frame.
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel);
  //! Regularly (via timer) updates the robot's pose based on current_velocity and publishes it via tf2.
  void continuouslyUpdateRobotPose();

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

}  // namespace mbf_test_utility
