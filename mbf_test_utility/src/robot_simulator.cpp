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

#include "mbf_test_utility/robot_simulator.hpp"

#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace mbf_simple_nav
{

RobotSimulator::RobotSimulator()
: Node("robot_simulator"), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  trf_parent_robot_.header.stamp = now();
  trf_parent_robot_.header.frame_id = config_.parent_frame_id;
  trf_parent_robot_.child_frame_id = config_.robot_frame_id;
  continuouslyUpdateRobotPose();
  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10, std::bind(&RobotSimulator::velocityCallback, this, std::placeholders::_1));
}

void RobotSimulator::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel)
{
  if (vel->header.frame_id != config_.robot_frame_id) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Dropping velocity msg. Node expects velocities in robot frame ('"
        << config_.robot_frame_id << "'), but got frame '" << vel->header.frame_id
        << "'");
  }
  /* Update the robot pose before updating to the new velocity.
   * This ensures that the robot will move according to the old velocity for time interval [t_last_update, t_now].
   * Otherwise, the robot would move based on the new velocity for
   * [t_last_update, t_next_update] (with t_now being somewhere in that interval).
   */
  continuouslyUpdateRobotPose();
  current_velocity_ = vel->twist;
}

void RobotSimulator::continuouslyUpdateRobotPose()
{
  const auto t_now = now();

  // Calculate how much the has robot moved, based on current_velocity_ (assuming constant velocity)
  const double seconds_since_last_update = (t_now - trf_parent_robot_.header.stamp).seconds();
  tf2::Quaternion orientationTrf_robotTLastUpdate_robotTNow;
  orientationTrf_robotTLastUpdate_robotTNow.setRPY(
    current_velocity_.angular.x * seconds_since_last_update,
    current_velocity_.angular.y * seconds_since_last_update,
    current_velocity_.angular.z * seconds_since_last_update);
  const tf2::Transform trf_robotTLastUpdate_robotTNow(
    orientationTrf_robotTLastUpdate_robotTNow, tf2::Vector3(
      current_velocity_.linear.x * seconds_since_last_update,
      current_velocity_.linear.y * seconds_since_last_update,
      current_velocity_.linear.z * seconds_since_last_update));

  // Updated robot pose member and publish tf
  tf2::Transform trf_parent_robotTLastUpdate;
  tf2::fromMsg(trf_parent_robot_.transform, trf_parent_robotTLastUpdate);
  const tf2::Transform trf_parent_robotTNow = trf_parent_robotTLastUpdate *
    trf_robotTLastUpdate_robotTNow;
  tf2::toMsg(trf_parent_robotTNow, trf_parent_robot_.transform);
  trf_parent_robot_.header.stamp = t_now;
  tf_broadcaster_->sendTransform(trf_parent_robot_);

  // restart timer for continuous updates
  update_robot_pose_timer_ =
    create_wall_timer(10ms, std::bind(&RobotSimulator::continuouslyUpdateRobotPose, this));
}

}  // namespace mbf_simple_nav
