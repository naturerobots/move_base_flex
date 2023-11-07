/*
 *  Copyright 2018, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  robot_information.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include "mbf_utility/robot_information.h"
#include "mbf_utility/navigation_utility.h"

#include <rclcpp/rclcpp.hpp>

namespace mbf_utility
{
RobotInformation::RobotInformation(const rclcpp::Node::SharedPtr& node,
                                   const TF &tf_listener,
                                   const std::string &global_frame,
                                   const std::string &robot_frame,
                                   const rclcpp::Duration &tf_timeout,
                                   const std::string &odom_topic)
 : node_(node), tf_listener_(tf_listener), global_frame_(global_frame), robot_frame_(robot_frame), tf_timeout_(tf_timeout),
   odom_helper_(node_, odom_topic)
{

}


bool RobotInformation::getRobotPose(geometry_msgs::msg::PoseStamped &robot_pose) const
{
  bool tf_success = mbf_utility::getRobotPose(node_, tf_listener_, robot_frame_, global_frame_,
                                              rclcpp::Duration(tf_timeout_), robot_pose);
  if (!tf_success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Can not get the robot pose in the global frame. - robot frame: \""
                         << robot_frame_ << "\"   global frame: \"" << global_frame_);
    return false;
  }
  return true;
}

bool RobotInformation::getRobotVelocity(geometry_msgs::msg::TwistStamped &robot_velocity) const
{
  if (odom_helper_.getOdomTopic().empty())
  {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000, "Odometry topic set as empty; ignoring retrieve velocity requests");
    return true;
  }

  nav_msgs::msg::Odometry base_odom;
  odom_helper_.getOdom(base_odom);
  if (base_odom.header.stamp == rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()))
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
                                "No messages received on topic " << odom_helper_.getOdomTopic() << "; robot velocity unknown");
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
                                "You can disable these warnings by setting parameter 'odom_topic' as empty");
    return false;
  }
  robot_velocity.header = base_odom.header;
  robot_velocity.twist = base_odom.twist.twist;
  return true;
}

bool RobotInformation::isRobotStopped(double rot_stopped_velocity, double trans_stopped_velocity) const
{
  nav_msgs::msg::Odometry base_odom;
  odom_helper_.getOdom(base_odom);
  return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity &&
         fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity &&
         fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity;
}

const std::string& RobotInformation::getGlobalFrame() const {return global_frame_;};

const std::string& RobotInformation::getRobotFrame() const {return robot_frame_;};

const TF& RobotInformation::getTransformListener() const {return tf_listener_;};

const rclcpp::Duration& RobotInformation::getTfTimeout() const {return tf_timeout_;}

} /* namespace mbf_utility */
