/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Nature Robots GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>

#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>

namespace rviz_mbf_plugins
{
using GetPathClient = rclcpp_action::Client<mbf_msgs::action::GetPath>;
using ExePathClient = rclcpp_action::Client<mbf_msgs::action::ExePath>;

class MbfGoalActionsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MbfGoalActionsPanel(QWidget * parent = nullptr);
  ~MbfGoalActionsPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

  void newMeshGoalCallback(const geometry_msgs::msg::PoseStamped & msg);

protected:
  //! Sets up the properties widget, which contains editable fields that configures the panel (e.g. which topic to subscribe to)
  void constructPropertiesWidget();
  //! Sets up the goal input widget, which displays information about whether goal poses are being received
  void constructGoalInputWidget();
  //! Sets up the get path widget, which displays information about the currently active or previous get path action goal
  void constructGetPathWidget();
  //! Sets up the exe path widget, which displays information about the currently active or previous exe path action
  void constructExePathWidget();

  void sendGetPathGoal(const mbf_msgs::action::GetPath::Goal & goal);
  void getPathResultCallback(const GetPathClient::GoalHandle::WrappedResult & wrapped_result);

  void sendExePathGoal(const mbf_msgs::action::ExePath::Goal & goal);
  void exePathResultCallback(const ExePathClient::GoalHandle::WrappedResult & wrapped_result);

private Q_SLOTS:
  void updateGoalInputSubscription();
  void updateGetPathServiceClient();
  void updateExePathServiceClient();

protected:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

  //! Action client for getting a path
  GetPathClient::SharedPtr action_client_get_path_;
  //! Goal handle of active get path action
  GetPathClient::GoalHandle::SharedPtr goal_handle_get_path_;
  //! Potential next get path goal, used for quickly restarting the planner after cancelling the previous goal
  std::optional<mbf_msgs::action::GetPath::Goal> next_get_path_goal_;

  //! Action client for traversing a path
  ExePathClient::SharedPtr action_client_exe_path_;
  //! Goal handle of active exe path action
  ExePathClient::GoalHandle::SharedPtr goal_handle_exe_path_;
  //! Potential next exe path goal, used for quickly restarting the path execution after cancelling the previous goal
  std::optional<mbf_msgs::action::ExePath::Goal> next_exe_path_goal_;

  ////////////////
  // UI elements
  ////////////////
  QVBoxLayout * ui_layout_;

  rviz_common::properties::PropertyTreeWidget * properity_tree_widget_;
  rviz_common::properties::PropertyTreeModel * properity_tree_model_;
  rviz_common::properties::RosTopicProperty * goal_input_topic_;
  rviz_common::properties::RosTopicProperty * get_path_action_server_path_;
  rviz_common::properties::RosTopicProperty * exe_path_action_server_path_;

  QGroupBox * goal_input_ui_box_;
  QVBoxLayout * goal_input_ui_layout_;
  QLabel * goal_input_status_;

  QGroupBox * get_path_ui_box_;
  QVBoxLayout * get_path_ui_layout_;
  QHBoxLayout * get_path_ui_layout_server_status_;
  QLabel * get_path_action_server_status_desc_;
  QLabel * get_path_action_server_status_;
  QHBoxLayout * get_path_ui_layout_goal_status_;
  QLabel * get_path_action_goal_status_desc_;
  QLabel * get_path_action_goal_status_;

  QGroupBox * exe_path_ui_box_;
  QVBoxLayout * exe_path_ui_layout_;
  QHBoxLayout * exe_path_ui_layout_server_status_;
  QLabel * exe_path_action_server_status_desc_;
  QLabel * exe_path_action_server_status_;
  QHBoxLayout * exe_path_ui_layout_goal_status_;
  QLabel * exe_path_action_goal_status_desc_;
  QLabel * exe_path_action_goal_status_;
};

} // namespace rviz_mbf_plugins
