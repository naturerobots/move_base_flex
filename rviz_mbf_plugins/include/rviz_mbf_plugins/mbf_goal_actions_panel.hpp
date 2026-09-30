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
#include <mbf_msgs/action/refine_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/ros_action_property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>

#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QPushButton>

#include <memory>
#include <optional>
#include <future>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

namespace rviz_mbf_plugins
{
using GetPathClient = rclcpp_action::Client<mbf_msgs::action::GetPath>;
using ExePathClient = rclcpp_action::Client<mbf_msgs::action::ExePath>;
using RefinePathClient = rclcpp_action::Client<mbf_msgs::action::RefinePath>;

class MbfGoalActionsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MbfGoalActionsPanel(QWidget * parent = nullptr);
  ~MbfGoalActionsPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

  void newGoalCallback(const geometry_msgs::msg::PoseStamped & msg);

protected:
  //! Sets up the properties widget, which contains editable fields that configures the panel (e.g. which topic to subscribe to)
  void constructPropertiesWidget();
  //! Sets up the goal input widget, which displays information about whether goal poses are being received
  void constructGoalInputWidget();
  //! Sets up the get path widget, which displays information about the currently active or previous get path action goal
  void constructGetPathWidget();
  //! Sets up the refine path widget, which displays information about the currently active or previous refine path action goal
  void constructRefinePathWidget();
  //! Sets up the exe path widget, which displays information about the currently active or previous exe path action
  void constructExePathWidget();

  void sendGetPathGoal(const mbf_msgs::action::GetPath::Goal & goal);
  void getPathResultCallback(const GetPathClient::GoalHandle::WrappedResult & wrapped_result);

  //! Starts the chain of selected plan refiners on the given path.
  //! If no refiner is selected (or the refiner action is unavailable), the path is executed as is.
  void startRefineChain(const nav_msgs::msg::Path & path);
  //! Sends the path currently held in refine_current_path_ to the refiner at refine_queue_idx_.
  //! Caller must hold refine_path_action_client_mutex_.
  void sendRefinePathGoalLocked();
  void refinePathResultCallback(
    const RefinePathClient::GoalHandle::WrappedResult & wrapped_result, uint64_t chain_id);
  //! Ends the running refine chain: cancels its goal and invalidates its pending callbacks.
  //! Caller must hold refine_path_action_client_mutex_.
  void stopRefineChainLocked(RefinePathClient::CancelCallback cancel_callback = nullptr);

  //! Sends the given path to the controller, cancelling a possibly still active exe path goal first
  void dispatchExePath(const nav_msgs::msg::Path & path);

  void sendExePathGoal(const mbf_msgs::action::ExePath::Goal & goal);
  void exePathResultCallback(const ExePathClient::GoalHandle::WrappedResult & wrapped_result);

  //! Rebuilds the refiner checkbox list from the names loaded on the server. Must run on the GUI thread.
  void updateRefinerProperties(const std::vector<std::string> & refiner_names);
  //! Caches the checked refiners into selected_refiners_. Must run on the GUI thread.
  void cacheSelectedRefiners();

Q_SIGNALS:
  void getPathServerStatusChanged(const QString & text, const QString & style);
  void getPathGoalStatusChanged(const QString & text, const QString & style);
  void exePathServerStatusChanged(const QString & text, const QString & style);
  void exePathGoalStatusChanged(const QString & text, const QString & style);
  void refinePathServerStatusChanged(const QString & text, const QString & style);
  void refinePathGoalStatusChanged(const QString & text, const QString & style);
  void goalInputStatusChanged(const QString & text);

private Q_SLOTS:
  void updateGoalInputSubscription();
  void updateGetPathActionClient();
  void updateRefinePathActionClient();
  void updateExePathActionClient();
  void stopGetPathAction();
  void stopRefinePathAction();
  void stopExePathAction();

protected:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

  rclcpp::Node::SharedPtr ros_node_;

  //! Action client for getting a path
  mutable std::mutex get_path_action_client_mutex_;
  GetPathClient::SharedPtr action_client_get_path_;
  std::shared_ptr<rclcpp::AsyncParametersClient>  planner_parameter_client_;
  std::string get_path_node_name_;
  std::string get_path_action_server_name_;
  std::string get_path_planner_name_;

  //! Goal handle of active get path action
  GetPathClient::GoalHandle::SharedPtr goal_handle_get_path_;

  //! Action client for refining a path
  mutable std::mutex refine_path_action_client_mutex_;
  RefinePathClient::SharedPtr action_client_refine_path_;
  std::shared_ptr<rclcpp::AsyncParametersClient> refiner_parameter_client_;
  std::string refine_path_node_name_;
  std::string refine_path_action_server_name_;
  //! Names of the checked refiners, in the order the server declares them. Cached from GUI thread.
  std::vector<std::string> selected_refiners_;
  //! Refiner names restored by load(), applied once the server's refiner list is known
  std::vector<std::string> pending_selected_refiners_;
  //! True until the refiner list has been received for the first time
  bool refiner_list_pending_;

  // All refine chain state below is protected by refine_path_action_client_mutex_.

  //! Goal handle of active refine path action
  RefinePathClient::GoalHandle::SharedPtr goal_handle_refine_path_;

  //! Refiners still to be applied to the current path, and the index of the running one
  std::vector<std::string> refine_queue_;
  size_t refine_queue_idx_;
  //! Path handed from one refiner to the next
  nav_msgs::msg::Path refine_current_path_;
  //! Bumped whenever a chain is stopped or replaced. Callbacks of older chains are dropped.
  uint64_t refine_chain_id_;

  //! Action client for traversing a path
  mutable std::mutex exe_path_action_client_mutex_;
  ExePathClient::SharedPtr action_client_exe_path_;
  std::shared_ptr<rclcpp::AsyncParametersClient>  controller_parameter_client_;
  std::string exe_path_node_name_;
  std::string exe_path_action_server_name_;
  std::string exe_path_controller_name_;

  //! Goal handle of active exe path action
  ExePathClient::GoalHandle::SharedPtr goal_handle_exe_path_;

  std::atomic_bool conn_check_thread_get_path_stop_;
  std::thread conn_check_thread_get_path_;
  std::atomic_bool conn_check_thread_refine_path_stop_;
  std::thread conn_check_thread_refine_path_;
  std::atomic_bool conn_check_thread_exe_path_stop_;
  std::thread conn_check_thread_exe_path_;
  std::atomic_bool executor_thread_stop_;
  std::thread executor_thread_;

  //! Retry counter for goal execution
  size_t goal_retry_cnt_;
  //! Current goal pose
  geometry_msgs::msg::PoseStamped current_goal_;

  ////////////////
  // UI elements
  ////////////////
  QVBoxLayout * ui_layout_;

  rviz_common::properties::PropertyTreeWidget*    properity_tree_widget_;
  rviz_common::properties::PropertyTreeModel*     properity_tree_model_;
  rviz_common::properties::RosTopicProperty*      goal_input_topic_;
  rviz_common::properties::RosActionProperty*     get_path_action_server_path_;
  rviz_common::properties::EditableEnumProperty*  planner_name_property_;
  rviz_common::properties::RosActionProperty*     refine_path_action_server_path_;
  //! Parent node of the refiner checkboxes. Collapse it to hide a long refiner list.
  rviz_common::properties::Property*              refiners_property_;
  std::vector<rviz_common::properties::BoolProperty*> refiner_properties_;
  rviz_common::properties::RosActionProperty*     exe_path_action_server_path_;
  rviz_common::properties::EditableEnumProperty*  controller_name_property_;

  QGroupBox * goal_input_ui_box_;
  QVBoxLayout * goal_input_ui_layout_;
  QLabel * goal_input_status_;

  void setGetPathServerStatusMessage(const QString & text, const QString& style);
  void setGetPathGoalStatusMessage(const QString & text, const QString& style);

  QGroupBox * get_path_ui_box_;
  QVBoxLayout * get_path_ui_layout_;
  QHBoxLayout * get_path_ui_layout_server_status_;
  QLabel * get_path_action_server_status_desc_;
  QLabel * get_path_action_server_status_;
  QHBoxLayout * get_path_ui_layout_goal_status_;
  QLabel * get_path_action_goal_status_desc_;
  QLabel * get_path_action_goal_status_;
  QPushButton * stop_get_path_button_;

  void setRefinePathServerStatusMessage(const QString & text, const QString& style);
  void setRefinePathGoalStatusMessage(const QString & text, const QString& style);

  QGroupBox * refine_path_ui_box_;
  QVBoxLayout * refine_path_ui_layout_;
  QHBoxLayout * refine_path_ui_layout_server_status_;
  QLabel * refine_path_action_server_status_desc_;
  QLabel * refine_path_action_server_status_;
  QHBoxLayout * refine_path_ui_layout_goal_status_;
  QLabel * refine_path_action_goal_status_desc_;
  QLabel * refine_path_action_goal_status_;
  QPushButton * stop_refine_path_button_;

  void setExePathServerStatusMessage(const QString & text, const QString& style);
  void setExePathGoalStatusMessage(const QString & text, const QString& style);

  QGroupBox * exe_path_ui_box_;
  QVBoxLayout * exe_path_ui_layout_;
  QHBoxLayout * exe_path_ui_layout_server_status_;
  QLabel * exe_path_action_server_status_desc_;
  QLabel * exe_path_action_server_status_;
  QHBoxLayout * exe_path_ui_layout_goal_status_;
  QLabel * exe_path_action_goal_status_desc_;
  QLabel * exe_path_action_goal_status_;
  QPushButton * stop_exe_path_button_;
};

} // namespace rviz_mbf_plugins
