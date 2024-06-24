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


#include "rviz_mbf_plugins/mbf_goal_actions_panel.hpp"

#include <functional>

#include <rviz_common/display_context.hpp>
#include <QGroupBox>
#include <QVBoxLayout>

namespace
{
constexpr auto default_goal_input_topic = "pick topic";
constexpr auto default_action_server_path = "input action server";
} // anonymous namespace

namespace rviz_mbf_plugins
{

MbfGoalActionsPanel::MbfGoalActionsPanel(QWidget * parent)
: Panel(parent)
{
  constructPropertiesWidget();
  constructGoalInputWidget();
  constructGetPathWidget();
  constructExePathWidget();

  // outermost layout: put different widgets in vertical box
  ui_layout_ = new QVBoxLayout();
  ui_layout_->addWidget(properity_tree_widget_);
  ui_layout_->addWidget(goal_input_ui_box_);
  ui_layout_->addWidget(get_path_ui_box_);
  ui_layout_->addWidget(exe_path_ui_box_);
  setLayout(ui_layout_);
}

void MbfGoalActionsPanel::constructPropertiesWidget()
{
  properity_tree_model_ = new rviz_common::properties::PropertyTreeModel(
    new rviz_common::properties::Property());

  goal_input_topic_ = new rviz_common::properties::RosTopicProperty(
    "Goal Topic", default_goal_input_topic,
    QString::fromStdString(rosidl_generator_traits::name<geometry_msgs::msg::PoseStamped>()),
    "Goal input topic to subscribe to",
    nullptr,
    SLOT(updateGoalInputSubscription()), this);
  properity_tree_model_->getRoot()->addChild(goal_input_topic_);
  get_path_action_server_path_ = new rviz_common::properties::RosTopicProperty(
    "Get Path", default_action_server_path, "",
    "ROS path to move base flex get_path action server",
    nullptr,
    SLOT(updateGetPathServiceClient()), this);
  properity_tree_model_->getRoot()->addChild(get_path_action_server_path_);
  exe_path_action_server_path_ = new rviz_common::properties::RosTopicProperty(
    "Exe Path", default_action_server_path, "",
    "ROS path to move base flex exe_path action server",
    nullptr,
    SLOT(updateExePathServiceClient()), this);
  properity_tree_model_->getRoot()->addChild(exe_path_action_server_path_);

  properity_tree_widget_ = new rviz_common::properties::PropertyTreeWidget();
  properity_tree_widget_->setModel(properity_tree_model_);
}

void MbfGoalActionsPanel::constructGoalInputWidget()
{
  goal_input_status_ = new QLabel("No goal received");
  goal_input_ui_layout_ = new QVBoxLayout();
  goal_input_ui_layout_->addWidget(goal_input_status_);
  goal_input_ui_box_ = new QGroupBox("Goal Input");
  goal_input_ui_box_->setLayout(goal_input_ui_layout_);
}

void MbfGoalActionsPanel::constructGetPathWidget()
{
  get_path_ui_layout_server_status_ = new QHBoxLayout();
  get_path_action_server_status_desc_ = new QLabel("Server:");
  get_path_action_server_status_ = new QLabel("input path");
  get_path_ui_layout_server_status_->addWidget(get_path_action_server_status_desc_);
  get_path_ui_layout_server_status_->addWidget(get_path_action_server_status_);

  get_path_ui_layout_goal_status_ = new QHBoxLayout();
  get_path_action_goal_status_desc_ = new QLabel("Goal:");
  get_path_action_goal_status_ = new QLabel("None sent yet");
  get_path_action_goal_status_->setWordWrap(true);
  get_path_ui_layout_goal_status_->addWidget(get_path_action_goal_status_desc_);
  get_path_ui_layout_goal_status_->addWidget(get_path_action_goal_status_);

  get_path_ui_layout_ = new QVBoxLayout();
  get_path_ui_layout_->addLayout(get_path_ui_layout_server_status_);
  get_path_ui_layout_->addLayout(get_path_ui_layout_goal_status_);

  get_path_ui_box_ = new QGroupBox("Get Path");
  get_path_ui_box_->setLayout(get_path_ui_layout_);
}

void MbfGoalActionsPanel::constructExePathWidget()
{
  exe_path_ui_layout_server_status_ = new QHBoxLayout();
  exe_path_action_server_status_desc_ = new QLabel("Server:");
  exe_path_action_server_status_ = new QLabel("input path");
  exe_path_ui_layout_server_status_->addWidget(exe_path_action_server_status_desc_);
  exe_path_ui_layout_server_status_->addWidget(exe_path_action_server_status_);

  exe_path_ui_layout_goal_status_ = new QHBoxLayout();
  exe_path_action_goal_status_desc_ = new QLabel("Goal:");
  exe_path_action_goal_status_ = new QLabel("None sent yet");
  exe_path_action_goal_status_->setWordWrap(true);
  exe_path_ui_layout_goal_status_->addWidget(exe_path_action_goal_status_desc_);
  exe_path_ui_layout_goal_status_->addWidget(exe_path_action_goal_status_);

  exe_path_ui_layout_ = new QVBoxLayout();
  exe_path_ui_layout_->addLayout(exe_path_ui_layout_server_status_);
  exe_path_ui_layout_->addLayout(exe_path_ui_layout_goal_status_);

  exe_path_ui_box_ = new QGroupBox("Execute Path");
  exe_path_ui_box_->setLayout(exe_path_ui_layout_);
}

MbfGoalActionsPanel::~MbfGoalActionsPanel()
{
  delete ui_layout_;

  delete properity_tree_widget_;
  delete properity_tree_model_;
  delete goal_input_topic_;
  delete get_path_action_server_path_;
  delete exe_path_action_server_path_;

  delete goal_input_ui_box_;
  delete goal_input_ui_layout_;
  delete goal_input_status_;

  delete get_path_ui_box_;
  delete get_path_ui_layout_;
  delete get_path_ui_layout_goal_status_;
  delete get_path_action_goal_status_;
  delete get_path_action_goal_status_desc_;
  delete get_path_ui_layout_server_status_;
  delete get_path_action_server_status_;
  delete get_path_action_server_status_desc_;

  delete exe_path_ui_box_;
  delete exe_path_ui_layout_;
  delete exe_path_ui_layout_goal_status_;
  delete exe_path_action_goal_status_;
  delete exe_path_action_goal_status_desc_;
  delete exe_path_ui_layout_server_status_;
  delete exe_path_action_server_status_;
  delete exe_path_action_server_status_desc_;
}


void MbfGoalActionsPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  goal_input_topic_->save(config.mapMakeChild("goal_input_topic"));
  get_path_action_server_path_->save(config.mapMakeChild("get_path_action_server_path"));
  exe_path_action_server_path_->save(config.mapMakeChild("exe_path_action_server_path"));
}

void MbfGoalActionsPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  goal_input_topic_->load(config.mapGetChild("goal_input_topic"));
  get_path_action_server_path_->load(config.mapGetChild("get_path_action_server_path"));
  exe_path_action_server_path_->load(config.mapGetChild("exe_path_action_server_path"));
}

void MbfGoalActionsPanel::updateGoalInputSubscription()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  const std::string selected_topic = goal_input_topic_->getTopic().toStdString();
  if (selected_topic != default_goal_input_topic) {
    goal_pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      selected_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&MbfGoalActionsPanel::newMeshGoalCallback, this, std::placeholders::_1));
  }
}

void MbfGoalActionsPanel::updateGetPathServiceClient()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  const std::string selected_server = get_path_action_server_path_->getTopic().toStdString();

  if (selected_server != default_action_server_path) {
    action_client_get_path_ = rclcpp_action::create_client<mbf_msgs::action::GetPath>(
      node, selected_server);

    if (action_client_get_path_->action_server_is_ready()) {
      get_path_action_server_status_->setText("ready");
    } else {
      get_path_action_server_status_->setText("connecting");
    }
  }
}

void MbfGoalActionsPanel::updateExePathServiceClient()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  const std::string selected_server = exe_path_action_server_path_->getTopic().toStdString();
  if (selected_server != default_action_server_path) {
    action_client_exe_path_ = rclcpp_action::create_client<mbf_msgs::action::ExePath>(
      node, selected_server);
    if (action_client_exe_path_->action_server_is_ready()) {
      exe_path_action_server_status_->setText("ready");
    } else {
      exe_path_action_server_status_->setText("connecting");
    }
  }
}

void MbfGoalActionsPanel::onInitialize()
{
  const auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction();
  goal_input_topic_->initialize(ros_node_abstraction);
  get_path_action_server_path_->initialize(ros_node_abstraction);
  exe_path_action_server_path_->initialize(ros_node_abstraction);
}

void MbfGoalActionsPanel::newMeshGoalCallback(const geometry_msgs::msg::PoseStamped & msg)
{
  goal_input_status_->setText(QString("Goal received at t=%1").arg(msg.header.stamp.sec));

  mbf_msgs::action::GetPath::Goal goal;
  goal.target_pose = msg;
  goal.use_start_pose = false; // planner shall use the current robot pose
  if (goal_handle_get_path_) {
    // planner is currently active, cancel first
    // result callback will start new planner with next_get_path_goal_ as goal
    // UI currently cannot properly handle parallel actions
    next_get_path_goal_ = goal;
    get_path_action_goal_status_->setText("Cancelling...");
    action_client_get_path_->async_cancel_goal(goal_handle_get_path_);
  } else {
    sendGetPathGoal(goal);
  }
}

void MbfGoalActionsPanel::sendGetPathGoal(const mbf_msgs::action::GetPath::Goal & goal)
{
  GetPathClient::SendGoalOptions options;
  const auto goal_stamp = goal.target_pose.header.stamp;
  options.goal_response_callback =
    [goal_stamp, this](const GetPathClient::GoalHandle::SharedPtr & goal_handle) {
      this->goal_handle_get_path_ = goal_handle;
      if (goal_handle) {
        this->get_path_action_goal_status_->setText(QString("(t=%1) accepted").arg(goal_stamp.sec));
      } else {
        this->get_path_action_goal_status_->setText(QString("(t=%1) rejected").arg(goal_stamp.sec));
      }
    };
  options.result_callback = std::bind(
    &MbfGoalActionsPanel::getPathResultCallback, this,
    std::placeholders::_1);

  action_client_get_path_->async_send_goal(goal, options);
  get_path_action_goal_status_->setText(
    QString("(t=%1) sent, awaiting response").arg(
      goal_stamp.sec));
}

void MbfGoalActionsPanel::getPathResultCallback(
  const GetPathClient::GoalHandle::WrappedResult & wrapped_result)
{
  mbf_msgs::action::ExePath::Goal exe_path_goal;
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      get_path_action_goal_status_->setText(
        QString("Succeeded. Path cost %1").arg(
          wrapped_result.result->cost));

      exe_path_goal.path = wrapped_result.result->path;

      if (goal_handle_exe_path_) {
        // path execution is currently active, cancel first
        // result callback will start new path execution with next_exe_path_goal_ as goal
        // UI currently cannot properly handle parallel actions
        next_exe_path_goal_ = exe_path_goal;
        exe_path_action_goal_status_->setText("Requested cancel");
        action_client_exe_path_->async_cancel_goal(goal_handle_exe_path_);
      } else {
        sendExePathGoal(exe_path_goal);
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      get_path_action_goal_status_->setText(
        QString("Aborted. %1").arg(
          QString::fromStdString(
            wrapped_result.result->
            message)));
      break;
    case rclcpp_action::ResultCode::CANCELED:
      get_path_action_goal_status_->setText(
        QString("Cancelled. %1").arg(
          QString::fromStdString(
            wrapped_result.result->
            message)));
      if (next_get_path_goal_.has_value()) {
        sendGetPathGoal(next_get_path_goal_.value());
        next_get_path_goal_.reset();
      }
      break;
    default:
      get_path_action_goal_status_->setText("Error: Unknown action result code");
      break;
  }
  goal_handle_get_path_.reset();
}

void MbfGoalActionsPanel::sendExePathGoal(const mbf_msgs::action::ExePath::Goal & goal)
{
  const auto goal_stamp = goal.path.header.stamp;
  ExePathClient::SendGoalOptions options;
  options.goal_response_callback =
    [goal_stamp, this](const ExePathClient::GoalHandle::SharedPtr & goal_handle) {
      this->goal_handle_exe_path_ = goal_handle;
      if (goal_handle) {
        this->exe_path_action_goal_status_->setText(
          QString("Goal (t=%1) accepted").arg(
            goal_stamp.
            sec));
      } else {
        this->exe_path_action_goal_status_->setText(
          QString("Goal (t=%1) rejected").arg(
            goal_stamp.
            sec));
      }
    };
  options.feedback_callback = [this](
    ExePathClient::GoalHandle::SharedPtr,
    const mbf_msgs::action::ExePath::Feedback::ConstSharedPtr feedback)
    {
      this->exe_path_action_goal_status_->setText(
        QString("Executing. Remaining distance: %1m").arg(
          feedback->dist_to_goal));
    };
  options.result_callback = std::bind(
    &MbfGoalActionsPanel::exePathResultCallback, this,
    std::placeholders::_1);

  action_client_exe_path_->async_send_goal(goal, options);
  exe_path_action_goal_status_->setText(
    QString("(t=%1) sent, awaiting response").arg(
      goal_stamp.sec));
}

void MbfGoalActionsPanel::exePathResultCallback(
  const ExePathClient::GoalHandle::WrappedResult & wrapped_result)
{
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      exe_path_action_goal_status_->setText(
        QString("Succeeded (remaining distance: %1").arg(
          wrapped_result.result->dist_to_goal));
      break;
    case rclcpp_action::ResultCode::ABORTED:
      exe_path_action_goal_status_->setText(
        QString("Aborted. %1").arg(
          QString::fromStdString(
            wrapped_result.result->
            message)));
      break;
    case rclcpp_action::ResultCode::CANCELED:
      exe_path_action_goal_status_->setText(
        QString("Cancelled. %1").arg(
          QString::fromStdString(
            wrapped_result.result->
            message)));
      if (next_exe_path_goal_.has_value()) {
        sendExePathGoal(next_exe_path_goal_.value());
        next_get_path_goal_.reset();
      }
      break;
    default:
      exe_path_action_goal_status_->setText("Error: Unknown action result code");
      break;
  }
  goal_handle_exe_path_.reset();
}

} // namespace rviz_mbf_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mbf_plugins::MbfGoalActionsPanel, rviz_common::Panel)
