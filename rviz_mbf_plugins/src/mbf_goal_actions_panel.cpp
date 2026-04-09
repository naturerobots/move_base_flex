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

#include <mbf_msgs/action/exe_path.hpp>

#include <atomic>

namespace
{
constexpr auto default_goal_input_topic = "pick topic";
constexpr auto default_action_server_path = "input action server";
} // anonymous namespace

namespace rviz_mbf_plugins
{

MbfGoalActionsPanel::MbfGoalActionsPanel(QWidget * parent)
: Panel(parent)
  , goal_retry_cnt_(0)
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
  
  get_path_action_server_path_ = new rviz_common::properties::RosActionProperty(
    "Planner Action", default_action_server_path, "mbf_msgs/action/GetPath",
    "ROS path to move base flex get_path action server",
    nullptr,
    SLOT(updateGetPathActionClient()), this);
  properity_tree_model_->getRoot()->addChild(get_path_action_server_path_);

  planner_name_property_ = new rviz_common::properties::EditableEnumProperty(
    "Planner Name", "", "Key name of planner to use (must defined in config), e.g. 'GridBased' or 'mesh_planner'");
  properity_tree_model_->getRoot()->addChild(planner_name_property_);

  exe_path_action_server_path_ = new rviz_common::properties::RosActionProperty(
    "Controller Action", default_action_server_path, "mbf_msgs/action/ExePath",
    "ROS path to move base flex exe_path action server",
    nullptr,
    SLOT(updateExePathActionClient()), this);
  properity_tree_model_->getRoot()->addChild(exe_path_action_server_path_);

  controller_name_property_ = new rviz_common::properties::EditableEnumProperty(
    "Controller Name", "", "Key name of planner to use (must defined in config), e.g. 'mppi' or 'mesh_controller'");
  properity_tree_model_->getRoot()->addChild(controller_name_property_);

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
  const std::string selected_topic = goal_input_topic_->getTopic().toStdString();
  if (selected_topic != default_goal_input_topic) {
    goal_pose_subscription_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      selected_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&MbfGoalActionsPanel::newGoalCallback, this, std::placeholders::_1));
  }
}

std::string node_name_guess(const std::string& topic)
{
  // this just guesses the node name and returns it.
  // split topic by first slashes from right. return first part as node name

  std::string delimiter = "/";

  size_t pos = topic.rfind(delimiter);

  if (pos != std::string::npos) {
    return topic.substr(0, pos);
  } else {
    return "";
  }
}

template<typename ActionClientT>
bool sync_cancel_goal(
  const typename ActionClientT::SharedPtr& action_client, 
  const typename ActionClientT::GoalHandle::SharedPtr& goal_handle,
  std::chrono::duration<double> timeout = std::chrono::seconds(5))
{
  using CancelResponse = typename ActionClientT::CancelResponse;

  std::atomic_bool cancel_process_finished = false;
  std::atomic_bool cancel_successful = false;

  action_client->async_cancel_goal(goal_handle, [&cancel_process_finished, &cancel_successful](
      const typename CancelResponse::SharedPtr & response) 
  {
    cancel_successful = (response->return_code == CancelResponse::ERROR_NONE);
    cancel_process_finished = true;
  });

  // wait for cancel process to finish (with timeout)
  auto start_time = std::chrono::steady_clock::now();
  while (!cancel_process_finished) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      break;
    }
  }

  return cancel_successful;
}

void MbfGoalActionsPanel::updateGetPathActionClient()
{
  const std::string selected_server = get_path_action_server_path_->getAction().toStdString();
  
  if(selected_server == default_action_server_path)
  {
    // reset action client: first cancel active goal, then reset client
    if (action_client_get_path_ && goal_handle_get_path_) {

      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active get_path goal due to action server change...");
      bool success = sync_cancel_goal<GetPathClient>(action_client_get_path_, goal_handle_get_path_);
      if(!success)
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active get_path goal during action server change. It might still be executing in the background.");
      } else {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active get_path goal during action server change.");
      }
    }

    // reset action client and planner parameter client
    action_client_get_path_.reset();
    goal_handle_get_path_ = nullptr;

    planner_parameter_client_.reset();
    get_path_action_server_status_->setText("waiting for input");

    return;
  }
  
  // reset server if path has changed
  if(selected_server != get_path_action_server_name_)
  {
    // reset action client: first cancel active goal, then reset client
    if (action_client_get_path_ && goal_handle_get_path_) {

      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active get_path goal due to action server change...");
      bool success = sync_cancel_goal<GetPathClient>(action_client_get_path_, goal_handle_get_path_);
      if(!success)
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active get_path goal during action server change. It might still be executing in the background.");
      } else {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active get_path goal during action server change.");
      }
    }
    action_client_get_path_.reset();
    goal_handle_get_path_ = nullptr;
  }

  // initialize action client if not initialized or resetted due to server change
  if(!action_client_get_path_)
  {
    action_client_get_path_ = rclcpp_action::create_client<mbf_msgs::action::GetPath>(
      ros_node_, selected_server);
    planner_parameter_client_.reset(); // force the following steps to reinitialize the parameter client
    planner_name_property_->clearOptions();
  }

  // connecting
  get_path_action_server_status_->setText("connecting...");
  action_client_get_path_->wait_for_action_server(
    std::chrono::seconds(1));

  if(!action_client_get_path_->action_server_is_ready())
  {
    get_path_action_server_status_->setText("connection failed!");
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Could not connect to get_path action server at " << selected_server);
    return;
  }
  
  get_path_action_server_name_ = selected_server;
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Connected to get_path action server at " << selected_server);
  get_path_action_server_status_->setText("ready");
  get_path_action_server_status_->setStyleSheet("background-color: #90EE90;");
  // { // make green
  //   QPalette palette = get_path_action_server_status_->palette();
  //   palette.setColor(QPalette::Window, Qt::green);
  //   get_path_action_server_status_->setPalette(palette);
  //   get_path_action_server_status_->setAutoFillBackground(true);
  // }

  // bool parameter_client_reinitialized = false;
  if(!planner_parameter_client_)
  {
    // TODO(amock): this is a bit hacky. We extract the node name from the action server topic
    // - but if the topic was remapped we get a problem. Currently I dont know an efficient (!) solution to that problem
    get_path_node_name_ = node_name_guess(selected_server);

    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Guessed node name for planner list: " << get_path_node_name_);

    get_path_action_server_status_->setText("connecting to parameters...");
    planner_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ros_node_, get_path_node_name_);
  }
  planner_parameter_client_->wait_for_service(std::chrono::seconds(1));

  if(!planner_parameter_client_->service_is_ready())
  {
    get_path_action_server_status_->setText("Could not connect to parameters!");
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Could not connect to parameter service of planner action server node " << get_path_node_name_);
    return;
  }

  // update list of planners
  get_path_action_server_status_->setText("updating planner list...");
  planner_parameter_client_->get_parameters({"planners"},
    [this, selected_server](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      
      auto parameters = future.get();
      if(!parameters.empty())
      {
        const std::vector<std::string> planners = parameters[0].as_string_array();
        for(const std::string& planner : planners)
        {
          planner_name_property_->addOptionStd(planner);
        }
        RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Received planner list with " << planners.size() << " entries for planner action " << selected_server);
        this->get_path_action_server_status_->setText("ready");
      } else {
        this->get_path_action_server_status_->setText("No planner found!");
        RCLCPP_WARN_STREAM(this->ros_node_->get_logger(), "No planner found for planner action " << selected_server);
      }
    });
}

void MbfGoalActionsPanel::updateExePathActionClient()
{
  const std::string selected_server = exe_path_action_server_path_->getAction().toStdString();

  if (selected_server == default_action_server_path) {

    // reset action client: first cancel active goal, then reset client
    if (action_client_exe_path_ && goal_handle_exe_path_) {

      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active exe_path goal due to action server change...");
      bool success = sync_cancel_goal<ExePathClient>(action_client_exe_path_, goal_handle_exe_path_);
      if(!success)
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active exe_path goal during action server change. It might still be executing in the background.");
      } else {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active exe_path goal during action server change.");
      }
    }
    action_client_exe_path_.reset();
    goal_handle_exe_path_ = nullptr;

    controller_parameter_client_.reset();
    exe_path_action_server_status_->setText("waiting for input");
  }

  // reset server if path has changed
  if(selected_server != exe_path_action_server_name_)
  {
    // reset action client: first cancel active goal, then reset client
    if (action_client_exe_path_ && goal_handle_exe_path_) {

      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active exe_path goal due to action server change...");
      bool success = sync_cancel_goal<ExePathClient>(action_client_exe_path_, goal_handle_exe_path_);
      if(!success)
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active exe_path goal during action server change. It might still be executing in the background.");
      } else {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active exe_path goal during action server change.");
      }
    }
    action_client_exe_path_.reset();
    goal_handle_exe_path_ = nullptr;
  }

  // initialize action client if not initialized or resetted due to server change
  if(!action_client_exe_path_)
  {
    action_client_exe_path_ = rclcpp_action::create_client<mbf_msgs::action::ExePath>(
      ros_node_, selected_server);
    controller_parameter_client_.reset(); // force the following steps to reinitialize the parameter client
    controller_name_property_->clearOptions();
  }

  // connecting
  exe_path_action_server_status_->setText("connecting...");
  action_client_exe_path_->wait_for_action_server(
    std::chrono::seconds(1));
  
  if(!action_client_exe_path_->action_server_is_ready())
  {
    exe_path_action_server_status_->setText("connection failed!");
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Could not connect to exe_path action server at " << selected_server);
    return;
  }
  
  exe_path_action_server_name_ = selected_server;
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Connected to exe_path action server at " << selected_server);
  exe_path_action_server_status_->setText("ready");

  // bool parameter_client_reinitialized = false;
  if(!controller_parameter_client_)
  {
    // TODO(amock): this is a bit hacky. We extract the node name from the action server topic
    // - but if the topic was remapped we get a problem. Currently I dont know an efficient (!) solution to that problem
    exe_path_node_name_ = node_name_guess(selected_server);

    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Guessed node name for controller list: " << exe_path_node_name_);

    exe_path_action_server_status_->setText("connecting to parameters...");
    controller_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ros_node_, exe_path_node_name_);
  }
  controller_parameter_client_->wait_for_service(std::chrono::seconds(1));

  if(!controller_parameter_client_->service_is_ready())
  {
    exe_path_action_server_status_->setText("Could not connect to parameters!");
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Could not connect to parameter service of exe_path action server node " << exe_path_node_name_);
    return;
  }

  // update list of controllers
  exe_path_action_server_status_->setText("updating controller list...");
  controller_parameter_client_->get_parameters({"controllers"},
    [this, selected_server](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      
      auto parameters = future.get();
      if(!parameters.empty())
      {
        const std::vector<std::string> controllers = parameters[0].as_string_array();
        for(const std::string& controller : controllers)
        {
          controller_name_property_->addOptionStd(controller);
        }
        RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Received controller list with " << controllers.size() << " entries for exe_path action " << selected_server);
        this->exe_path_action_server_status_->setText("ready");
      } else {
        this->exe_path_action_server_status_->setText("No controller found!");
        RCLCPP_WARN_STREAM(this->ros_node_->get_logger(), "No controller found for exe_path action " << selected_server);
      }
    });
}

void MbfGoalActionsPanel::onInitialize()
{
  const auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction();
  goal_input_topic_->initialize(ros_node_abstraction);
  get_path_action_server_path_->initialize(ros_node_abstraction);
  exe_path_action_server_path_->initialize(ros_node_abstraction);
  ros_node_ = ros_node_abstraction.lock()->get_raw_node();
}

void MbfGoalActionsPanel::newGoalCallback(const geometry_msgs::msg::PoseStamped & msg)
{
  goal_input_status_->setText(QString("Goal received at t=%1").arg(msg.header.stamp.sec));

  goal_retry_cnt_ = 0;
  current_goal_ = msg;
  mbf_msgs::action::GetPath::Goal goal;
  goal.target_pose = msg;
  goal.planner = planner_name_property_->getStdString();
  goal.use_start_pose = false; // planner shall use the current robot pose

  if(!action_client_get_path_->action_server_is_ready())
  {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Received new goal, but get_path action server is not ready. Restarting ...");
    updateGetPathActionClient();

    if(!action_client_get_path_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Get_path action server is still not ready after update. Cannot send goal.");
    }

    return;
  }
  
  if (goal_handle_get_path_) {
    // planner is currently active, cancel first
    // result callback will start new planner with next_get_path_goal_ as goal
    // UI currently cannot properly handle parallel actions
    next_get_path_goal_ = goal;
    get_path_action_goal_status_->setText("Cancelling...");
    bool success = sync_cancel_goal<GetPathClient>(action_client_get_path_, goal_handle_get_path_);
    if(!success)
    {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active get_path goal. It might still be executing in the background.");
      get_path_action_goal_status_->setText("Failed to cancel active goal");
    } else {
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active get_path goal.");
    }

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
      exe_path_goal.controller = controller_name_property_->getStdString();

      if(!action_client_exe_path_->action_server_is_ready())
      {
        RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Received new goal, but exe_path action server is not ready. Restarting ...");
        updateExePathActionClient();

        if(!action_client_exe_path_->action_server_is_ready())
        {
          RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Exe_path action server is still not ready after update. Cannot send goal.");
        }

        return;
      }

      if (goal_handle_exe_path_) {
        // path execution is currently active, cancel first
        // result callback will start new path execution with next_exe_path_goal_ as goal
        // UI currently cannot properly handle parallel actions
        next_exe_path_goal_ = exe_path_goal;
        exe_path_action_goal_status_->setText("Requested cancel");
        bool success = sync_cancel_goal<ExePathClient>(action_client_exe_path_, goal_handle_exe_path_);
        if(!success)
        {
          RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to cancel active exe_path goal.");
          // directly send new goal, even though the previous one is not properly cancelled. UI might be a bit inconsistent, but at least we try to execute the new goal
          exe_path_action_goal_status_->setText("Failed to cancel active goal.");
        } else {
          RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Successfully cancelled active exe_path goal.");
        }

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
      if (wrapped_result.result->outcome == mbf_msgs::action::ExePath::Result::ROBOT_STUCK) {
        if (goal_retry_cnt_ < 3) {
          goal_retry_cnt_ += 1;
          mbf_msgs::action::GetPath::Goal goal;
          goal.target_pose = current_goal_;
          goal.planner = planner_name_property_->getStdString();
          // Overwrite the timestamp to prevent tf extrapolation errors in the planners
          goal.target_pose.header.set__stamp(getDisplayContext()->getClock()->now());
          goal.use_start_pose = false;
          sendGetPathGoal(goal);
          exe_path_action_goal_status_->setText(
            QString("Robot stuck. Replanning ...")
          );
        } else {
          exe_path_action_goal_status_->setText(
            QString("Robot stuck. Maximum retries exceeded! Goal failed!")
          );
        }
        break;
      }
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
#include <rviz_common/panel.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mbf_plugins::MbfGoalActionsPanel, rviz_common::Panel)
