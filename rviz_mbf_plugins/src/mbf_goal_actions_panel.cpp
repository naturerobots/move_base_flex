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
#include <QMetaObject>

#include <mbf_msgs/action/exe_path.hpp>

#include <atomic>

namespace rviz_mbf_plugins
{

constexpr auto notset_goal_input_topic = "pick topic";
constexpr auto notset_action_server_path = "input action server";

// Color scheme for status messages

static const QString status_success   = "background-color: #90EE90; color: #1a1a1a;";   // Light green, dark text
static const QString status_error     = "background-color: #FFB6C6; color: #ffffff;";   // Light red, white text
static const QString status_warning   = "background-color: #FFE4B5; color: #1a1a1a;";   // Light orange, dark text
static const QString status_info      = "background-color: #ADD8E6; color: #1a1a1a;";   // Light blue, dark text
static const QString status_neutral   = "background-color: #D3D3D3; color: #1a1a1a;";   // Light gray, dark text

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

  // Wire status signals to widget updates. AutoConnection: direct on GUI thread, queued from other threads.
  connect(this, &MbfGoalActionsPanel::goalInputStatusChanged,
    goal_input_status_, &QLabel::setText);
  connect(this, &MbfGoalActionsPanel::getPathServerStatusChanged,
    this, [this](const QString& t, const QString& s) {
      get_path_action_server_status_->setText(t);
      get_path_action_server_status_->setStyleSheet(s);
    });
  connect(this, &MbfGoalActionsPanel::getPathGoalStatusChanged,
    this, [this](const QString& t, const QString& s) {
      get_path_action_goal_status_->setText(t);
      get_path_action_goal_status_->setStyleSheet(s);
    });
  connect(this, &MbfGoalActionsPanel::exePathServerStatusChanged,
    this, [this](const QString& t, const QString& s) {
      exe_path_action_server_status_->setText(t);
      exe_path_action_server_status_->setStyleSheet(s);
    });
  connect(this, &MbfGoalActionsPanel::exePathGoalStatusChanged,
    this, [this](const QString& t, const QString& s) {
      exe_path_action_goal_status_->setText(t);
      exe_path_action_goal_status_->setStyleSheet(s);
    });
}

MbfGoalActionsPanel::~MbfGoalActionsPanel() {

  // joining threads here
  conn_check_thread_get_path_stop_ = true;
  if(conn_check_thread_get_path_.joinable())
  {
    conn_check_thread_get_path_.join();
  }
  conn_check_thread_exe_path_stop_ = true;
  if(conn_check_thread_exe_path_.joinable())
  {
    conn_check_thread_exe_path_.join();
  }
  executor_thread_stop_ = true;
  if(executor_thread_.joinable())
  {
    executor_thread_.join();
  }
}

void MbfGoalActionsPanel::constructPropertiesWidget()
{
  properity_tree_model_ = new rviz_common::properties::PropertyTreeModel(
    new rviz_common::properties::Property());

  goal_input_topic_ = new rviz_common::properties::RosTopicProperty(
    "Goal Topic", notset_goal_input_topic,
    QString::fromStdString(rosidl_generator_traits::name<geometry_msgs::msg::PoseStamped>()),
    "Goal input topic to subscribe to",
    nullptr,
    SLOT(updateGoalInputSubscription()), this);
  properity_tree_model_->getRoot()->addChild(goal_input_topic_);
  
  get_path_action_server_path_ = new rviz_common::properties::RosActionProperty(
    "Planner Action", notset_action_server_path, "mbf_msgs/action/GetPath",
    "ROS path to move base flex get_path action server",
    nullptr,
    SLOT(updateGetPathActionClient()), this);
  properity_tree_model_->getRoot()->addChild(get_path_action_server_path_);

  planner_name_property_ = new rviz_common::properties::EditableEnumProperty(
    "Planner Name", "", "Key name of planner to use (must defined in config), e.g. 'GridBased' or 'mesh_planner'");
  properity_tree_model_->getRoot()->addChild(planner_name_property_);
  // Keep cached name in sync with the property (signal runs on GUI thread)
  connect(planner_name_property_, &rviz_common::properties::Property::changed, this, [this]() {
    std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
    get_path_planner_name_ = planner_name_property_->getStdString();
  });

  exe_path_action_server_path_ = new rviz_common::properties::RosActionProperty(
    "Controller Action", notset_action_server_path, "mbf_msgs/action/ExePath",
    "ROS path to move base flex exe_path action server",
    nullptr,
    SLOT(updateExePathActionClient()), this);
  properity_tree_model_->getRoot()->addChild(exe_path_action_server_path_);

  controller_name_property_ = new rviz_common::properties::EditableEnumProperty(
    "Controller Name", "", "Key name of planner to use (must defined in config), e.g. 'mppi' or 'mesh_controller'");
  properity_tree_model_->getRoot()->addChild(controller_name_property_);
  // Keep cached name in sync with the property (signal runs on GUI thread)
  connect(controller_name_property_, &rviz_common::properties::Property::changed, this, [this]() {
    std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
    exe_path_controller_name_ = controller_name_property_->getStdString();
  });

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

  stop_get_path_button_ = new QPushButton("Stop Planner Action");
  connect(stop_get_path_button_, &QPushButton::clicked, this, &MbfGoalActionsPanel::stopGetPathAction);

  get_path_ui_layout_ = new QVBoxLayout();
  get_path_ui_layout_->addLayout(get_path_ui_layout_server_status_);
  get_path_ui_layout_->addLayout(get_path_ui_layout_goal_status_);
  get_path_ui_layout_->addWidget(stop_get_path_button_);

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

  stop_exe_path_button_ = new QPushButton("Stop Controller Action");
  connect(stop_exe_path_button_, &QPushButton::clicked, this, &MbfGoalActionsPanel::stopExePathAction);

  exe_path_ui_layout_ = new QVBoxLayout();
  exe_path_ui_layout_->addLayout(exe_path_ui_layout_server_status_);
  exe_path_ui_layout_->addLayout(exe_path_ui_layout_goal_status_);
  exe_path_ui_layout_->addWidget(stop_exe_path_button_);

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
  if (selected_topic != notset_goal_input_topic) {
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
  // shared state: the callback may still fire after we gave up waiting
  auto cancel_process_finished = std::make_shared<std::atomic_bool>(false);
  auto cancel_successful = std::make_shared<std::atomic_bool>(false);
  action_client->async_cancel_goal(goal_handle, [cancel_process_finished, cancel_successful](
      const typename CancelResponse::SharedPtr & response) 
  {
    *cancel_successful = (response->return_code == CancelResponse::ERROR_NONE);
    *cancel_process_finished = true;
  });
  // wait for cancel process to finish (with timeout)
  auto start_time = std::chrono::steady_clock::now();
  while (!*cancel_process_finished) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      break;
    }
  }

  return *cancel_successful;
}

void MbfGoalActionsPanel::updateGetPathActionClient()
{
  // Runs on the GUI thread whenever the action server property changes. Only (re-)creates the clients and never
  // waits for the server: connecting and fetching the planner list is done by the connection check thread.
  std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);

  const std::string selected_server = get_path_action_server_path_->getAction().toStdString();

  if(action_client_get_path_ && selected_server == get_path_action_server_name_)
  {
    // nothing changed
    return;
  }

  // take the old client and goal out under the lock and reset the state. The cancel below must run without holding
  // the lock, because the cancel response is processed by the executor thread, whose callbacks also take the lock.
  GetPathClient::SharedPtr old_client = std::move(action_client_get_path_);
  GetPathClient::GoalHandle::SharedPtr old_goal = std::move(goal_handle_get_path_);
  planner_parameter_client_.reset();
  get_path_action_server_name_.clear();
  get_path_connected_ = false;
  planner_list_requested_ = false;
  lock.unlock();

  if(old_client && old_goal && old_client->action_server_is_ready())
  {
    // server changed or unset: cancel the active goal (skipped if the server is gone anyway)
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active get_path goal due to action server change...");
    setGetPathGoalStatusMessage("Cancelling active goal", status_warning);
    if(sync_cancel_goal<GetPathClient>(old_client, old_goal, std::chrono::seconds(1)))
    {
      setGetPathGoalStatusMessage("Planner action stopped", status_success);
    } else {
      setGetPathGoalStatusMessage("Failed to stop planner action", status_error);
    }
  } else {
    setGetPathGoalStatusMessage("none sent yet", status_neutral);
  }
  old_goal.reset();
  old_client.reset();

  lock.lock();

  if(selected_server == notset_action_server_path)
  {
    setGetPathServerStatusMessage("waiting for input", status_neutral);
    return;
  }

  try {
    action_client_get_path_ = rclcpp_action::create_client<mbf_msgs::action::GetPath>(ros_node_, selected_server);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to create action client for get_path action server at " << selected_server << ": " << e.what());
    setGetPathServerStatusMessage("connection failed!", status_error);
    return;
  }

  // TODO(amock): this is a bit hacky. We extract the node name from the action server topic
  // - but if the topic was remapped we get a problem. Currently I dont know an efficient (!) solution to that problem
  get_path_node_name_ = node_name_guess(selected_server);
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Guessed node name for planner list: " << get_path_node_name_);
  planner_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ros_node_, get_path_node_name_);

  get_path_action_server_name_ = selected_server;
  setGetPathServerStatusMessage("connecting...", status_info);
}

void MbfGoalActionsPanel::checkGetPathConnection()
{
  // Runs on the connection check thread. Only non-blocking calls in here.
  std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
  if(!action_client_get_path_ || !planner_parameter_client_)
  {
    return;
  }

  if(!action_client_get_path_->action_server_is_ready())
  {
    if(get_path_connected_)
    {
      RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Connection to get_path action server lost. Waiting for it to come back...");
      goal_handle_get_path_.reset();
      setGetPathGoalStatusMessage("none sent yet", status_neutral);
    }
    get_path_connected_ = false;
    planner_list_requested_ = false;
    setGetPathServerStatusMessage("waiting for server...", status_warning);
    return;
  }

  if(!get_path_connected_)
  {
    get_path_connected_ = true;
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Connected to get_path action server at " << get_path_action_server_name_);
  }

  if(planner_list_requested_)
  {
    // the parameter callback sets the final status
    return;
  }

  if(!planner_parameter_client_->service_is_ready())
  {
    setGetPathServerStatusMessage("connecting to parameters...", status_info);
    return;
  }

  // update list of planners
  planner_list_requested_ = true;
  setGetPathServerStatusMessage("updating planner list...", status_info);
  const std::string selected_server = get_path_action_server_name_;
  planner_parameter_client_->get_parameters({"planners"},
    [this, selected_server](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      // Parameter client callback runs on executor thread — marshal Qt property access to GUI thread.
      auto parameters = future.get();
      QMetaObject::invokeMethod(this, [this, selected_server, parameters]() {
        if(!parameters.empty())
        {
          planner_name_property_->clearOptions();
          const std::vector<std::string> planners = parameters[0].as_string_array();
          for(const std::string& planner : planners)
          {
            planner_name_property_->addOptionStd(planner);
          }
          RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Received planner list with " << planners.size() << " entries for planner action " << selected_server);
          setGetPathServerStatusMessage("ready", status_success);
        } else {
          setGetPathServerStatusMessage("No planner found!", status_error);
          RCLCPP_WARN_STREAM(this->ros_node_->get_logger(), "No planner found for planner action " << selected_server);
        }
      }, Qt::QueuedConnection);
    });
}

void MbfGoalActionsPanel::updateExePathActionClient()
{
  // Runs on the GUI thread whenever the action server property changes. Only (re-)creates the clients and never
  // waits for the server: connecting and fetching the controller list is done by the connection check thread.
  std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);

  const std::string selected_server = exe_path_action_server_path_->getAction().toStdString();

  if(action_client_exe_path_ && selected_server == exe_path_action_server_name_)
  {
    // nothing changed
    return;
  }

  // take the old client and goal out under the lock and reset the state. The cancel below must run without holding
  // the lock, because the cancel response is processed by the executor thread, whose callbacks also take the lock.
  ExePathClient::SharedPtr old_client = std::move(action_client_exe_path_);
  ExePathClient::GoalHandle::SharedPtr old_goal = std::move(goal_handle_exe_path_);
  controller_parameter_client_.reset();
  exe_path_action_server_name_.clear();
  exe_path_connected_ = false;
  controller_list_requested_ = false;
  lock.unlock();

  if(old_client && old_goal && old_client->action_server_is_ready())
  {
    // server changed or unset: cancel the active goal (skipped if the server is gone anyway)
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cancelling active exe_path goal due to action server change...");
    setExePathGoalStatusMessage("Cancelling active goal", status_warning);
    if(sync_cancel_goal<ExePathClient>(old_client, old_goal, std::chrono::seconds(1)))
    {
      setExePathGoalStatusMessage("Controller action stopped", status_success);
    } else {
      setExePathGoalStatusMessage("Failed to stop controller action", status_error);
    }
  } else {
    setExePathGoalStatusMessage("none sent yet", status_neutral);
  }
  old_goal.reset();
  old_client.reset();

  lock.lock();

  if(selected_server == notset_action_server_path)
  {
    setExePathServerStatusMessage("waiting for input", status_neutral);
    return;
  }

  try {
    action_client_exe_path_ = rclcpp_action::create_client<mbf_msgs::action::ExePath>(ros_node_, selected_server);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to create action client for exe_path action server at " << selected_server << ": " << e.what());
    setExePathServerStatusMessage("connection failed!", status_error);
    return;
  }

  // TODO(amock): this is a bit hacky. We extract the node name from the action server topic
  // - but if the topic was remapped we get a problem. Currently I dont know an efficient (!) solution to that problem
  exe_path_node_name_ = node_name_guess(selected_server);
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Guessed node name for controller list: " << exe_path_node_name_);
  controller_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ros_node_, exe_path_node_name_);

  exe_path_action_server_name_ = selected_server;
  setExePathServerStatusMessage("connecting...", status_info);
}

void MbfGoalActionsPanel::checkExePathConnection()
{
  // Runs on the connection check thread. Only non-blocking calls in here.
  std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
  if(!action_client_exe_path_ || !controller_parameter_client_)
  {
    return;
  }

  if(!action_client_exe_path_->action_server_is_ready())
  {
    if(exe_path_connected_)
    {
      RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Connection to exe_path action server lost. Waiting for it to come back...");
      goal_handle_exe_path_.reset();
      setExePathGoalStatusMessage("none sent yet", status_neutral);
    }
    exe_path_connected_ = false;
    controller_list_requested_ = false;
    setExePathServerStatusMessage("waiting for server...", status_warning);
    return;
  }

  if(!exe_path_connected_)
  {
    exe_path_connected_ = true;
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Connected to exe_path action server at " << exe_path_action_server_name_);
  }

  if(controller_list_requested_)
  {
    // the parameter callback sets the final status
    return;
  }

  if(!controller_parameter_client_->service_is_ready())
  {
    setExePathServerStatusMessage("connecting to parameters...", status_info);
    return;
  }

  // update list of controllers
  controller_list_requested_ = true;
  setExePathServerStatusMessage("updating controller list...", status_info);
  const std::string selected_server = exe_path_action_server_name_;
  controller_parameter_client_->get_parameters({"controllers"},
    [this, selected_server](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      // Parameter client callback runs on executor thread — marshal Qt property access to GUI thread.
      auto parameters = future.get();
      QMetaObject::invokeMethod(this, [this, selected_server, parameters]() {
        if(!parameters.empty())
        {
          controller_name_property_->clearOptions();
          const std::vector<std::string> controllers = parameters[0].as_string_array();
          for(const std::string& controller : controllers)
          {
            controller_name_property_->addOptionStd(controller);
          }
          RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Received controller list with " << controllers.size() << " entries for exe_path action " << selected_server);
          setExePathServerStatusMessage("ready", status_success);
        } else {
          setExePathServerStatusMessage("No controller found!", status_error);
          RCLCPP_WARN_STREAM(this->ros_node_->get_logger(), "No controller found for exe_path action " << selected_server);
        }
      }, Qt::QueuedConnection);
    });
}

void MbfGoalActionsPanel::stopGetPathAction()
{
  std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);

  if (!action_client_get_path_) 
  {
    setGetPathGoalStatusMessage("No planner action to stop", status_neutral);
    return;
  }

  if(!goal_handle_get_path_)
  {
    setGetPathGoalStatusMessage("No planner goal to stop", status_neutral);
    return;
  }

  // start stopping planner action
  setGetPathGoalStatusMessage("Stopping planner action...", status_warning);
  const auto active_goal = goal_handle_get_path_;
  action_client_get_path_->async_cancel_goal(active_goal, [this, active_goal](
      const typename GetPathClient::CancelResponse::SharedPtr & response) 
  {
    if(response->return_code == GetPathClient::CancelResponse::ERROR_NONE)
    {
      setGetPathGoalStatusMessage("Planner action stopped", status_success);
    } else {
      setGetPathGoalStatusMessage("Failed to stop planner action", status_error);
    }
    std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
    if(goal_handle_get_path_ == active_goal)
    {
      goal_handle_get_path_.reset();
    }
  });
}

void MbfGoalActionsPanel::stopExePathAction()
{
  std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);

  if (!action_client_exe_path_) 
  {
    setExePathGoalStatusMessage("No controller action to stop", status_neutral);
    return;
  }

  if(!goal_handle_exe_path_)
  {
    setExePathGoalStatusMessage("No controller goal to stop", status_neutral);
    return;
  }

  // start stopping controller action
  setExePathGoalStatusMessage("Stopping controller action...", status_warning);
  const auto active_goal = goal_handle_exe_path_;
  action_client_exe_path_->async_cancel_goal(active_goal, [this, active_goal](
      const typename ExePathClient::CancelResponse::SharedPtr & response) 
  {
    if(response->return_code == ExePathClient::CancelResponse::ERROR_NONE)
    {
      setExePathGoalStatusMessage("Controller action stopped", status_success);
    } else {
      setExePathGoalStatusMessage("Failed to stop controller action", status_error);
    }
    std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
    if(goal_handle_exe_path_ == active_goal)
    {
      goal_handle_exe_path_.reset();
    }
  });
}

void MbfGoalActionsPanel::onInitialize()
{
  const auto rviz_ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction();
  goal_input_topic_->initialize(rviz_ros_node_abstraction);
  get_path_action_server_path_->initialize(rviz_ros_node_abstraction);
  exe_path_action_server_path_->initialize(rviz_ros_node_abstraction);

  // rviz node is spinned externally. But we need full control when calling action clients
  ros_node_ = rclcpp::Node::make_shared("mbf_goal_actions_panel");

  conn_check_thread_get_path_stop_ = false;
  conn_check_thread_get_path_ = std::thread([this]() -> void {
    while (rclcpp::ok() && !conn_check_thread_get_path_stop_) {
      checkGetPathConnection();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  conn_check_thread_exe_path_stop_ = false;
  conn_check_thread_exe_path_ = std::thread([this]() -> void {
    while (rclcpp::ok() && !conn_check_thread_exe_path_stop_) {
      checkExePathConnection();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  // executor thread. needed so that actions work properly
  executor_thread_stop_ = false;
  executor_thread_ = std::thread([this]() -> void {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros_node_);
    while (rclcpp::ok() && !executor_thread_stop_) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
  });

} 

void MbfGoalActionsPanel::newGoalCallback(const geometry_msgs::msg::PoseStamped & msg)
{
  emit goalInputStatusChanged(QString("Goal received at t=%1").arg(msg.header.stamp.sec));

  goal_retry_cnt_ = 0;
  current_goal_ = msg;
  mbf_msgs::action::GetPath::Goal get_path_goal;
  get_path_goal.target_pose = msg;
  get_path_goal.use_start_pose = false; // planner shall use the current robot pose

  // copy client and active goal under the lock, use the copies outside of it
  GetPathClient::SharedPtr client;
  GetPathClient::GoalHandle::SharedPtr active_goal;
  {
    std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
    get_path_goal.planner = get_path_planner_name_; // cached from GUI thread, protected by mutex
    client = action_client_get_path_;
    active_goal = goal_handle_get_path_;
  }

  if(!client)
  {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Get path action client not initialized. Cannot send goal.");
    setGetPathGoalStatusMessage("Not sent: no planner action selected", status_error);
    return;
  }
  if(!client->action_server_is_ready())
  {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Get_path action server is not connected. Cannot send goal.");
    setGetPathGoalStatusMessage("Not sent: planner server not connected", status_error);
    return;
  }

  if (active_goal) {
    // planner is currently active, cancel first and send the new goal afterwards
    // UI currently cannot properly handle parallel actions
    setGetPathGoalStatusMessage("Cancelling...", status_warning);
    client->async_cancel_goal(active_goal, [this, get_path_goal, active_goal](
        const typename GetPathClient::CancelResponse::SharedPtr & response)
    {
      if(response->return_code == GetPathClient::CancelResponse::ERROR_NONE)
      {
        setGetPathGoalStatusMessage("Cancelled", status_success);
      } else {
        setGetPathGoalStatusMessage("Failed to cancel active goal", status_error);
      }
      {
        std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
        if(goal_handle_get_path_ == active_goal)
        {
          goal_handle_get_path_.reset();
        }
      }
      sendGetPathGoal(get_path_goal);
    });

  } else {
    sendGetPathGoal(get_path_goal);
  }
}

void MbfGoalActionsPanel::sendGetPathGoal(const mbf_msgs::action::GetPath::Goal & goal)
{
  GetPathClient::SharedPtr client;
  {
    std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
    client = action_client_get_path_;
  }
  if(!client)
  {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Get path action client not initialized when sending goal");
    setGetPathGoalStatusMessage("Goal sending failed: client not ready", status_error);
    return;
  }

  GetPathClient::SendGoalOptions options;
  const auto goal_stamp = goal.target_pose.header.stamp;
  options.goal_response_callback =
    [goal_stamp, this](const GetPathClient::GoalHandle::SharedPtr & goal_handle) {
      {
        std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
        goal_handle_get_path_ = goal_handle;
      }
      if (goal_handle) {
        setGetPathGoalStatusMessage(QString("(t=%1) accepted").arg(goal_stamp.sec), status_success);
      } else {
        setGetPathGoalStatusMessage(QString("(t=%1) rejected").arg(goal_stamp.sec), status_error);
      }
    };
  options.result_callback = std::bind(
    &MbfGoalActionsPanel::getPathResultCallback, this,
    std::placeholders::_1);

  client->async_send_goal(goal, options);
  setGetPathGoalStatusMessage(QString("(t=%1) sent, awaiting response").arg(goal_stamp.sec), status_info);
}

void MbfGoalActionsPanel::getPathResultCallback(
  const GetPathClient::GoalHandle::WrappedResult & wrapped_result)
{
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      setGetPathGoalStatusMessage(QString("Succeeded. Path cost %1").arg(wrapped_result.result->cost), status_success);

      mbf_msgs::action::ExePath::Goal exe_path_goal;
      exe_path_goal.path = wrapped_result.result->path;

      // copy client and active goal under the lock, use the copies outside of it
      ExePathClient::SharedPtr client;
      ExePathClient::GoalHandle::SharedPtr active_goal;
      {
        std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
        exe_path_goal.controller = exe_path_controller_name_; // cached from GUI thread, protected by mutex
        client = action_client_exe_path_;
        active_goal = goal_handle_exe_path_;
      }

      if(!client)
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Exe_path action client not initialized. Cannot send goal.");
        setExePathGoalStatusMessage("Not sent: no controller action selected", status_error);
        break;
      }
      if(!client->action_server_is_ready())
      {
        RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Exe_path action server is not connected. Cannot send goal.");
        setExePathGoalStatusMessage("Not sent: controller server not connected", status_error);
        break;
      }

      if (active_goal) {
        // path execution is currently active, cancel first and send the new goal afterwards
        // UI currently cannot properly handle parallel actions
        setExePathGoalStatusMessage("Cancel existing", status_warning);

        client->async_cancel_goal(active_goal, [this, exe_path_goal, active_goal](
            const typename ExePathClient::CancelResponse::SharedPtr & response)
        {
          if(response->return_code == ExePathClient::CancelResponse::ERROR_NONE)
          {
            setExePathGoalStatusMessage("Existing goal cancelled", status_success);
          } else {
            setExePathGoalStatusMessage("Failed to cancel existing goal", status_error);
          }
          {
            std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
            if(goal_handle_exe_path_ == active_goal)
            {
              goal_handle_exe_path_.reset();
            }
          }
          sendExePathGoal(exe_path_goal);
        });

      } else {
        sendExePathGoal(exe_path_goal);
      }
      break;
    }
    case rclcpp_action::ResultCode::ABORTED:
      setGetPathGoalStatusMessage(QString("Aborted. %1").arg(QString::fromStdString(wrapped_result.result->message)), status_error);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      setGetPathGoalStatusMessage(QString("Cancelled. %1").arg(QString::fromStdString(wrapped_result.result->message)), status_warning);
      break;
    default:
      setGetPathGoalStatusMessage("Error: Unknown action result code", status_error);
      break;
  }

  // only forget the goal handle if it still belongs to this goal (a newer goal may already be active)
  std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
  if(goal_handle_get_path_ && goal_handle_get_path_->get_goal_id() == wrapped_result.goal_id)
  {
    goal_handle_get_path_.reset();
  }
}

void MbfGoalActionsPanel::sendExePathGoal(const mbf_msgs::action::ExePath::Goal & goal)
{
  ExePathClient::SharedPtr client;
  {
    std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
    client = action_client_exe_path_;
  }
  if(!client)
  {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Exe path action client not initialized when sending goal");
    setExePathGoalStatusMessage("Goal sending failed: client not ready", status_error);
    return;
  }

  const auto goal_stamp = goal.path.header.stamp;
  ExePathClient::SendGoalOptions options;
  options.goal_response_callback =
    [this, goal_stamp](const ExePathClient::GoalHandle::SharedPtr & goal_handle) {
      {
        std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
        goal_handle_exe_path_ = goal_handle;
      }
      if (goal_handle) {
        setExePathGoalStatusMessage(QString("Goal (t=%1) accepted").arg(goal_stamp.sec), status_success);
      } else {
        setExePathGoalStatusMessage(QString("Goal (t=%1) rejected").arg(goal_stamp.sec), status_error);
      }
    };
  options.feedback_callback = [this](
    ExePathClient::GoalHandle::SharedPtr,
    const mbf_msgs::action::ExePath::Feedback::ConstSharedPtr feedback)
    {
      setExePathGoalStatusMessage(QString("Executing. Remaining distance: %1m").arg(feedback->dist_to_goal), status_info);
    };
  options.result_callback = std::bind(
    &MbfGoalActionsPanel::exePathResultCallback, this,
    std::placeholders::_1);

  client->async_send_goal(goal, options);
  setExePathGoalStatusMessage(QString("(t=%1) sent, awaiting response").arg(goal_stamp.sec), status_info);
}

void MbfGoalActionsPanel::exePathResultCallback(
  const ExePathClient::GoalHandle::WrappedResult & wrapped_result)
{
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      setExePathGoalStatusMessage(QString("Succeeded (remaining distance: %1)").arg(wrapped_result.result->dist_to_goal), status_success);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if (wrapped_result.result->outcome == mbf_msgs::action::ExePath::Result::ROBOT_STUCK) {
        if (goal_retry_cnt_ < 3) {

          setExePathGoalStatusMessage(QString("Robot stuck. Replanning ... (retry %1/3)").arg(goal_retry_cnt_ + 1), status_warning);

          goal_retry_cnt_ += 1;
          mbf_msgs::action::GetPath::Goal get_path_goal;
          get_path_goal.target_pose = current_goal_;
          {
            std::unique_lock<std::mutex> lock(get_path_action_client_mutex_);
            get_path_goal.planner = get_path_planner_name_; // cached from GUI thread, protected by mutex
          }
          // Overwrite the timestamp to prevent tf extrapolation errors in the planners
          get_path_goal.target_pose.header.stamp = ros_node_->now();
          get_path_goal.use_start_pose = false;
          sendGetPathGoal(get_path_goal);

        } else {
          setExePathGoalStatusMessage(QString("Robot stuck. Maximum retries exceeded! Goal failed!"), status_error);
        }
        break;
      }
      setExePathGoalStatusMessage(QString("Aborted. %1").arg(QString::fromStdString(wrapped_result.result->message)), status_error);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      setExePathGoalStatusMessage(QString("Cancelled. %1").arg(QString::fromStdString(wrapped_result.result->message)), status_warning);
      break;
    default:
      setExePathGoalStatusMessage("Error: Unknown action result code", status_error);
      break;
  }

  // only forget the goal handle if it still belongs to this goal (a newer goal may already be active)
  std::unique_lock<std::mutex> lock(exe_path_action_client_mutex_);
  if(goal_handle_exe_path_ && goal_handle_exe_path_->get_goal_id() == wrapped_result.goal_id)
  {
    goal_handle_exe_path_.reset();
  }
}

void MbfGoalActionsPanel::setGetPathServerStatusMessage(const QString & text, const QString& style)
{
  emit getPathServerStatusChanged(text, style);
}

void MbfGoalActionsPanel::setGetPathGoalStatusMessage(const QString & text, const QString& style)
{
  emit getPathGoalStatusChanged(text, style);
}

void MbfGoalActionsPanel::setExePathServerStatusMessage(const QString & text, const QString& style)
{
  emit exePathServerStatusChanged(text, style);
}

void MbfGoalActionsPanel::setExePathGoalStatusMessage(const QString & text, const QString& style)
{
  emit exePathGoalStatusChanged(text, style);
}

} // namespace rviz_mbf_plugins

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mbf_plugins::MbfGoalActionsPanel, rviz_common::Panel)
