/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  abstract_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <nav_msgs/msg/path.hpp>

#include "mbf_abstract_nav/abstract_navigation_server.h"

namespace mbf_abstract_nav
{
using namespace std::placeholders;

AbstractNavigationServer::AbstractNavigationServer(const TFPtr &tf_listener_ptr, const rclcpp::Node::SharedPtr& node)
    : tf_listener_ptr_(tf_listener_ptr), node_(node),
      planner_plugin_manager_("planners",
          std::bind(&AbstractNavigationServer::loadPlannerPlugin, this, _1),
          std::bind(&AbstractNavigationServer::initializePlannerPlugin, this, _1, _2)),
      controller_plugin_manager_("controllers",
          std::bind(&AbstractNavigationServer::loadControllerPlugin, this, _1),
          std::bind(&AbstractNavigationServer::initializeControllerPlugin, this, _1, _2)),
      recovery_plugin_manager_("recovery_behaviors",
          std::bind(&AbstractNavigationServer::loadRecoveryPlugin, this, _1),
          std::bind(&AbstractNavigationServer::initializeRecoveryPlugin, this, _1, _2)),
      controller_action_(name_action_exe_path, robot_info_),
      planner_action_(name_action_get_path, robot_info_),
      recovery_action_(name_action_recovery, robot_info_),
      move_base_action_(name_action_move_base, robot_info_, recovery_plugin_manager_.getLoadedNames())
{
  node_->declare_parameter<double>("tf_timeout", 3.0);
  node_->declare_parameter<std::string>("global_frame", "map");
  node_->declare_parameter<std::string>("robot_frame", "base_link");
  node_->declare_parameter<std::string>("odom_topic", "odom");

  double tf_timeout_s;  
  node_->get_parameter("tf_timeout",tf_timeout_s);
  tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_s);
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_frame", robot_frame_);

  robot_info_(node_, *tf_listener_ptr, global_frame_, robot_frame_,
              tf_timeout_, node_->get_parameter("odom_topic").as_string());
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", 1);

  // init cmd_vel publisher for the robot velocity
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  action_server_get_path_ptr_ = ActionServerGetPathPtr(
    new ActionServerGetPath(
      node_,
      name_action_get_path,
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::handleGoalGetPath, this, _1, _2),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionGetPath, this, _1),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionGetPath, this, _1)));

  action_server_exe_path_ptr_ = ActionServerExePathPtr(
    new ActionServerExePath(
      node_,
      name_action_exe_path,
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::handleGoalExePath, this, _1, _2),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionExePath, this, _1),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionExePath, this, _1)));

  action_server_recovery_ptr_ = ActionServerRecoveryPtr(
    new ActionServerRecovery(
      node_,
      name_action_recovery,
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::handleGoalRecovery, this, _1, _2),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionRecovery, this, _1),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionRecovery, this, _1)));

  action_server_move_base_ptr_ = ActionServerMoveBasePtr(
    new ActionServerMoveBase(
      node_,
      name_action_move_base,
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::handleGoalMoveBase, this, _1, _2),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionMoveBase, this, _1),
      std::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionMoveBase, this, _1)));

  // XXX note that we don't start a dynamic reconfigure server, to avoid colliding with the one possibly created by
  // the base class. If none, it should call startDynamicReconfigureServer method to start the one defined here for
  // providing just the abstract server parameters
}

void AbstractNavigationServer::initializeServerComponents()
{
  planner_plugin_manager_.loadPlugins();
  controller_plugin_manager_.loadPlugins();
  recovery_plugin_manager_.loadPlugins();
}

AbstractNavigationServer::~AbstractNavigationServer()
{

}

virtual void AbstractNavigationServer::handleGoalGetPath(const rclcpp_action::GoalUUID uuid, std::shared_ptr<const mbf_msgs::action::GetPath::Goal> goal) {

}

void AbstractNavigationServer::callActionGetPath(std::shared_ptr<ActionServerGetPath::GoalHandle> goal_handle)
{
  const mbf_msgs::action::GetPath::Goal &goal = *(goal_handle->getGoal().get());
  const geometry_msgs::msg::Point &p = goal.target_pose.pose.position;

  std::string planner_name;
  if(!planner_plugin_manager_.getLoadedNames().empty())
  {
    planner_name = goal.planner.empty() ? planner_plugin_manager_.getLoadedNames().front() : goal.planner;
  }
  else
  {
    mbf_msgs::action::GetPath::Result result;
    result.outcome = mbf_msgs::action::GetPath::Result::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    RCLCPP_WARN_STREAM(rclcpp::get_logger("get_path"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  } // TODO move rejection code to handleGoal CB

  if(!planner_plugin_manager_.hasPlugin(planner_name))
  {
    mbf_msgs::action::GetPath::Result result;
    result.outcome = mbf_msgs::action::GetPath::Result::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.planner + "\"!";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("get_path"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractPlanner::Ptr planner_plugin = planner_plugin_manager_.getPlugin(planner_name);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("get_path"), "Start action \"get_path\" using planner \"" << planner_name
                        << "\" of type \"" << planner_plugin_manager_.getType(planner_name) << "\"");


  if(planner_plugin)
  {
    mbf_abstract_nav::AbstractPlannerExecution::Ptr planner_execution
        = newPlannerExecution(planner_name, planner_plugin);

    //start another planning action
    planner_action_.start(goal_handle, planner_execution);
  }
  else
  {
    mbf_msgs::action::GetPath::Result result;
    result.outcome = mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
    result.message = "Internal Error: \"planner_plugin\" pointer should not be a null pointer!";
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("get_path"), result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionGetPath(ServerGoalHandleGetPathPtr goal_handle)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("get_path"), "Cancel action \"get_path\"");
  planner_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionExePath(ServerGoalHandleExePathPtr goal_handle)
{
  const mbf_msgs::action::ExePath::Goal &goal = *(goal_handle.getGoal().get());

  std::string controller_name;
  if(!controller_plugin_manager_.getLoadedNames().empty())
  {
    controller_name = goal.controller.empty() ? controller_plugin_manager_.getLoadedNames().front() : goal.controller;
  }
  else
  {
    mbf_msgs::action::ExePath::Result result;
    result.outcome = mbf_msgs::action::ExePath::Result::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    RCLCPP_WARN_STREAM(rclcpp::get_logger("exe_path"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if(!controller_plugin_manager_.hasPlugin(controller_name))
  {
    mbf_msgs::action::ExePath::Result result;
    result.outcome = mbf_msgs::action::ExePath::Result::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.controller + "\"!";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("exe_path"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractController::Ptr controller_plugin = controller_plugin_manager_.getPlugin(controller_name);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("exe_path"), "Start action \"exe_path\" using controller \"" << controller_name
                        << "\" of type \"" << controller_plugin_manager_.getType(controller_name) << "\"");


  if(controller_plugin)
  {
    mbf_abstract_nav::AbstractControllerExecution::Ptr controller_execution
        = newControllerExecution(controller_name, controller_plugin);

    // starts another controller action
    controller_action_.start(goal_handle, controller_execution);
  }
  else
  {
    mbf_msgs::action::ExePath::Result result;
    result.outcome = mbf_msgs::action::ExePath::Result::INTERNAL_ERROR;
    result.message = "Internal Error: \"controller_plugin\" pointer should not be a null pointer!";
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("exe_path"), result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionExePath(ServerGoalHandleExePathPtr goal_handle)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("exe_path"), "Cancel action \"exe_path\"");
  controller_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionRecovery(ServerGoalHandleRecoveryPtr goal_handle)
{
  const mbf_msgs::action::Recovery::Goal &goal = *(goal_handle.getGoal().get());

  std::string recovery_name;

  if(!recovery_plugin_manager_.getLoadedNames().empty())
  {
    recovery_name = goal.behavior.empty() ? recovery_plugin_manager_.getLoadedNames().front() : goal.behavior;
  }
  else
  {
    mbf_msgs::action::Recovery::Result result;
    result.outcome = mbf_msgs::action::Recovery::Result::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    RCLCPP_WARN_STREAM(rclcpp::get_logger("recovery"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if(!recovery_plugin_manager_.hasPlugin(recovery_name))
  {
    mbf_msgs::action::Recovery::Result result;
    result.outcome = mbf_msgs::Recovery::Result::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.behavior + "\"!";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("recovery"), result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractRecovery::Ptr recovery_plugin = recovery_plugin_manager_.getPlugin(recovery_name);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("recovery"), "Start action \"recovery\" using recovery \"" << recovery_name
                        << "\" of type \"" << recovery_plugin_manager_.getType(recovery_name) << "\"");


  if(recovery_plugin)
  {
    mbf_abstract_nav::AbstractRecoveryExecution::Ptr recovery_execution
        = newRecoveryExecution(recovery_name, recovery_plugin);

    recovery_action_.start(goal_handle, recovery_execution);
  }
  else
  {
    mbf_msgs::action::Recovery::Result result;
    result.outcome = mbf_msgs::action::Recovery::Result::INTERNAL_ERROR;
    result.message = "Internal Error: \"recovery_plugin\" pointer should not be a null pointer!";
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("recovery"), result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionRecovery(ServerGoalHandleRecoveryPtr goal_handle)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("recovery"), "Cancel action \"recovery\"");
  recovery_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionMoveBase(ServerGoalHandleMoveBasePtr goal_handle)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Start action \"move_base\"");
  move_base_action_.start(goal_handle);
}

void AbstractNavigationServer::cancelActionMoveBase(ServerGoalHandleMoveBasePtr goal_handle)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "Cancel action \"move_base\"");
  move_base_action_.cancel();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Cancel action \"move_base\" completed");
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr AbstractNavigationServer::newPlannerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractPlanner::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractPlannerExecution>(plugin_name, plugin_ptr,
                                                                        robot_info_, last_config_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr AbstractNavigationServer::newControllerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractController::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractControllerExecution>(plugin_name, plugin_ptr, robot_info_,
                                                                           vel_pub_, goal_pub_, last_config_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr AbstractNavigationServer::newRecoveryExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractRecovery::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractRecoveryExecution>(plugin_name, plugin_ptr,
                                                                         robot_info_, last_config_);
}

void AbstractNavigationServer::startActionServers()
{
  action_server_get_path_ptr_->start();
  action_server_exe_path_ptr_->start();
  action_server_recovery_ptr_->start();
  action_server_move_base_ptr_->start();
}

// TODO add restore_defaults functionality again
// void AbstractNavigationServer::reconfigure(
//   mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level)
// {
//   boost::lock_guard<boost::mutex> guard(configuration_mutex_);
// 
//   // Make sure we have the original configuration the first time we're called, so we can restore it if needed
//   if (!setup_reconfigure_)
//   {
//     default_config_ = config;
//     setup_reconfigure_ = true;
//   }
// 
//   
//   if (config.restore_defaults)
//   {
//     config = default_config_;
//     // if someone sets restore defaults on the parameter server, prevent looping
//     config.restore_defaults = false;
//   }
// TODO can probably be removed - composite classes can use node handle to register callbacks for reacting on param changes themselves.
//   planner_action_.reconfigureAll(config, level);
//   controller_action_.reconfigureAll(config, level);
//   recovery_action_.reconfigureAll(config, level);
//   move_base_action_.reconfigure(config, level);
// 
//   last_config_ = config;
// }

void AbstractNavigationServer::stop(){
  planner_action_.cancelAll();
  controller_action_.cancelAll();
  recovery_action_.cancelAll();
  move_base_action_.cancel();
}

} /* namespace mbf_abstract_nav */
