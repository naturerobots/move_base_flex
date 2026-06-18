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
 *  simple_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_simple_nav/simple_navigation_server.h"


namespace mbf_simple_nav
{

SimpleNavigationServer::SimpleNavigationServer(
  const TFPtr & tf_listener_ptr,
  const rclcpp::Node::SharedPtr & node)
: mbf_abstract_nav::AbstractNavigationServer(tf_listener_ptr, node)
  , planner_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimplePlanner")
  , plan_refiner_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimplePlanRefiner")
  , controller_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimpleController")
  , recovery_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimpleRecovery")
{
  // initialize all plugins
  initializeServerComponents();
}

SimpleNavigationServer::~SimpleNavigationServer()
{
  // Loaded plugins are held by the action in which they are used and their respective plugin manager.
  // pluginlib::ClassLoaders need to get destructed after all plugins are destructed to avoid any leaks.
  // Therefore, destruct actions and unload plugins here instead of waiting for the base class' destructor.
  planner_action_.reset();
  plan_refiner_action_.reset();
  controller_action_.reset();
  recovery_action_.reset();
  planner_plugin_manager_.clearPlugins();
  plan_refiner_plugin_manager_.clearPlugins();
  controller_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();
}

mbf_abstract_core::AbstractPlanner::Ptr SimpleNavigationServer::loadPlannerPlugin(
  const std::string & planner_type)
{

  // throw std::runtime_error("This method should not be called.");
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  RCLCPP_INFO(node_->get_logger(), "[SimpleNavigationServer] Load global planner plugin.");
  try
  {
    planner_ptr = planner_plugin_loader_.createSharedInstance(planner_type);
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[SimpleNavigationServer] Failed to load the " << planner_type << " planner, are you sure it is properly registered"
                                           << " and that the containing library is built? Exception: " << ex.what());
  }

  if(planner_ptr)
  {
    RCLCPP_INFO(node_->get_logger(), "[SimpleNavigationServer] Global planner plugin loaded.");
  }

  return planner_ptr;
}

bool SimpleNavigationServer::initializePlannerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  mbf_simple_core::SimplePlanner::Ptr simple_planner_ptr =
      std::dynamic_pointer_cast<mbf_simple_core::SimplePlanner>(planner_ptr);
  
  if(!simple_planner_ptr)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[SimpleNavigationServer] Failed to initialize plugin " << name << " as simple planner. The plugin does not seem to be of type mbf_simple_core::SimplePlanner.");
    return false;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Initialize planner \"" << name << "\".");
  simple_planner_ptr->initialize(name, node_);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Planner plugin \"" << name << "\" initialized.");
  
  return true;
}

mbf_abstract_core::AbstractPlanRefiner::Ptr SimpleNavigationServer::loadPlanRefinerPlugin(
  const std::string & plan_refiner_type)
{
  mbf_abstract_core::AbstractPlanRefiner::Ptr plan_refiner_ptr;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Load plan refiner plugin: " << plan_refiner_type);
  try {
    plan_refiner_ptr = plan_refiner_plugin_loader_.createSharedInstance(plan_refiner_type);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL_STREAM(
      node_->get_logger(),
      "Failed to load the " << plan_refiner_type <<
        " plan refiner, are you sure it's properly registered"
                            << " and that the containing library is built? Exception: " <<
        ex.what());
  }
  return plan_refiner_ptr;
}

bool SimpleNavigationServer::initializePlanRefinerPlugin(
  const std::string & name,
  const mbf_abstract_core::AbstractPlanRefiner::Ptr & plan_refiner_ptr)
{
  mbf_simple_core::SimplePlanRefiner::Ptr simple_plan_refiner_ptr =
    std::dynamic_pointer_cast<mbf_simple_core::SimplePlanRefiner>(plan_refiner_ptr);

  if(!simple_plan_refiner_ptr)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[SimpleNavigationServer] Failed to initialize plugin " << name << " as simple plan refiner. The plugin does not seem to be of type mbf_simple_core::SimplePlanRefiner.");
    return false;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Initialize plan refiner \"" << name << "\".");
  simple_plan_refiner_ptr->initialize(name, node_);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Plan refiner plugin \"" << name << "\" initialized.");

  return true;
}

mbf_abstract_core::AbstractController::Ptr SimpleNavigationServer::loadControllerPlugin(
  const std::string & controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  RCLCPP_DEBUG(node_->get_logger(), "[SimpleNavigationServer] Load controller plugin.");
  try {
    controller_ptr = controller_plugin_loader_.createSharedInstance(controller_type);
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "[SimpleNavigationServer] MBF_core-based controller plugin " << controller_type << " loaded");
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL_STREAM(
      node_->get_logger(),
      "[SimpleNavigationServer] Failed to load the " << controller_type <<
        " controller, are you sure it's properly registered"
                            << " and that the containing library is built? Exception: " <<
        ex.what());
  }
  return controller_ptr;
}

bool SimpleNavigationServer::initializeControllerPlugin(
  const std::string & name,
  const mbf_abstract_core::AbstractController::Ptr & controller_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[SimpleNavigationServer] The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_simple_core::SimpleController::Ptr simple_controller_ptr =
      std::dynamic_pointer_cast<mbf_simple_core::SimpleController>(controller_ptr);

  if (!simple_controller_ptr)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[SimpleNavigationServer] Failed to initialize plugin " << name << " as simple controller. The plugin does not seem to be of type mbf_simple_core::SimpleController.");
    return false;
  }
  
  simple_controller_ptr->initialize(name, tf_listener_ptr_, node_);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Controller plugin \"" << name << "\" initialized.");
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr SimpleNavigationServer::loadRecoveryPlugin(
  const std::string & recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try {
    recovery_ptr = recovery_plugin_loader_.createSharedInstance(recovery_type);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL_STREAM(
      node_->get_logger(),
      "[SimpleNavigationServer] Failed to load the " << recovery_type <<
        " recovery behavior, are you sure it's properly registered"
                            <<
        " and that the containing library is built? Exception: " <<
        ex.what());
  }
  return recovery_ptr;
}

bool SimpleNavigationServer::initializeRecoveryPlugin(
  const std::string & name,
  const mbf_abstract_core::AbstractRecovery::Ptr & behavior_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[SimpleNavigationServer] Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[SimpleNavigationServer] The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_simple_core::SimpleRecovery::Ptr behavior =
      std::dynamic_pointer_cast<mbf_simple_core::SimpleRecovery>(behavior_ptr);
  
  if (!behavior)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[SimpleNavigationServer] Failed to initialize plugin " << name << " as simple recovery behavior. The plugin does not seem to be of type mbf_simple_core::SimpleRecovery.");
    return false;
  }
  
  behavior->initialize(name, tf_listener_ptr_, node_);
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "[SimpleNavigationServer] Recovery behavior plugin \"" << name << "\" initialized.");
  return true;
}

} /* namespace mbf_simple_nav */
