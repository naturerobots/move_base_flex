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

SimpleNavigationServer::SimpleNavigationServer(const TFPtr &tf_listener_ptr, const rclcpp::Node::SharedPtr& node)
  : mbf_abstract_nav::AbstractNavigationServer(tf_listener_ptr, node)
  , planner_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractPlanner")
  , controller_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractController")
  , recovery_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractRecovery")
{
  // initialize all plugins
  initializeServerComponents();
}

mbf_abstract_core::AbstractPlanner::Ptr SimpleNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  RCLCPP_INFO(node_->get_logger(), "Load global planner plugin.");
  try
  {
    planner_ptr = planner_plugin_loader_.createSharedInstance(planner_type);
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Failed to load the " << planner_type << " planner, are you sure it is properly registered"
                                           << " and that the containing library is built? Exception: " << ex.what());
  }
  RCLCPP_INFO(node_->get_logger(), "Global planner plugin loaded.");

  return planner_ptr;
}

bool SimpleNavigationServer::initializePlannerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractPlanner::Ptr&  planner_ptr
)
{
  return true;
}


mbf_abstract_core::AbstractController::Ptr SimpleNavigationServer::loadControllerPlugin(
    const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  RCLCPP_DEBUG(node_->get_logger(), "Load controller plugin.");
  try
  {
    controller_ptr = controller_plugin_loader_.createSharedInstance(controller_type);
    RCLCPP_INFO_STREAM(node_->get_logger(), "MBF_core-based local planner plugin " << controller_type << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Failed to load the " << controller_type
                                           << " local planner, are you sure it's properly registered"
                                           << " and that the containing library is built? Exception: " << ex.what());
  }
  return controller_ptr;
}

bool SimpleNavigationServer::initializeControllerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr SimpleNavigationServer::loadRecoveryPlugin(
    const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try
  {
    recovery_ptr = recovery_plugin_loader_.createSharedInstance(recovery_type);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Failed to load the " << recovery_type << " recovery behavior, are you sure it's properly registered"
                                           << " and that the containing library is built? Exception: " << ex.what());
  }
  return recovery_ptr;
}

bool SimpleNavigationServer::initializeRecoveryPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  return true;
}


SimpleNavigationServer::~SimpleNavigationServer()
{
}

} /* namespace mbf_simple_nav */
