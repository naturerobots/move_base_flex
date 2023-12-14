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
 *  abstract_recovery_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <boost/exception/diagnostic_information.hpp>

#include <mbf_abstract_nav/abstract_recovery_execution.h>

namespace mbf_abstract_nav
{

AbstractRecoveryExecution::AbstractRecoveryExecution(const std::string& name,
                                                     const mbf_abstract_core::AbstractRecovery::Ptr& recovery_ptr,
                                                     const mbf_utility::RobotInformation::ConstPtr& robot_info,
                                                     const rclcpp::Node::SharedPtr& node_handle)
  : AbstractExecutionBase(name, robot_info, node_handle),
    behavior_(recovery_ptr),
    state_(INITIALIZED),
    node_handle_(node_handle),
    patience_(0, 0)
{
  // dynamically reconfigurable parameters
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "How much time we allow recovery behaviors to complete before canceling (or stopping if "
                           "cancel fails";
  node_handle_->declare_parameter("recovery_patience", rclcpp::ParameterValue(15.0), param_desc);

  double patience;
  node_handle_->get_parameter("recovery_patience", patience);
  patience_ = rclcpp::Duration::from_seconds(patience);
}

AbstractRecoveryExecution::~AbstractRecoveryExecution()
{
}

rcl_interfaces::msg::SetParametersResult
AbstractRecoveryExecution::reconfigure(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(conf_mtx_);

  rcl_interfaces::msg::SetParametersResult result;

  for (const rclcpp::Parameter& param : parameters)
  {
    const auto& param_name = param.get_name();
    if (param_name == "recovery_patience")
    {
      try
      {
        patience_ = rclcpp::Duration::from_seconds(param.as_double());
      }
      catch (std::exception& ex)
      {
        RCLCPP_ERROR(node_handle_->get_logger(), "Failed to set recovery_patience: %s", ex.what());
        patience_ = rclcpp::Duration(0, 0);
      }
    }
  }
  result.successful = true;
  return result;
}


void AbstractRecoveryExecution::setState(RecoveryState state)
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  state_ = state;
}


typename AbstractRecoveryExecution::RecoveryState AbstractRecoveryExecution::getState()
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  return state_;
}

bool AbstractRecoveryExecution::cancel()
{
  cancel_ = true;
  // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
  if (!behavior_->cancel())
  {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),"Cancel recovery behavior \"" << name_ << "\" failed or is not supported by the plugin. "
                        << "Wait until the current recovery behavior finished!");
    return false;
  }
  return true;
}

bool AbstractRecoveryExecution::isPatienceExceeded()
{
  std::lock_guard<std::mutex> guard1(conf_mtx_);
  std::lock_guard<std::mutex> guard2(time_mtx_);
  RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "Patience: " << patience_.seconds() << ", start time: " << start_time_.seconds()
                                                                                    <<  " now: " << node_handle_->now().seconds());
  return !(patience_ == rclcpp::Duration(0, 0)) && (node_handle_->now() - start_time_ > patience_);
}

void AbstractRecoveryExecution::run()
{
  cancel_ = false; // reset the canceled state

  time_mtx_.lock();
  start_time_ = node_handle_->now();
  time_mtx_.unlock();
  setState(RECOVERING);
  try
  {
    outcome_ = behavior_->runBehavior(message_);
    if (cancel_)
    {
      setState(CANCELED);
    }
    else
    {
      setState(RECOVERY_DONE);
    }
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(node_handle_->get_logger(),
                       "Unknown error occurred in recovery behavior");
    setState(INTERNAL_ERROR);
  }
  condition_.notify_one();
}

void AbstractRecoveryExecution::handle_thread_interrupted()
{
  RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Recovery \"" << name_ << "\" interrupted!");
  setState(STOPPED);
  condition_.notify_one();
}

} /* namespace mbf_abstract_nav */
