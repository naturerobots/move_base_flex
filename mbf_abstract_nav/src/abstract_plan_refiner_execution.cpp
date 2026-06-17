/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Nature Robots GmbH
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
 *
 *  abstract_plan_refiner.h
 *
 *  author: Georg John <georg.john@naturerobots.com>
 *
 */

#include "mbf_abstract_nav/abstract_plan_refiner_execution.h"


namespace mbf_abstract_nav
{

AbstractPlanRefinerExecution::AbstractPlanRefinerExecution(
  const std::string & name,
  const mbf_abstract_core::AbstractPlanRefiner::Ptr & plan_refiner_ptr,
  const mbf_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp::Node::SharedPtr & node_handle)
: AbstractExecutionBase(name, robot_info, node_handle)
  , plan_refiner_(plan_refiner_ptr)
  , state_(INITIALIZED)
  , max_retries_(0)
  , refining_(false)
  , has_new_plan_(false)
  , node_handle_(node_handle)
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  if (!node_handle_->has_parameter("refiner_max_retries")) {
    param_desc.description =
      "How many times we will recall the refiner in an attempt to find a valid plan before giving up";
    node_handle_->declare_parameter("refiner_max_retries", rclcpp::ParameterValue(-1), param_desc);
  }
  node_handle_->get_parameter("refiner_max_retries", max_retries_);
}

AbstractPlanRefinerExecution::~AbstractPlanRefinerExecution()
{
}

float AbstractPlanRefinerExecution::getPositionError() const
{
  return current_position_error_;
}

float AbstractPlanRefinerExecution::getOrientationError() const
{
  return current_orientation_error_;
}

float AbstractPlanRefinerExecution::getPathLengthRatio() const
{
  return current_path_length_ratio_;
}

typename AbstractPlanRefinerExecution::PlanRefinerState
AbstractPlanRefinerExecution::getState() const
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  return state_;
}

void AbstractPlanRefinerExecution::setState(PlanRefinerState state, bool signalling)
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  state_ = state;

  // we exit refining if we are signalling.
  refining_ = !signalling;

  // some states are quiet, most aren't
  if (signalling) {
    condition_.notify_all();
  }
}

void AbstractPlanRefinerExecution::setNewPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  std::lock_guard<std::mutex> lock(plan_mtx_);
  plan_ = plan;
  has_new_plan_ = true;
}

std::vector<geometry_msgs::msg::PoseStamped> AbstractPlanRefinerExecution::getPlan() const
{
  std::lock_guard<std::mutex> lock(plan_mtx_);
  return plan_;
}

bool AbstractPlanRefinerExecution::start(const std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  if (refining_) {
    return false; // already running
  }
  std::lock_guard<std::mutex> guard(refining_mtx_);
  refining_ = true;

  std::lock_guard<std::mutex> plan_guard(plan_mtx_);
  plan_ = plan;

  RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "Starting plan refiner thread for " << name_);

  return AbstractExecutionBase::start();
}

bool AbstractPlanRefinerExecution::cancel()
{
  cancel_ = true; // force cancel immediately, as the call to cancel in the plan refiner can take a while

  // returns false if cancel is not implemented or rejected by the plan refiner (will run until completion)
  if (!plan_refiner_->cancel()) {
    RCLCPP_DEBUG_STREAM(
      node_handle_->get_logger(),
      "Cancel plan refinement failed or is not supported by the plugin. "
        << "Wait until the current refinement finished!");

    return false;
  }
  return true;
}

uint32_t AbstractPlanRefinerExecution::refinePlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  std::vector<geometry_msgs::msg::PoseStamped> & refined_plan,
  float & position_error,
  float & orientation_error,
  float & path_length_ratio,
  std::string & message)
{
  return plan_refiner_->applyRefinement(
    plan, refined_plan,
    position_error, orientation_error, path_length_ratio,
    message);
}

void AbstractPlanRefinerExecution::run()
{
  setState(STARTED, false);
  std::lock_guard<std::mutex> guard(refining_mtx_);
  int retries = 0;

  std::vector<geometry_msgs::msg::PoseStamped> current_plan(plan_);

  try {
    while (refining_ && rclcpp::ok()) {
      if (should_exit_) {
        // Early exit if should_exit_ is set
        handle_thread_interrupted();
        return;
      }

      float position_error = 0.0;
      float orientation_error = 0.0;
      float path_length_ratio = 0.0;

      // Check if a new plan is available
      {
        std::lock_guard<std::mutex> plan_lock(plan_mtx_);
        if (has_new_plan_) {
          has_new_plan_ = false;
          current_plan = plan_;
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(), "A new plan is available. Refining the new plan!");
        }
      }

      if (cancel_) {
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "The plan refiner \"" << name_ << "\" has been canceled!");
        setState(CANCELED, true);
      } else {
        setState(REFINING, false);

        std::vector<geometry_msgs::msg::PoseStamped> refined_plan;

        outcome_ = refinePlan(
          current_plan, refined_plan, position_error, orientation_error,
          path_length_ratio, message_);

        bool success = outcome_ < 10;

        if (cancel_) {
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "The plan refiner \"" << name_ << "\" has been canceled!"); // but not due to patience exceeded
          setState(CANCELED, true);
        } else if (success) {
          RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "Successfully refined the plan.");

          std::lock_guard<std::mutex> plan_mtx_guard(plan_mtx_);
          plan_ = refined_plan;
          current_position_error_ = position_error;
          current_orientation_error_ = orientation_error;
          current_path_length_ratio_ = path_length_ratio;
          setState(REFINED_PLAN, true);
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "The plan refiner \"" << name_ << "\" failed after " << retries << " retries!");
          setState(MAX_RETRIES, true);
        } else if (max_retries_ == 0) {
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "The plan refiner could not refine the plan!. Check the configuration of the maximum refiner steps.");
          setState(NO_PLAN_FOUND, true);
        } else {
          RCLCPP_DEBUG_STREAM(
            node_handle_->get_logger(), "The plan refiner could not refine the plan! "
            "Trying again...");
        }
      }
    }  // while (refining_ && rclcpp::ok())
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(
      node_handle_->get_logger(),
      "Exception in plan refiner thread: " << e.what());
    setState(INTERNAL_ERROR, true);
    condition_.notify_all();
  }
}

void AbstractPlanRefinerExecution::handle_thread_interrupted()
{
  // Refiner thread interrupted; probably we have exceeded refiner patience
  RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Refiner thread interrupted!");
  setState(STOPPED, true);
  condition_.notify_all();
}

}  /* namespace mbf_abstract_nav */
