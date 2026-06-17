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

 #include <sstream>

 #include "mbf_abstract_nav/plan_refiner_action.h"

namespace mbf_abstract_nav
{
PlanRefinerAction::PlanRefinerAction(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const mbf_utility::RobotInformation::ConstPtr & robot_info)
: AbstractActionBase(node, name, robot_info)
{
}

void PlanRefinerAction::runImpl(
  const GoalHandlePtr & goal_handle,
  AbstractPlanRefinerExecution & execution)
{
  const mbf_msgs::action::RefinePath::Goal & goal = *(goal_handle->get_goal().get());

  mbf_msgs::action::RefinePath::Result::SharedPtr result =
    std::make_shared<mbf_msgs::action::RefinePath::Result>();

  result->refined_path.header.frame_id =
    goal.path.header.frame_id.empty() ? robot_info_->getGlobalFrame() : goal.path.header.frame_id;

  const std::vector<geometry_msgs::msg::PoseStamped> & plan = goal.path.poses;
  AbstractPlanRefinerExecution::PlanRefinerState plan_refiner_state;

  bool refiner_active = true;
  while (refiner_active && rclcpp::ok()) {
    // get the current state of the refining thread
    plan_refiner_state = execution.getState();

    if (goal_handle->is_canceling()) { // action client requested to cancel the action and our server accepted that request
      result->outcome = mbf_msgs::action::RefinePath::Result::CANCELED;
      result->message = "Plan refiner action canceled by client.";
      refiner_active = false;
      execution.stop();
      execution.join();
      goal_handle->canceled(result);
      return;
    }

    switch (plan_refiner_state) {
      case AbstractPlanRefinerExecution::INITIALIZED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Plan refiner initialized.");
        if (!execution.start(plan)) {
          result->outcome = mbf_msgs::action::RefinePath::Result::INTERNAL_ERROR;
          result->message = "Plan refiner could not be started. Another thread is still running!";
          goal_handle->abort(result);
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(name_), result->message << " Canceling the action call.");
          refiner_active = false;
        }
        break;

      case AbstractPlanRefinerExecution::STARTED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Plan refiner started.");
        break;

      case AbstractPlanRefinerExecution::STOPPED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Plan refiner stopped.");
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger(name_), "Plan refiner has been stopped rigorously!");
        result->outcome = mbf_msgs::action::RefinePath::Result::STOPPED;
        result->message = "Plan refiner has been stopped!";
        goal_handle->abort(result);
        refiner_active = false;
        break;

      case AbstractPlanRefinerExecution::CANCELED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Plan refiner canceled.");
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_), "Plan refiner has been canceled successfully");
        result->refined_path.header.stamp = node_->now();
        result->outcome = mbf_msgs::action::RefinePath::Result::CANCELED;
        result->message = "Plan refiner has been canceled!";
        goal_handle->abort(result);
        refiner_active = false;
        break;

      case AbstractPlanRefinerExecution::REFINING: // in progress
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger(name_), *node_->get_clock(), 2000,
          "Plan refiner is refining the plan.");
        break;

      case AbstractPlanRefinerExecution::REFINED_PLAN:  // found a refined plan
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(
            name_),
          "Plan refiner found a refined plan with path length ratio: " <<
            execution.getPathLengthRatio());

        result->refined_path.header.stamp = node_->now();
        result->refined_path.poses = execution.getPlan();
        result->final_position_error = execution.getPositionError();
        result->final_orientation_error = execution.getOrientationError();
        result->path_length_ratio = execution.getPathLengthRatio();
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle->succeed(result);

        refiner_active = false;
        break;


      case AbstractPlanRefinerExecution::NO_PLAN_FOUND:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Plan refiner state: no plan found");
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle->abort(result);
        refiner_active = false;
        break;

      case AbstractPlanRefinerExecution::MAX_RETRIES:
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(
            name_), "Plan refiner reached the maximum number of retries");
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle->abort(result);
        refiner_active = false;
        break;

      case AbstractPlanRefinerExecution::INTERNAL_ERROR:
        RCLCPP_FATAL_STREAM(
          rclcpp::get_logger(
            name_), "Internal error: Unknown error thrown by the plugin!");                                    // TODO getMessage from planning
        refiner_active = false;
        result->outcome = mbf_msgs::action::RefinePath::Result::INTERNAL_ERROR;
        result->message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle->abort(result);
        break;

      default:
        result->outcome = mbf_msgs::action::RefinePath::Result::INTERNAL_ERROR;
        std::ostringstream ss;
        ss <<
          "Internal error: Unknown state in a move base flex refiner execution with the number: "
           << static_cast<int>(plan_refiner_state);
        result->message = ss.str();
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), result->message);
        goal_handle->abort(result);
        refiner_active = false;
    }

    if (refiner_active) {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      execution.waitForStateUpdate(std::chrono::milliseconds(500));
    }
  }  // while (refiner_active && rclcpp::ok())

  if (!refiner_active) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action ended properly.");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action has been stopped!");
  }
}
}   /* namespace mbf_abstract_nav */
