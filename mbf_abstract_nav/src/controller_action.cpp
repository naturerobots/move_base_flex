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
 *  controller_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/controller_action.h"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace mbf_abstract_nav
{

ControllerAction::ControllerAction(
    const rclcpp::Node::SharedPtr &node,
    const std::string &action_name,
    const mbf_utility::RobotInformation &robot_info)
    : AbstractActionBase(node, action_name, robot_info)
{
  rcl_interfaces::msg::ParameterDescriptor oscillation_timeout_description;
  rcl_interfaces::msg::FloatingPointRange oscillation_timeout_range;
  oscillation_timeout_range.from_value = 0.0;
  oscillation_timeout_range.to_value = 60.0;
  oscillation_timeout_range.step = 0.0;
  oscillation_timeout_description.description = "How long in seconds to allow for oscillation before executing recovery behaviors";
  oscillation_timeout_description.floating_point_range.push_back(oscillation_timeout_range);
  node->declare_parameter<double>("oscillation_timeout", 0.0, oscillation_timeout_description);

  rcl_interfaces::msg::ParameterDescriptor oscillation_distance_description;
  rcl_interfaces::msg::FloatingPointRange oscillation_distance_range;
  oscillation_distance_range.from_value = 0.0;
  oscillation_distance_range.to_value = 10.0;
  oscillation_distance_range.step = 0.0;
  oscillation_distance_description.description = "How far in meters the robot must move to be considered not to be oscillating";
  oscillation_distance_description.floating_point_range.push_back(oscillation_distance_range);
  node->declare_parameter<double>("oscillation_distance", 0.5, oscillation_distance_description);
}

void ControllerAction::start(
    GoalHandlePtr goal_handle,
    typename AbstractControllerExecution::Ptr execution_ptr
)
{
  if(goal_handle->is_canceling())
  {
    Action::Result::SharedPtr result = std::make_shared<Action::Result>();
    goal_handle->canceled(result); // TODO why trigger cancel if the goal is already being canceled?
  }

  uint8_t slot = goal_handle->get_goal()->concurrency_slot;

  bool update_plan = false;
  slot_map_mtx_.lock();
  std::map<uint8_t, ConcurrencySlot>::iterator slot_it = concurrency_slots_.find(slot);
  if(slot_it != concurrency_slots_.end() && slot_it->second.in_use)
  {
    boost::lock_guard<boost::mutex> goal_guard(goal_mtx_);
    if ((slot_it->second.execution->getName() == goal_handle->get_goal()->controller ||
         goal_handle->get_goal()->controller.empty()) &&
         slot_it->second.goal_handle->is_active())
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Updating running controller goal of slot " << static_cast<int>(slot));
      update_plan = true;
      // Goal requests to run the same controller on the same concurrency slot already in use:
      // we update the goal handle and pass the new plan and tolerances from the action to the
      // execution without stopping it
      execution_ptr = slot_it->second.execution;
      execution_ptr->setNewPlan(goal_handle->get_goal()->path.poses,
                                goal_handle->get_goal()->tolerance_from_action,
                                goal_handle->get_goal()->dist_tolerance,
                                goal_handle->get_goal()->angle_tolerance);
      // Update also goal pose, so the feedback remains consistent
      goal_pose_ = goal_handle->get_goal()->path.poses.back();
      mbf_msgs::action::ExePath::Result result;
      fillExePathResult(mbf_msgs::action::ExePath::Result::CANCELED, "Goal preempted by a new plan", result);
      concurrency_slots_[slot].goal_handle->setCanceled(result, result.message);
      concurrency_slots_[slot].goal_handle = goal_handle;
      concurrency_slots_[slot].goal_handle->setAccepted();
    }
  }
  slot_map_mtx_.unlock();
  if(!update_plan)
  {
    // Otherwise run parent version of this method
    AbstractActionBase::start(goal_handle, execution_ptr);
  }
}

void ControllerAction::runImpl(GoalHandle &goal_handle, AbstractControllerExecution &execution)
{
  goal_mtx_.lock();
  // Note that we always use the goal handle stored on the concurrency slots map, as it can change when replanning
  uint8_t slot = goal_handle->get_goal()->concurrency_slot;
  goal_mtx_.unlock();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Start action "  << name_);

  // ensure we don't provide values from previous execution on case of error before filling both poses
  goal_pose_ = geometry_msgs::msg::PoseStamped();
  robot_pose_ = geometry_msgs::msg::PoseStamped();

  ros::NodeHandle private_nh("~");

  double oscillation_timeout_tmp;
  private_nh.param("oscillation_timeout", oscillation_timeout_tmp, 0.0);
  ros::Duration oscillation_timeout(oscillation_timeout_tmp);

  double oscillation_distance;
  private_nh.param("oscillation_distance", oscillation_distance, 0.03);

  double oscillation_angle;
  private_nh.param("oscillation_angle", oscillation_angle, M_PI);

  mbf_msgs::action::ExePath::Result result;
  mbf_msgs::action::ExePath::Feedback feedback;

  typename AbstractControllerExecution::ControllerState state_moving_input;
  bool controller_active = true;

  goal_mtx_.lock();
  const mbf_msgs::action::ExePath::Goal &goal = *(goal_handle->get_goal().get());

  const std::vector<geometry_msgs::msg::PoseStamped> &plan = goal.path.poses;
  if (plan.empty())
  {
    fillExePathResult(mbf_msgs::action::ExePath::Result::INVALID_PATH, "Controller started with an empty plan!", result);
    goal_handle->setAborted(result, result.message);
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result.message << " Canceling the action call.");
    controller_active = false;
    goal_mtx_.unlock();
    return;
  }

  goal_pose_ = plan.back();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Called action \""
      << name_ << "\" with plan:" << std::endl
      << "frame: \"" << goal.path.header.frame_id << "\" " << std::endl
      << "stamp: " << goal.path.header.stamp << std::endl
      << "poses: " << goal.path.poses.size() << std::endl
      << "goal: (" << goal_pose_.pose.position.x << ", "
      << goal_pose_.pose.position.y << ", "
      << goal_pose_.pose.position.z << ")");

  goal_mtx_.unlock();


  geometry_msgs::msg::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  while (controller_active && ros::ok())
  {
    // goal_handle could change between the loop cycles due to adapting the plan
    // with a new goal received for the same concurrency slot
    if (!robot_info_.getRobotPose(robot_pose_))
    {
      controller_active = false;
      fillExePathResult(mbf_msgs::action::ExePath::Result::TF_ERROR, "Could not get the robot pose!", result);
      goal_mtx_.lock();
      goal_handle->setAborted(result, result.message);
      goal_mtx_.unlock();
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result.message << " Canceling the action call.");
      break;
    }

    if (first_cycle)
    {
      // init oscillation pose
      oscillation_pose = robot_pose_;
    }

    goal_mtx_.lock();
    state_moving_input = execution.getState();

    switch (state_moving_input)
    {
      case AbstractControllerExecution::INITIALIZED:
        execution.setNewPlan(plan, goal.tolerance_from_action, goal.dist_tolerance, goal.angle_tolerance);
        execution.start();
        break;

      case AbstractControllerExecution::STOPPED:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has been stopped rigorously!");
        controller_active = false;
        result.outcome = mbf_msgs::action::ExePath::Result::STOPPED;
        result.message = "Controller has been stopped!";
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::CANCELED:
        RCLCPP_INFO_STREAM("Action \"exe_path\" canceled");
        fillExePathResult(mbf_msgs::action::ExePath::Result::CANCELED, "Controller canceled", result);
        goal_handle->setCanceled(result, result.message);
        controller_active = false;
        break;

      case AbstractControllerExecution::STARTED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "The moving has been started!");
        break;

      case AbstractControllerExecution::PLANNING:
        if (execution.isPatienceExceeded())
        {
          RCLCPP_INFO_STREAM("Try to cancel the plugin \"" << name_ << "\" after the patience time has been exceeded!");
          if (execution.cancel())
          {
            RCLCPP_INFO_STREAM("Successfully canceled the plugin \"" << name_ << "\" after the patience time has been exceeded!");
          }
        }
        break;

      case AbstractControllerExecution::MAX_RETRIES:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has been aborted after it exceeded the maximum number of retries!");
        controller_active = false;
        fillExePathResult(execution.getOutcome(), execution.getMessage(), result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::PAT_EXCEEDED:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has been aborted after it exceeded the patience time");
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::PAT_EXCEEDED, execution.getMessage(), result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_PLAN:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has been started without a plan!");
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::INVALID_PATH, "Controller started without a path", result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::EMPTY_PLAN:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has received an empty plan");
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::INVALID_PATH, "Controller started with an empty plan", result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::INVALID_PLAN:
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller has received an invalid plan");
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::INVALID_PATH, "Controller started with an invalid plan", result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_LOCAL_CMD:
        RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger(name_), node_->get_clock(), 3000, 
                                    "No velocity command received from controller! " 
                                    << execution.getMessage());
        controller_active = execution.isMoving();
        if (!controller_active)
        {
          fillExePathResult(execution.getOutcome(), execution.getMessage(), result);
          goal_handle->setAborted(result, result.message);
        }
        else
        {
          publishExePathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(),
                                 execution.getVelocityCmd());
        }
        break;

      case AbstractControllerExecution::GOT_LOCAL_CMD:
        if (!oscillation_timeout.isZero())
        {
          // check if oscillating
          if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance ||
              mbf_utility::angle(robot_pose_, oscillation_pose) >= oscillation_angle)
          {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose_;
          }
          else if (last_oscillation_reset + oscillation_timeout < ros::Time::now())
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "The controller is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");

            execution.cancel();
            controller_active = false;
            fillExePathResult(mbf_msgs::action::ExePath::Result::OSCILLATION, "Oscillation detected!", result);
            goal_handle->setAborted(result, result.message);
            break;
          }
        }
        publishExePathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
        break;

      case AbstractControllerExecution::ARRIVED_GOAL:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Controller succeeded; arrived at goal");
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::SUCCESS, "Controller succeeded; arrived at goal!", result);
        goal_handle->setSucceeded(result, result.message);
        break;

      case AbstractControllerExecution::INTERNAL_ERROR:
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), "Internal error: Unknown error thrown by the plugin: " << execution.getMessage());
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::INTERNAL_ERROR, "Internal error: Unknown error thrown by the plugin!", result);
        goal_handle->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::ROBOT_DISABLED:
        controller_active = false;
        fillExePathResult(mbf_msgs::action::ExePath::Result::ROBOT_STUCK,
                          "Robot ignored velocity commands for more than tolerance time!", result);
        goal_handle->setAborted(result, result.message);
        break;

      default:
        std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex controller execution with the number: "
           << static_cast<int>(state_moving_input);
        fillExePathResult(mbf_msgs::action::ExePath::Result::INTERNAL_ERROR, ss.str(), result);
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), result.message);
        goal_handle->setAborted(result, result.message);
        controller_active = false;
    }
    goal_mtx_.unlock();

    if (controller_active)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the controller execution thread
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }

    first_cycle = false;
  }  // while (controller_active && ros::ok())

  if (!controller_active)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    // normal on continuous replanning
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action has been stopped!");
  }
}

void ControllerAction::publishExePathFeedback(
        GoalHandle &goal_handle,
        uint32_t outcome, const std::string &message,
        const geometry_msgs::msg::TwistStamped &current_twist)
{
  mbf_msgs::action::ExePath::Feedback feedback;
  feedback.outcome = outcome;
  feedback.message = message;

  feedback.last_cmd_vel = current_twist;
  if (feedback.last_cmd_vel.header.stamp.isZero())
    feedback.last_cmd_vel.header.stamp = ros::Time::now();

  feedback.current_pose = robot_pose_;
  feedback.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  feedback.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
  goal_handle->publishFeedback(feedback);
}

void ControllerAction::fillExePathResult(
        uint32_t outcome, const std::string &message,
        mbf_msgs::action::ExePath::Result &result)
{
  result.outcome = outcome;
  result.message = message;
  result.final_pose = robot_pose_;
  result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
}

} /* mbf_abstract_nav */
