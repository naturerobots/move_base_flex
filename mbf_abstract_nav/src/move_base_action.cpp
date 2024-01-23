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
 *  move_base_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <limits>
#include <chrono>

#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/move_base_action.h"

namespace mbf_abstract_nav
{

//! helper function for prettier debug output
std::string resultCodeToString(rclcpp_action::ResultCode result_code)
{
  switch (result_code)
  {
    case rclcpp_action::ResultCode::UNKNOWN:
    return "UNKNOWN";
    case rclcpp_action::ResultCode::SUCCEEDED:
    return "SUCCEEDED";
    case rclcpp_action::ResultCode::CANCELED:
    return "CANCELED";
    case rclcpp_action::ResultCode::ABORTED:
    return "ABORTED";
  }
  // should be unreachable code. If you see this, add new ResultCode to the switch case above.
  return "NEW-RESULT-CODE-ADDED?";
}

using namespace std::placeholders;

MoveBaseAction::MoveBaseAction(const rclcpp::Node::SharedPtr &node, const std::string& name,
                               const mbf_utility::RobotInformation::ConstPtr& robot_info, const std::vector<std::string>& behaviors)
  : name_(name)
  , robot_info_(robot_info)
  , node_(node)
  , oscillation_timeout_(0, 0)
  , oscillation_distance_(0)
  , replanning_thread_shutdown_(false)
  , recovery_enabled_(true)
  , behaviors_(behaviors)
  , action_state_(NONE)
  , recovery_trigger_(NONE)
  , dist_to_goal_(std::numeric_limits<double>::infinity())
  , replanning_period_(0, 0)
  , replanning_thread_(std::bind(&MoveBaseAction::replanningThread, this))
{ 
  action_client_exe_path_ = rclcpp_action::create_client<ExePath>(node_, name_action_exe_path);
  action_client_get_path_ = rclcpp_action::create_client<GetPath>(node_, name_action_get_path);
  action_client_recovery_ = rclcpp_action::create_client<Recovery>(node_, name_action_recovery);
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&MoveBaseAction::reconfigure, this, _1));

  get_path_send_goal_options_.goal_response_callback = std::bind(&MoveBaseAction::actionGetPathGoalResponse, this, _1);
  get_path_send_goal_options_.result_callback = std::bind(&MoveBaseAction::actionGetPathResult, this, _1);

  exe_path_send_goal_options_.goal_response_callback = std::bind(&MoveBaseAction::actionExePathGoalResponse, this, _1);
  exe_path_send_goal_options_.feedback_callback = std::bind(&MoveBaseAction::actionExePathFeedback, this, _1, _2);
  exe_path_send_goal_options_.result_callback = std::bind(&MoveBaseAction::actionExePathResult, this, _1),

  recovery_send_goal_options_.goal_response_callback = std::bind(&MoveBaseAction::actionRecoveryGoalResponse, this, _1);
  recovery_send_goal_options_.result_callback = std::bind(&MoveBaseAction::actionRecoveryResult, this, _1);
}

MoveBaseAction::~MoveBaseAction()
{
  action_state_ = NONE;
  replanning_thread_shutdown_ = true;
  if (replanning_thread_.joinable())
  {
    replanning_thread_.join();
  }
}

rcl_interfaces::msg::SetParametersResult MoveBaseAction::reconfigure(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const rclcpp::Parameter &param : parameters) 
  {
    const auto &param_name = param.get_name();
    if (param_name == "planner_frequency") 
    {
      const double new_planner_frequency = param.as_double();
      if (new_planner_frequency > 0.0)
        replanning_period_ = rclcpp::Duration::from_seconds(1.0 / new_planner_frequency);
      else
        replanning_period_ = rclcpp::Duration::from_seconds(0.0);
    }
    else if (param_name == "oscillation_timeout") 
    {
      oscillation_timeout_ = rclcpp::Duration::from_seconds(param.as_double());
    }
    else if (param_name == "oscillation_distance") 
    {
      oscillation_distance_ = param.as_double();
    }
    else if (param_name == "recovery_enabled") 
    {
      recovery_enabled_ = param.as_bool();
    }
  }
  result.successful = true;
  return result;
}

void MoveBaseAction::cancel()
{
  action_state_ = CANCELED;

  action_client_get_path_->async_cancel_all_goals();
  action_client_exe_path_->async_cancel_all_goals();
  action_client_recovery_->async_cancel_all_goals();
}

void MoveBaseAction::start(std::shared_ptr<GoalHandle> goal_handle)
{
  dist_to_goal_ = std::numeric_limits<double>::infinity();

  action_state_ = GET_PATH;

  goal_handle_ = goal_handle;
  // TODO 
  // Accepting goals needs a separate callback now.
  // Smallest change would be to adapt abstract_navigation_server to always accept goals.
  // Current implementation (based on ROS1) aborts the goal handle in this method.
  // Better (with ROS2): Move logic for aborting goal into new accept check method.
  //goal_handle_.setAccepted();


  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Start action \"move_base\"");

  const mbf_msgs::action::MoveBase::Goal& goal = *(goal_handle->get_goal());

  mbf_msgs::action::MoveBase::Result::SharedPtr move_base_result = std::make_shared<mbf_msgs::action::MoveBase::Result>();

  get_path_goal_.target_pose = goal.target_pose;
  get_path_goal_.use_start_pose = false; // use the robot pose
  get_path_goal_.planner = goal.planner;
  exe_path_goal_.controller = goal.controller;

  const auto connection_timeout = std::chrono::seconds(1);

  last_oscillation_reset_ = node_->now();

  // start recovering with the first behavior, use the recovery behaviors from the action request, if specified,
  // otherwise, use all loaded behaviors.

  recovery_behaviors_ = goal.recovery_behaviors.empty() ? behaviors_ : goal.recovery_behaviors;
  current_recovery_behavior_ = recovery_behaviors_.begin();

  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!robot_info_->getRobotPose(robot_pose_))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("move_base"), "Could not get the current robot pose!");
    move_base_result->message = "Could not get the current robot pose!";
    move_base_result->outcome = mbf_msgs::action::MoveBase::Result::TF_ERROR;
    goal_handle->abort(move_base_result);
    return;
  }
  goal_pose_ = goal.target_pose;

  // wait for server connections
  if (!action_client_get_path_->wait_for_action_server(connection_timeout) ||
      !action_client_exe_path_->wait_for_action_server(connection_timeout) ||
      !action_client_recovery_->wait_for_action_server(connection_timeout))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("move_base"), "Could not connect to one or more of move_base_flex actions: "
        "\"get_path\", \"exe_path\", \"recovery \"!");
    move_base_result->outcome = mbf_msgs::action::MoveBase::Result::INTERNAL_ERROR;
    move_base_result->message = "Could not connect to the move_base_flex actions!";
    goal_handle->abort(move_base_result);
    return;
  }

  // call get_path action server to get a first plan
  // TODO what about goal response (accept/reject) and feedback callbacks?
  get_path_goal_handle_ = action_client_get_path_->async_send_goal(get_path_goal_, get_path_send_goal_options_);
}

void MoveBaseAction::actionExePathGoalResponse(const rclcpp_action::ClientGoalHandle<ExePath>::ConstSharedPtr& exe_path_goal_handle)
{
  if (exe_path_goal_handle) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The \"exe_path\" action goal (stamp " << exe_path_goal_handle->get_goal_stamp().seconds() << ") has been accepted.");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("move_base"), "The last action goal to \"exe_path\" has been rejected");
    action_state_ = FAILED;
  }
}

void MoveBaseAction::actionExePathFeedback(const rclcpp_action::ClientGoalHandle<ExePath>::ConstSharedPtr& goal_handle, const ExePath::Feedback::ConstSharedPtr &feedback)
//void MoveBaseAction::actionExePathFeedback(const mbf_msgs::action::ExePath::Feedback::ConstSharedPtr &feedback)
{
  mbf_msgs::action::MoveBase::Feedback::SharedPtr move_base_feedback;
  move_base_feedback->outcome = feedback->outcome;
  move_base_feedback->message = feedback->message;
  move_base_feedback->angle_to_goal = feedback->angle_to_goal;
  move_base_feedback->dist_to_goal = feedback->dist_to_goal;
  move_base_feedback->current_pose = feedback->current_pose;
  move_base_feedback->last_cmd_vel = feedback->last_cmd_vel;
  goal_handle_->publish_feedback(move_base_feedback);
  dist_to_goal_ = feedback->dist_to_goal;
  robot_pose_ = feedback->current_pose;

  // we create a navigation-level oscillation detection using exe_path action's feedback,
  // as the latter doesn't handle oscillations created by quickly failing repeated plans

  // if oscillation detection is enabled by oscillation_timeout != 0
  if (oscillation_timeout_ != rclcpp::Duration(0, 0))
  {
    // check if oscillating
    // moved more than the minimum oscillation distance
    const rclcpp::Time tNow = node_->now();
    if (mbf_utility::distance(robot_pose_, last_oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = tNow;
      last_oscillation_pose_ = robot_pose_;

      if (recovery_trigger_ == OSCILLATING)
      {
        RCLCPP_INFO(rclcpp::get_logger("move_base"), "Recovered from robot oscillation: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }
    }
    else if (last_oscillation_reset_ + oscillation_timeout_ < tNow)
    {
      std::stringstream oscillation_msgs;
      oscillation_msgs << "Robot is oscillating for " << (tNow - last_oscillation_reset_).seconds() << "s!";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), oscillation_msgs.str());
      action_client_exe_path_->async_cancel_all_goals();

      if (attemptRecovery())
      {
        recovery_trigger_ = OSCILLATING;
      }
      else
      {
        mbf_msgs::action::MoveBase::Result::SharedPtr move_base_result;
        move_base_result->outcome = mbf_msgs::action::MoveBase::Result::OSCILLATION;
        move_base_result->message = oscillation_msgs.str();
        move_base_result->final_pose = robot_pose_;
        move_base_result->angle_to_goal = move_base_feedback->angle_to_goal;
        move_base_result->dist_to_goal = move_base_feedback->dist_to_goal;
        goal_handle_->abort(move_base_result);
      }
    }
  }
}

void MoveBaseAction::actionGetPathGoalResponse(const rclcpp_action::ClientGoalHandle<GetPath>::ConstSharedPtr& get_path_goal_handle)
{
  if (get_path_goal_handle) 
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The \"get_path\" action goal (stamp " << get_path_goal_handle->get_goal_stamp().seconds() << ") has been accepted.");
  }
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("move_base"), "The last action goal to \"get_path\" has been rejected, cancelling move base goal.");
    mbf_msgs::action::MoveBase::Result::SharedPtr result = std::make_shared<mbf_msgs::action::MoveBase::Result>();
    result->message = "last action goal to get_path has been rejected";
    goal_handle_->canceled(result);
    action_state_ = FAILED;
  }
}

void MoveBaseAction::actionGetPathResult(const rclcpp_action::ClientGoalHandle<GetPath>::WrappedResult &result)
{
  const mbf_msgs::action::GetPath::Result &get_path_result = *(result.result);
  const mbf_msgs::action::MoveBase::Result::SharedPtr move_base_result = std::make_shared<mbf_msgs::action::MoveBase::Result>();

  // copy result from get_path action
  fillMoveBaseResult(get_path_result, *move_base_result);

  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Action \""
          << "move_base\" received a path from \""
          << "get_path\": " << get_path_result.message);

      exe_path_goal_.path = get_path_result.path;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Action \""
          << "move_base\" sends the path to \""
          << "exe_path\".");

      if (recovery_trigger_ == GET_PATH)
      {
        RCLCPP_WARN(rclcpp::get_logger("move_base"), "Recovered from planner failure: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }
      
      action_client_exe_path_->async_send_goal(exe_path_goal_, exe_path_send_goal_options_);
          
      action_state_ = EXE_PATH;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      //if (!action_client_exe_path_->async_get_result().getState().isDone()) // TODO how to check if not with ros2 actions? does it hurt to call cancel when exe_path is done? outputting the warning makes no sense
      //{
      RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Cancel previous goal, as planning to the new one has failed");
      cancel();
      //}
      if (attemptRecovery())
      {
        recovery_trigger_ = GET_PATH;
      }
      else
      {
        // copy result from get_path action
        RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Abort the execution of the planner: " << get_path_result.message);
        goal_handle_->abort(move_base_result);
      }
      action_state_ = FAILED;
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"get_path\" has been " << resultCodeToString(result.code));
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing get_path; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base preempted while executing get_path");
        goal_handle_->canceled(move_base_result);
      }
      break;

    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unknown action server state!");
      goal_handle_->abort(move_base_result);
      action_state_ = FAILED;
      break;
  }
}

void MoveBaseAction::actionExePathResult(const rclcpp_action::ClientGoalHandle<ExePath>::WrappedResult &result)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Action \"exe_path\" finished.");

  const mbf_msgs::action::ExePath::Result& exe_path_result = *(result.result);
  mbf_msgs::action::MoveBase::Result::SharedPtr move_base_result = std::make_shared<mbf_msgs::action::MoveBase::Result>();

  // copy result from exe_path action
  fillMoveBaseResult(exe_path_result, *move_base_result);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Current state: " << resultCodeToString(result.code));

  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      move_base_result->outcome = mbf_msgs::action::MoveBase::Result::SUCCESS;
      move_base_result->message = "Action \"move_base\" succeeded!";
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), move_base_result->message);
      goal_handle_->succeed(move_base_result);
      action_state_ = SUCCEEDED;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      action_state_ = FAILED;

      switch (exe_path_result.outcome)
      {
        case mbf_msgs::action::ExePath::Result::INVALID_PATH:
        case mbf_msgs::action::ExePath::Result::TF_ERROR:
        case mbf_msgs::action::ExePath::Result::NOT_INITIALIZED:
        case mbf_msgs::action::ExePath::Result::INVALID_PLUGIN:
        case mbf_msgs::action::ExePath::Result::INTERNAL_ERROR:
          // none of these errors is recoverable
          goal_handle_->abort(move_base_result);
          break;

        default:
          // all the rest are, so we start calling the recovery behaviors in sequence

          if (attemptRecovery())
          {
            recovery_trigger_ = EXE_PATH;
          }
          else
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Abort the execution of the controller: " << exe_path_result.message);
            goal_handle_->abort(move_base_result);
          }
          break;
      }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"exe_path\" has been " << resultCodeToString(result.code));
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing exe_path; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base preempted while executing exe_path");
        goal_handle_->canceled(move_base_result);
      }
      break;

    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unknown action result state!");
      goal_handle_->abort(move_base_result);
      action_state_ = FAILED;
      break;
  }
}

bool MoveBaseAction::attemptRecovery()
{
  if (!recovery_enabled_)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Recovery behaviors are disabled!");
    return false;
  }

  if (current_recovery_behavior_ == recovery_behaviors_.end())
  {
    if (recovery_behaviors_.empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "No Recovery Behaviors loaded!");
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Executed all available recovery behaviors!");
    }
    return false;
  }

  recovery_goal_.behavior = *current_recovery_behavior_;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Start recovery behavior\""
      << *current_recovery_behavior_ <<"\".");
  action_client_recovery_->async_send_goal(recovery_goal_, recovery_send_goal_options_);
  action_state_ = RECOVERY;
  return true;
}

void MoveBaseAction::actionRecoveryGoalResponse(const rclcpp_action::ClientGoalHandle<Recovery>::ConstSharedPtr& recovery_goal_handle)
{
  if (recovery_goal_handle) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The \"recovery\" action goal (stamp " << recovery_goal_handle->get_goal_stamp().seconds() << ") has been accepted.");
  } else {
    // recoveryRejectedOrAborted(); TODO gimme result?
  }
}

void MoveBaseAction::recoveryRejectedOrAborted(const rclcpp_action::ClientGoalHandle<Recovery>::WrappedResult &result) 
{
}

void MoveBaseAction::actionRecoveryResult(const rclcpp_action::ClientGoalHandle<Recovery>::WrappedResult &result)
{
  // give the robot some time to stop oscillating after executing the recovery behavior
  last_oscillation_reset_ = node_->now();

  const mbf_msgs::action::Recovery::Result& recovery_result = *(result.result);
  mbf_msgs::action::MoveBase::Result::SharedPtr move_base_result = std::make_shared<mbf_msgs::action::MoveBase::Result>();

  // copy result from recovery action
  fillMoveBaseResult(recovery_result, *move_base_result);

  switch (result.code)
  {
    case rclcpp_action::ResultCode::ABORTED:
      action_state_ = FAILED;

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The recovery behavior \""
          << *current_recovery_behavior_ << "\" has failed. ");
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Recovery behavior message: " << recovery_result.message
                                    << ", outcome: " << recovery_result.outcome);

      current_recovery_behavior_++; // use next behavior;
      if (current_recovery_behavior_ == recovery_behaviors_.end())
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                                "All recovery behaviors failed. Abort recovering and abort the move_base action");
        move_base_result->message = "All recovery behaviors failed."; 
        goal_handle_->abort(move_base_result);
      }
      else
      {
        recovery_goal_.behavior = *current_recovery_behavior_;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "Run the next recovery behavior \""
            << *current_recovery_behavior_ << "\".");
        action_client_recovery_->async_send_goal(recovery_goal_, recovery_send_goal_options_);
      }
      break;
    case rclcpp_action::ResultCode::SUCCEEDED:
      //go to planning state
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Execution of the recovery behavior \""
          << *current_recovery_behavior_ << "\" succeeded!");
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                             "Try planning again and increment the current recovery behavior in the list.");
      action_state_ = GET_PATH;
      current_recovery_behavior_++; // use next behavior, the next time;
      get_path_goal_handle_ = action_client_get_path_->async_send_goal(get_path_goal_, get_path_send_goal_options_);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"recovery\" has been canceled!");
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing a recovery; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base canceled while executing a recovery behavior");
        goal_handle_->canceled(move_base_result);
      }
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unreachable case! Unknown state!");
      goal_handle_->abort(move_base_result);
      action_state_ = FAILED;
      break;
  }
}

bool MoveBaseAction::replanningActive() const
{
  // replan only while following a path and if replanning is enabled (can be disabled by dynamic reconfigure)
  return replanning_period_.seconds() > 0.0 && action_state_ == EXE_PATH && dist_to_goal_ > 0.1;
}

void MoveBaseAction::replanningThread()
{
  const auto update_preiod = std::chrono::milliseconds(5);
  rclcpp::Time last_replan_time(0, 0, node_->get_clock()->get_clock_type());

  while (rclcpp::ok() && !replanning_thread_shutdown_ && get_path_goal_handle_.valid())
  {
    get_path_goal_handle_.wait(); // TODO maybe use wait_for and fail gracefully if the future does not return in time?
    const auto get_path_goal_handle = get_path_goal_handle_.get();
    if (get_path_goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED || get_path_goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
      const auto get_path_result_future = action_client_get_path_->async_get_result(get_path_goal_handle);
      if (get_path_result_future.wait_for(update_preiod) == std::future_status::ready)
      {
        const auto get_path_result = get_path_result_future.get();
        if (get_path_result.code == rclcpp_action::ResultCode::SUCCEEDED && replanningActive())
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Replanning succeeded; sending a goal to \"exe_path\" with the new plan");
          exe_path_goal_.path = get_path_result.result->path;
          mbf_msgs::action::ExePath::Goal exe_path_goal = exe_path_goal_;
          action_client_exe_path_->async_send_goal(exe_path_goal, exe_path_send_goal_options_);
        }
        else
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                                 "Replanning failed with error code " << get_path_result.result->outcome << ": " << get_path_result.result->message);
        }
      }
      // else keep waiting for planning to complete (we already waited update_period in waitForResult)
    }
    else if (!replanningActive())
    {
      rclcpp::sleep_for(update_preiod);
    }
    else if (node_->now() - last_replan_time >= replanning_period_)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Next replanning cycle, using the \"get_path\" action!");
      get_path_goal_handle_ = action_client_get_path_->async_send_goal(get_path_goal_); // TODO no callbacks needed?
      last_replan_time = node_->now();
    }
  }
}

} /* namespace mbf_abstract_nav */
