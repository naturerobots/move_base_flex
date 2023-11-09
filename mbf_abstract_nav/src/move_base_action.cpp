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

using namespace std::placeholders;

MoveBaseAction::MoveBaseAction(const std::string& name, const mbf_utility::RobotInformation& robot_info,
                               const std::vector<std::string>& behaviors, const rclcpp::Node::WeakPtr &node)
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
  , replanning_thread_(boost::bind(&MoveBaseAction::replanningThread, this))
{
  auto nodeActive = node_.lock();
  action_client_exe_path_ = rclcpp_action::create_client<ExePath>(nodeActive, "exe_path");
  action_client_get_path_ = rclcpp_action::create_client<GetPath>(nodeActive, "get_path");
  action_client_recovery_ = rclcpp_action::create_client<Recovery>(nodeActive, "recovery");
  dyn_params_handler_ = nodeActive->add_on_set_parameters_callback(std::bind(&MoveBaseAction::reconfigure, this, _1));
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

  if (!action_client_get_path_.getState().isDone())
  {
    action_client_get_path_.cancelGoal();
  }

  if (!action_client_exe_path_.getState().isDone())
  {
    action_client_exe_path_.cancelGoal();
  }

  if (!action_client_recovery_.getState().isDone())
  {
    action_client_recovery_.cancelGoal();
  }
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

  last_oscillation_reset_ = node_.lock()->now();

  // start recovering with the first behavior, use the recovery behaviors from the action request, if specified,
  // otherwise, use all loaded behaviors.

  recovery_behaviors_ = goal.recovery_behaviors.empty() ? behaviors_ : goal.recovery_behaviors;
  current_recovery_behavior_ = recovery_behaviors_.begin();

  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!robot_info_.getRobotPose(robot_pose_))
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
  auto get_path_send_goal_options = rclcpp_action::Client<GetPath>::SendGoalOptions();
  get_path_send_goal_options.result_callback = std::bind(&MoveBaseAction::actionGetPathDone, this, _1, _2);
  // TODO what about goal response (accept/reject) and feedback callbacks?
  action_client_get_path_->async_send_goal(get_path_goal_, get_path_send_goal_options);
}

void MoveBaseAction::actionExePathActive()
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The \"exe_path\" action is active.");
}

void MoveBaseAction::actionExePathFeedback(const mbf_msgs::action::ExePath::Feedback::ConstSharedPtr &feedback)
{
  mbf_msgs::action::MoveBase::Feedback move_base_feedback;
  move_base_feedback.outcome = feedback->outcome;
  move_base_feedback.message = feedback->message;
  move_base_feedback.angle_to_goal = feedback->angle_to_goal;
  move_base_feedback.dist_to_goal = feedback->dist_to_goal;
  move_base_feedback.current_pose = feedback->current_pose;
  move_base_feedback.last_cmd_vel = feedback->last_cmd_vel;
  goal_handle_.publishFeedback(move_base_feedback); // TODO update action
  dist_to_goal_ = feedback->dist_to_goal;
  robot_pose_ = feedback->current_pose;

  // we create a navigation-level oscillation detection using exe_path action's feedback,
  // as the latter doesn't handle oscillations created by quickly failing repeated plans

  // if oscillation detection is enabled by oscillation_timeout != 0
  if (!oscillation_timeout_.isZero())
  {
    // check if oscillating
    // moved more than the minimum oscillation distance
    if (mbf_utility::distance(robot_pose_, last_oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = node_->now();
      last_oscillation_pose_ = robot_pose_;

      if (recovery_trigger_ == OSCILLATING)
      {
        RCLCPP_INFO(rclcpp::get_logger("move_base"), "Recovered from robot oscillation: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }
    }
    else if (last_oscillation_reset_ + oscillation_timeout_ < node_->now())
    {
      std::stringstream oscillation_msgs;
      oscillation_msgs << "Robot is oscillating for " << (node_->now() - last_oscillation_reset_).toSec() << "s!";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), oscillation_msgs.str());
      action_client_exe_path_.cancelGoal();

      if (attemptRecovery())
      {
        recovery_trigger_ = OSCILLATING;
      }
      else
      {
        mbf_msgs::action::MoveBase::Result move_base_result;
        move_base_result.outcome = mbf_msgs::action::MoveBase::Result::OSCILLATION;
        move_base_result.message = oscillation_msgs.str();
        move_base_result.final_pose = robot_pose_;
        move_base_result.angle_to_goal = move_base_feedback.angle_to_goal;
        move_base_result.dist_to_goal = move_base_feedback.dist_to_goal;
        goal_handle_.setAborted(move_base_result, move_base_result.message);
      }
    }
  }
}

void MoveBaseAction::actionGetPathDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::action::GetPath::Result::ConstSharedPtr &result_ptr)
{
  const mbf_msgs::action::GetPath::Result &get_path_result = *result_ptr;
  mbf_msgs::action::MoveBase::Result move_base_result;

  // copy result from get_path action
  fillMoveBaseResult(get_path_result, move_base_result);

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::PENDING:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "get_path PENDING state not implemented, this should not be reachable!");
      break;

    case actionlib::SimpleClientGoalState::SUCCEEDED:
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Action \""
          << "move_base\" received a path from \""
          << "get_path\": " << state.getText());

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

      action_client_exe_path_.sendGoal(
          exe_path_goal_,
          boost::bind(&MoveBaseAction::actionExePathDone, this, _1, _2),
          boost::bind(&MoveBaseAction::actionExePathActive, this),
          boost::bind(&MoveBaseAction::actionExePathFeedback, this, _1));
      action_state_ = EXE_PATH;
      break;

    case actionlib::SimpleClientGoalState::ABORTED:
      if (!action_client_exe_path_.getState().isDone())
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Cancel previous goal, as planning to the new one has failed");
        cancel();
      }
      if (attemptRecovery())
      {
        recovery_trigger_ = GET_PATH;
      }
      else
      {
        // copy result from get_path action
        RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"), "Abort the execution of the planner: " << get_path_result.message);
        goal_handle_.setAborted(move_base_result, state.getText());
      }
      action_state_ = FAILED;
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::PREEMPTED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"get_path\" has been " << state.toString());
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing get_path; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base preempted while executing get_path");
        goal_handle_.setCanceled(move_base_result, state.getText());
      }
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"get_path\" has been " << state.toString());
      goal_handle_.setCanceled(move_base_result, state.getText());
      action_state_ = FAILED;
      break;

    case actionlib::SimpleClientGoalState::LOST:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Connection lost to the action \"get_path\"!");
      goal_handle_.setAborted();
      action_state_ = FAILED;
      break;

    default:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unknown action server state!");
      goal_handle_.setAborted();
      action_state_ = FAILED;
      break;
  }
}

void MoveBaseAction::actionExePathDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::action::ExePath::Result::ConstSharedPtr &result_ptr)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Action \"exe_path\" finished.");

  const mbf_msgs::action::ExePath::Result& exe_path_result = *result_ptr;
  mbf_msgs::action::MoveBase::Result move_base_result;

  // copy result from exe_path action
  fillMoveBaseResult(exe_path_result, move_base_result);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Current state: " << state.toString());

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      move_base_result.outcome = mbf_msgs::action::MoveBase::Result::SUCCESS;
      move_base_result.message = "Action \"move_base\" succeeded!";
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), move_base_result.message);
      goal_handle_.setSucceeded(move_base_result, move_base_result.message);
      action_state_ = SUCCEEDED;
      break;

    case actionlib::SimpleClientGoalState::ABORTED:
      action_state_ = FAILED;

      switch (exe_path_result.outcome)
      {
        case mbf_msgs::action::ExePath::Result::INVALID_PATH:
        case mbf_msgs::action::ExePath::Result::TF_ERROR:
        case mbf_msgs::action::ExePath::Result::NOT_INITIALIZED:
        case mbf_msgs::action::ExePath::Result::INVALID_PLUGIN:
        case mbf_msgs::action::ExePath::Result::INTERNAL_ERROR:
          // none of these errors is recoverable
          goal_handle_.setAborted(move_base_result, state.getText());
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
            goal_handle_.setAborted(move_base_result, state.getText());
          }
          break;
      }
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::PREEMPTED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"exe_path\" has been " << state.toString());
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing exe_path; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base preempted while executing exe_path");
        goal_handle_.setCanceled(move_base_result, state.getText());
      }
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"exe_path\" has been " << state.toString());
      goal_handle_.setCanceled(move_base_result, state.getText());
      action_state_ = FAILED;
      break;

    case actionlib::SimpleClientGoalState::LOST:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Connection lost to the action \"exe_path\"!");
      goal_handle_.setAborted();
      action_state_ = FAILED;
      break;

    default:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unreachable case! Unknown SimpleActionServer state!");
      goal_handle_.setAborted();
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
  action_client_recovery_.sendGoal(
      recovery_goal_,
      boost::bind(&MoveBaseAction::actionRecoveryDone, this, _1, _2)
  );
  action_state_ = RECOVERY;
  return true;
}

void MoveBaseAction::actionRecoveryDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::action::Recovery::Result::ConstSharedPtr &result_ptr)
{
  // give the robot some time to stop oscillating after executing the recovery behavior
  last_oscillation_reset_ = node_->now();

  const mbf_msgs::action::Recovery::Result& recovery_result = *result_ptr;
  mbf_msgs::action::MoveBase::Result move_base_result;

  // copy result from recovery action
  fillMoveBaseResult(recovery_result, move_base_result);

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::REJECTED:
    case actionlib::SimpleClientGoalState::ABORTED:
      action_state_ = FAILED;

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "The recovery behavior \""
          << *current_recovery_behavior_ << "\" has failed. ");
      RCLCPP_DEBUG_STREAM("Recovery behavior message: " << recovery_result.message
                                    << ", outcome: " << recovery_result.outcome);

      current_recovery_behavior_++; // use next behavior;
      if (current_recovery_behavior_ == recovery_behaviors_.end())
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                               "All recovery behaviors failed. Abort recovering and abort the move_base action");
        goal_handle_.setAborted(move_base_result, "All recovery behaviors failed.");
      }
      else
      {
        recovery_goal_.behavior = *current_recovery_behavior_;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "Run the next recovery behavior \""
            << *current_recovery_behavior_ << "\".");
        action_client_recovery_.sendGoal(
            recovery_goal_,
            boost::bind(&MoveBaseAction::actionRecoveryDone, this, _1, _2)
        );
      }
      break;
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      //go to planning state
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Execution of the recovery behavior \""
          << *current_recovery_behavior_ << "\" succeeded!");
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                             "Try planning again and increment the current recovery behavior in the list.");
      action_state_ = GET_PATH;
      current_recovery_behavior_++; // use next behavior, the next time;
      action_client_get_path_.sendGoal(
          get_path_goal_,
          boost::bind(&MoveBaseAction::actionGetPathDone, this, _1, _2)
      );
      break;
    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::PREEMPTED:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "The last action goal to \"recovery\" has been preempted!");
      if (action_state_ == CANCELED)
      {
        // move_base preempted while executing a recovery; fill result and report canceled to the client
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_base"), "move_base preempted while executing a recovery behavior");
        goal_handle_.setCanceled(move_base_result, state.getText());
      }
      break;

    case actionlib::SimpleClientGoalState::LOST:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Connection lost to the action \"recovery\"!");
      goal_handle_.setAborted();
      action_state_ = FAILED;
      break;
    default:
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("move_base"), "Reached unreachable case! Unknown state!");
      goal_handle_.setAborted();
      action_state_ = FAILED;
      break;
  }
}

bool MoveBaseAction::replanningActive() const
{
  // replan only while following a path and if replanning is enabled (can be disabled by dynamic reconfigure)
  return !replanning_period_.isZero() && action_state_ == EXE_PATH && dist_to_goal_ > 0.1;
}

void MoveBaseAction::replanningThread()
{
  rclcpp::Duration update_period = rclcpp::Duration::from_seconds(0.005);
  rclcpp::Time last_replan_time(0, 0, node_->get_clock()->get_clock_type());

  while (rclcpp::ok() && !replanning_thread_shutdown_)
  {
    if (!action_client_get_path_.getState().isDone())
    {
      if (action_client_get_path_.waitForResult(update_period))
      {
        actionlib::SimpleClientGoalState state = action_client_get_path_.getState();
        mbf_msgs::action::GetPath::Result::ConstSharedPtr result = action_client_get_path_.getResult();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED && replanningActive())
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Replanning succeeded; sending a goal to \"exe_path\" with the new plan");
          exe_path_goal_.path = result->path;
          mbf_msgs::action::ExePath::Goal goal(exe_path_goal_);
          action_client_exe_path_.sendGoal(goal, boost::bind(&MoveBaseAction::actionExePathDone, this, _1, _2),
                                           boost::bind(&MoveBaseAction::actionExePathActive, this),
                                           boost::bind(&MoveBaseAction::actionExePathFeedback, this, _1));
        }
        else
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"),
                                 "Replanning failed with error code " << result->outcome << ": " << result->message);
        }
      }
      // else keep waiting for planning to complete (we already waited update_period in waitForResult)
    }
    else if (!replanningActive())
    {
      update_period.sleep();
    }
    else if (node_->now() - last_replan_time >= replanning_period_)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("move_base"), "Next replanning cycle, using the \"get_path\" action!");
      action_client_get_path_.sendGoal(get_path_goal_);
      last_replan_time = node_->now();
    }
  }
}

} /* namespace mbf_abstract_nav */
