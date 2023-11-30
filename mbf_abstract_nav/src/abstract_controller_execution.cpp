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
 *  abstract_controller_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <mbf_msgs/action/exe_path.hpp>

#include "mbf_abstract_nav/abstract_controller_execution.h"

#include <tf2/utils.h>
#include <rclcpp/parameter_value.hpp>

namespace mbf_abstract_nav
{

const double AbstractControllerExecution::DEFAULT_CONTROLLER_FREQUENCY = 100.0; // 100 Hz

AbstractControllerExecution::AbstractControllerExecution(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr,
    const mbf_utility::RobotInformation& robot_info,
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& vel_pub,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& goal_pub,
    const rclcpp::Node::SharedPtr& node_handle)
  : AbstractExecutionBase(name, robot_info, node_handle)
  , controller_(controller_ptr)
  , state_(INITIALIZED)
  , moving_(false)
  , max_retries_(0)
  , patience_(0,0)
  , vel_pub_(vel_pub)
  , current_goal_pub_(goal_pub)
  , loop_rate_(std::make_shared<rclcpp::Rate>(DEFAULT_CONTROLLER_FREQUENCY))
  , node_handle_(node_handle)
{

  // reconfigurable parameters
  node_handle_->declare_parameter("robot_frame", "base_link");
  node_handle_->declare_parameter("map_frame", "map");
  node_handle_->declare_parameter("force_stop_at_goal", false);
  node_handle_->declare_parameter("force_stop_on_retry", true);
  node_handle_->declare_parameter("force_stop_on_cancel",false);
  node_handle_->declare_parameter("mbf_tolerance_check", false);
  node_handle_->declare_parameter("dist_tolerance", 0.1);
  node_handle_->declare_parameter("angle_tolerance", M_PI / 18.0);
  node_handle_->declare_parameter("tf_timeout", 1.0);
  node_handle_->declare_parameter("cmd_vel_ignored_tolerance", 5.0);

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "The rate in Hz at which to run the control loop and send velocity commands to the base";
  node_handle_->declare_parameter("controller_frequency", rclcpp::ParameterValue(20), param_desc);
  param_desc.description = "How long the controller will wait in seconds without receiving a valid control before "
                           "giving up";
  node_handle_->declare_parameter("controller_patience", rclcpp::ParameterValue(5.0), param_desc);
  param_desc.description ="How many times we will recall the controller in an attempt to find a valid command before giving up";
      node_handle_->declare_parameter("controller_max_retries", rclcpp::ParameterValue(-1), param_desc);

  node_handle_->get_parameter("robot_frame", robot_frame_);
  node_handle_->get_parameter("map_frame", global_frame_);
  node_handle_->get_parameter("force_stop_at_goal", force_stop_at_goal_);
  node_handle_->get_parameter("force_stop_on_retry", force_stop_on_retry_);
  node_handle_->get_parameter("force_stop_on_cancel", force_stop_on_cancel_);
  node_handle_->get_parameter("mbf_tolerance_check", mbf_tolerance_check_);
  node_handle_->get_parameter("dist_tolerance", dist_tolerance_);
  node_handle_->get_parameter("angle_tolerance", angle_tolerance_);
  node_handle_->get_parameter("tf_timeout", tf_timeout_);
  node_handle_->get_parameter("cmd_vel_ignored_tolerance", cmd_vel_ignored_tolerance_);

  double frequency;
  node_handle_->get_parameter("controller_frequency", frequency);
  setControllerFrequency(frequency);

  double patience;
  node_handle_->get_parameter("controller_patience", patience);
  patience_ = rclcpp::Duration::from_seconds(patience);
  node_handle_->get_parameter("controller_frequency", max_retries_);

  // dynamically reconfigurable parameters
  dyn_params_handler_ = node_handle_->add_on_set_parameters_callback(
      std::bind(&AbstractControllerExecution::reconfigure, this, std::placeholders::_1));
}

AbstractControllerExecution::~AbstractControllerExecution()
{
}

bool AbstractControllerExecution::setControllerFrequency(double frequency)
{
  // set the calling duration by the moving frequency
  if (frequency <= 0.0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AbstractControllerExecution"), "Controller frequency must be greater than 0.0! No "
                                                                    "change of the frequency!");
    return false;
  }
  loop_rate_ = std::make_shared<rclcpp::Rate>(frequency);
  return true;
}

rcl_interfaces::msg::SetParametersResult
AbstractControllerExecution::reconfigure(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(configuration_mutex_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const rclcpp::Parameter& param : parameters)
  {
    const auto& param_name = param.get_name();
    if (param_name == "controller_frequency")
    {
      setControllerFrequency(param.as_double());
    }
    else if (param_name == "controller_patience")
    {
      patience_ = rclcpp::Duration::from_seconds(param.as_double());
    }
    else if (param_name == "controller_max_retries")
    {
      max_retries_ = param.as_int();
    }
  }
  result.successful = true;
  return result;
}


bool AbstractControllerExecution::start()
{
  setState(STARTED);
  if (moving_)
  {
    return false; // thread is already running.
  }
  moving_ = true;
  return AbstractExecutionBase::start();
}


void AbstractControllerExecution::setState(ControllerState state)
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  state_ = state;
}

typename AbstractControllerExecution::ControllerState AbstractControllerExecution::getState() const
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  return state_;
}

void AbstractControllerExecution::setNewPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> &plan,
  bool tolerance_from_action,
  double action_dist_tolerance,
  double action_angle_tolerance)
{
  if (moving_)
  {
    // This is fine on continuous replanning
    RCLCPP_DEBUG(rclcpp::get_logger("AbstractControllerExecution"), "Setting new plan while moving");
  }
  std::lock_guard<std::mutex> guard(plan_mtx_);
  new_plan_ = true;

  plan_ = plan;
  tolerance_from_action_ = tolerance_from_action;
  action_dist_tolerance_ = action_dist_tolerance;
  action_angle_tolerance_ = action_angle_tolerance;
}


bool AbstractControllerExecution::hasNewPlan()
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  return new_plan_;
}


std::vector<geometry_msgs::msg::PoseStamped> AbstractControllerExecution::getNewPlan()
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  new_plan_ = false;
  return plan_;
}

uint32_t AbstractControllerExecution::computeVelocityCmd(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                         const geometry_msgs::msg::TwistStamped& robot_velocity,
                                                         geometry_msgs::msg::TwistStamped& vel_cmd,
                                                         std::string& message)
{
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

void AbstractControllerExecution::setVelocityCmd(const geometry_msgs::msg::TwistStamped& vel_cmd)
{
  std::lock_guard<std::mutex> guard(vel_cmd_mtx_);
  vel_cmd_stamped_ = vel_cmd;
  if (vel_cmd_stamped_.header.stamp.sec == 0 && vel_cmd_stamped_.header.stamp.nanosec == 0)
    vel_cmd_stamped_.header.stamp =  node_handle_->now();
  // TODO what happen with frame id?
  // TODO Add a queue here for handling the outcome, message and cmd_vel values bundled,
  // TODO so there should be no loss of information in the feedback stream
}

bool AbstractControllerExecution::checkCmdVelIgnored(const geometry_msgs::msg::Twist& cmd_vel)
{
  // check if the velocity ignored check is enabled or not
  if (cmd_vel_ignored_tolerance_ <= 0.0)
  { 
    return false;
  }

  const bool robot_stopped = robot_info_.isRobotStopped(1e-3, 1e-3);

  // compute linear and angular velocity magnitude
  const double cmd_linear = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
  const double cmd_angular = std::abs(cmd_vel.angular.z);

  const bool cmd_is_zero = cmd_linear < 1e-3 && cmd_angular < 1e-3;

  if (!robot_stopped || cmd_is_zero)
  {
    // velocity is not being ignored
    first_ignored_time_ =  node_handle_->now();
    return false;
  }

  if (first_ignored_time_.seconds() == 0.0)
  {
    // set first_ignored_time_ to now if it was zero
    first_ignored_time_ =  node_handle_->now();
  }

  const double ignored_duration = ( node_handle_->now() - first_ignored_time_).seconds();

  if (ignored_duration > cmd_vel_ignored_tolerance_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AbstractControllerExecution"),
                 "Robot is ignoring velocity commands for more than %.2f seconds. Tolerance exceeded!",
                 cmd_vel_ignored_tolerance_);
    return true;
  }
  else if (ignored_duration > 1.0)
  {
    auto clock = node_handle_->get_clock();
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("AbstractControllerExecution"), *clock, 1.0,
                         "Robot is ignoring velocity commands for %.2f seconds (last command: vx=%.2f, vy=%.2f, "
                         "w=%.2f)",
                         ignored_duration, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  }

  return false;
}

geometry_msgs::msg::TwistStamped AbstractControllerExecution::getVelocityCmd() const
{
  std::lock_guard<std::mutex> guard(vel_cmd_mtx_);
  return vel_cmd_stamped_;
}

rclcpp::Time AbstractControllerExecution::getLastPluginCallTime() const
{
  std::lock_guard<std::mutex> guard(lct_mtx_);
  return last_call_time_;
}

bool AbstractControllerExecution::isPatienceExceeded() const
{
  std::lock_guard<std::mutex> guard(lct_mtx_);
  if(!(patience_== rclcpp::Duration(0,0)) &&  node_handle_->now() - start_time_ > patience_) // not zero -> activated, start_time handles init case
  {
    if ( node_handle_->now() - last_call_time_ > patience_)
    {
      auto clock = node_handle_->get_clock();
      RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *clock, 1.0,
                           "The controller plugin \"%s\" needs more time to compute in one run than the patience time!",
                           name_.c_str());
      return true;
    }
    if ( node_handle_->now() - last_valid_cmd_time_ > patience_)
    {
      RCLCPP_DEBUG(node_handle_->get_logger(),
                   "The controller plugin \"%s\" does not return a success state (outcome < 10) for more than the "
                   "patience time in multiple runs!",
                   name_.c_str());
      return true;
    }
  }
  return false;
}

bool AbstractControllerExecution::isMoving() const
{
  return moving_;
}

bool AbstractControllerExecution::reachedGoalCheck()
{
  //if action has a specific tolerance, check goal reached with those tolerances
  if (tolerance_from_action_)
  {
    return controller_->isGoalReached(action_dist_tolerance_, action_angle_tolerance_) ||
        (mbf_tolerance_check_ && mbf_utility::distance(robot_pose_, plan_.back()) < action_dist_tolerance_
        && mbf_utility::angle(robot_pose_, plan_.back()) < action_angle_tolerance_);
  }

  // Otherwise, check whether the controller plugin returns goal reached or if mbf should check for goal reached.
  return controller_->isGoalReached(dist_tolerance_, angle_tolerance_) || (mbf_tolerance_check_
      && mbf_utility::distance(robot_pose_, plan_.back()) < dist_tolerance_
      && mbf_utility::angle(robot_pose_, plan_.back()) < angle_tolerance_);
}

bool AbstractControllerExecution::cancel()
{
  // Request the controller to cancel; it will return true if it takes care of stopping, returning CANCELED on
  // computeVelocityCmd when done. This allows for smooth, controlled stops.
  // If false (meaning cancel is not implemented, or that the controller defers handling it) MBF will take care.
  if (controller_->cancel())
  {
    RCLCPP_INFO(rclcpp::get_logger("AbstractControllerExecution"), "Controller will take care of stopping");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("AbstractControllerExecution"), "Controller defers handling cancel; force it and "
                                                                   "wait until the current control cycle finished");
    cancel_ = true;
    // wait for the control cycle to stop
    if (waitForStateUpdate(std::chrono::milliseconds(500)) == std::cv_status::timeout)
    {
      // this situation should never happen; if it does, the action server will be unready for goals immediately sent
      RCLCPP_WARN_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "Timeout while waiting for control cycle "
                                                                            "to stop; immediately sent goals can get "
                                                                            "stuck");
      return false;
    }
  }
  return true;
  }
  void AbstractControllerExecution::run()
  {
    start_time_ = node_handle_->now();

    // init plan
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    if (!hasNewPlan())
    {
      setState(NO_PLAN);
      moving_ = false;
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "robot navigation moving has no plan!");
    }

    last_valid_cmd_time_ =  node_handle_->now();
    int retries = 0;
    int seq = 0;
    first_ignored_time_ =  node_handle_->now();

    try
    {
      while (moving_ && rclcpp::ok())
      {
        if (cancel_)
        {
          if (force_stop_on_cancel_)
          {
            publishZeroVelocity();  // command the robot to stop on canceling navigation
          }
          setState(CANCELED);
          moving_ = false;
          condition_.notify_all();
          return;
        }

        if (!safetyCheck())
        {
          // the specific implementation must have detected a risk situation; at this abstract level, we
          // cannot tell what the problem is, but anyway we command the robot to stop to avoid crashes
          publishZeroVelocity();
          loop_rate_->sleep();
          continue;
        }

        // update plan dynamically
        if (hasNewPlan())
        {
          plan = getNewPlan();

          // check if plan is empty
          if (plan.empty())
          {
            setState(EMPTY_PLAN);
            moving_ = false;
            condition_.notify_all();
            return;
          }

          // check if plan could be set
          if (!controller_->setPlan(plan))
          {
            setState(INVALID_PLAN);
            moving_ = false;
            condition_.notify_all();
            return;
          }
          current_goal_pub_->publish(plan.back());
        }

        // compute robot pose and store it in robot_pose_
        if (!robot_info_.getRobotPose(robot_pose_))
        {
          message_ = "Could not get the robot pose";
          outcome_ = mbf_msgs::action::ExePath::Result::TF_ERROR;
          publishZeroVelocity();
          setState(INTERNAL_ERROR);
          moving_ = false;
          condition_.notify_all();
          return;
        }

        // ask planner if the goal is reached
        if (reachedGoalCheck())
        {
          RCLCPP_DEBUG(rclcpp::get_logger("abstract_controller_execution"), "Reached the goal!");
          if (force_stop_at_goal_)
          {
            publishZeroVelocity();
          }
          setState(ARRIVED_GOAL);
          // goal reached, tell it the controller
          moving_ = false;
          condition_.notify_all();
          // if not, keep moving
        }
        else
        {
          setState(PLANNING);

          // save time and call the plugin
          lct_mtx_.lock();
          last_call_time_ =  node_handle_->now();
          lct_mtx_.unlock();

          // call plugin to compute the next velocity command
          geometry_msgs::msg::TwistStamped cmd_vel_stamped;
          geometry_msgs::msg::TwistStamped robot_velocity;
          robot_info_.getRobotVelocity(robot_velocity);
          outcome_ = computeVelocityCmd(robot_pose_, robot_velocity, cmd_vel_stamped, message_ = "");

          if (outcome_ < 10)
          {
            setState(GOT_LOCAL_CMD);
            vel_pub_->publish(cmd_vel_stamped.twist);
            last_valid_cmd_time_ =  node_handle_->now();
            retries = 0;
            // check if robot is ignoring velocity command
            if (checkCmdVelIgnored(cmd_vel_stamped.twist))
            {
              setState(ROBOT_DISABLED);
              moving_ = false;
            }
          }
          else if (outcome_ == mbf_msgs::action::ExePath::Result::CANCELED)
          {
            RCLCPP_INFO(rclcpp::get_logger("AbstractControllerExecution"), "Controller-handled cancel completed");
            cancel_ = true;
            continue;
          }
          else
          {
            std::lock_guard<std::mutex> guard(configuration_mutex_);
            if (max_retries_ > 0 && ++retries > max_retries_)
            {
              setState(MAX_RETRIES);
              moving_ = false;
            }
            else if (isPatienceExceeded())
            {
              // patience limit enabled and running controller for more than patience without valid commands
              setState(PAT_EXCEEDED);
              moving_ = false;
            }
            else
            {
              setState(NO_LOCAL_CMD);  // useful for server feedback
              // keep trying if we have > 0 or -1 (infinite) retries
              moving_ = max_retries_;
            }

            // could not compute a valid velocity command
            if (!moving_ || force_stop_on_retry_)
            {
              publishZeroVelocity();  // command the robot to stop; we still feedback command calculated by the plugin
            }
            else
            {
              // we are retrying compute velocity commands; we keep sending the command calculated by the plugin
              // with the expectation that it's a sensible one (e.g. slow down while respecting acceleration limits)
              vel_pub_->publish(cmd_vel_stamped.twist);
            }
          }

          // set stamped values; timestamp and frame_id should be set by the plugin; otherwise setVelocityCmd will do
          //cmd_vel_stamped.header.seq = seq++;  // sequence number
          setVelocityCmd(cmd_vel_stamped);
          condition_.notify_all();
        }

        if (moving_)
        {
          // The nanosleep used by ROS time is not interruptable, therefore providing an interrupt point before and
          // after
          // The nanosleep used by ROS time is not interruptable, therefore providing an interrupt point before and
          // after
          // Simulate boost::this_thread::interruption_point()
          if (should_exit_)
          {
            // Early exit if shouldExit is set
            break;
          }
          if (!loop_rate_->sleep())
          {
            // TODO: find ROS2 equivalent or port for r.cycletime()
            // ROS_WARN_THROTTLE(1.0, "Calculation needs too much time to stay in the moving frequency! (%.4fs >
            // %.4fs)",
            //                   loop_rate_.cycleTime().toSec(), loop_rate_.expectedCycleTime().toSec());
          }
          // Simulate boost::this_thread::interruption_point()
          if (should_exit_)
          {
            // Early exit if shouldExit is set
            break;
          }
        }
      }
    }
    catch (const std::exception& ex)
    {
      // Controller thread interrupted; in most cases we have started a new plan
      // Can also be that robot is oscillating or we have exceeded planner patience
      RCLCPP_DEBUG(rclcpp::get_logger("AbstractControllerExecution"), "Controller thread interrupted!");
      publishZeroVelocity();
      setState(STOPPED);
      condition_.notify_all();
      moving_ = false;
    }
    catch (...)
    {
      message_ = "Unknown error occurred";
      RCLCPP_FATAL(node_handle_->get_logger(), "%s", message_.c_str());
      setState(INTERNAL_ERROR);
      moving_ = false;
      condition_.notify_all();
    }
}


void AbstractControllerExecution::publishZeroVelocity()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  vel_pub_->publish(cmd_vel);
}

} /* namespace mbf_abstract_nav */
