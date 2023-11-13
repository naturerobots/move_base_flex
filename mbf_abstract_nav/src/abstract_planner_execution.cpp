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
 *  abstract_planner_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/abstract_planner_execution.h"

namespace mbf_abstract_nav
{

AbstractPlannerExecution::AbstractPlannerExecution(const std::string& name,
                                                   const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr,
                                                   const mbf_utility::RobotInformation& robot_info,
                                                   const rclcpp::Node::SharedPtr& node_handle)
  : AbstractExecutionBase(name, robot_info)
  , planner_(planner_ptr)
  , state_(INITIALIZED)
  , max_retries_(0)
  , planning_(false)
  , has_new_start_(false)
  , has_new_goal_(false)
  , node_handle_(node_handle)
  , patience_(0, 0)
{
  //reconfigurable parameters
  // private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
  // private_nh.param("map_frame", global_frame_, std::string("map"));
  node_handle_->declare_parameter("robot_frame", std::string("base_footprint"));
  node_handle_->declare_parameter("map_frame", std::string("map"));


  node_handle_->declare_parameter("planner_frequency", 0.0);
  node_handle_->declare_parameter("planner_patience", 5.0);
  node_handle_->declare_parameter("planner_max_retries", -1);
  
  // dynamically reconfigurable parameters
  dyn_params_handler_ = node_handle_->add_on_set_parameters_callback(
      std::bind(&AbstractPlannerExecution::reconfigure, this, std::placeholders::_1));
}

AbstractPlannerExecution::~AbstractPlannerExecution()
{
}

template <typename _Iter>
double sumDistance(_Iter _begin, _Iter _end)
{
  // helper function to get the distance of a path.
  // in C++11, we could add static_assert on the interator_type.
  double dist = 0.;

  // minimum length of the path is 2.
  if (std::distance(_begin, _end) < 2)
    return dist;

  // two pointer iteration
  for (_Iter next = _begin + 1; next != _end; ++_begin, ++next)
    dist += mbf_utility::distance(*_begin, *next);

  return dist;
}

double AbstractPlannerExecution::getCost() const
{
  return cost_;
}

rcl_interfaces::msg::SetParametersResult
AbstractPlannerExecution::reconfigure(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(configuration_mutex_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const rclcpp::Parameter& param : parameters)
  {
    const auto& param_name = param.get_name();
    if (param_name == "planner_frequency")
    {
      frequency_= param.as_double();
    }
    else if (param_name == "planner_patience")
    {
      try
      {
          patience_ = rclcpp::Duration::from_seconds(param.as_double());
      }
      catch (std::exception& ex)
      {
        RCLCPP_ERROR(rclcpp::get_logger("AbstractPlannerExecution"), "Failed to set planner_patience: %s",
                     ex.what());
        patience_ = rclcpp::Duration(0,0);
      }
    }
    else if (param_name == "planner_max_retries")
    {
      max_retries_ = param.as_int();
    }
  }
  result.successful = true;
  return result;
}


typename AbstractPlannerExecution::PlanningState AbstractPlannerExecution::getState() const
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  return state_;
}

void AbstractPlannerExecution::setState(PlanningState state, bool signalling)
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  state_ = state;

  // we exit planning if we are signalling.
  planning_ = !signalling;

  // some states are quiet, most aren't
  if(signalling)
    condition_.notify_all();
}


rclcpp::Time AbstractPlannerExecution::getLastValidPlanTime() const
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  return last_valid_plan_time_;
}


bool AbstractPlannerExecution::isPatienceExceeded() const
{
  return !(patience_ == rclcpp::Duration(0, 0)) && (node_handle_->now() - last_call_start_time_ > patience_);
}


std::vector<geometry_msgs::msg::PoseStamped> AbstractPlannerExecution::getPlan() const
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  // copy plan and costs to output
  return plan_;
}


void AbstractPlannerExecution::setNewGoal(const geometry_msgs::msg::PoseStamped &goal, double tolerance)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  goal_ = goal;
  tolerance_ = tolerance;
  has_new_goal_ = true;
}


void AbstractPlannerExecution::setNewStart(const geometry_msgs::msg::PoseStamped &start)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  start_ = start;
  has_new_start_ = true;
}

void AbstractPlannerExecution::setNewStartAndGoal(const geometry_msgs::msg::PoseStamped& start,
                                                  const geometry_msgs::msg::PoseStamped& goal,
                                                  double tolerance)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  start_ = start;
  goal_ = goal;
  tolerance_ = tolerance;
  has_new_start_ = true;
  has_new_goal_ = true;
}

bool AbstractPlannerExecution::start(const geometry_msgs::msg::PoseStamped& start,
                                     const geometry_msgs::msg::PoseStamped& goal, double tolerance)
{
  if (planning_)
  {
    return false;
  }
  std::lock_guard<std::mutex> guard(planning_mtx_);
  planning_ = true;
  start_ = start;
  goal_ = goal;
  tolerance_ = tolerance;

  const geometry_msgs::msg::Point& s = start.pose.position;
  const geometry_msgs::msg::Point& g = goal.pose.position;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AbstractControllerExecution") ,"Start planning from the start pose: ("
                   << s.x << ", " << s.y << ", " << s.z << ")"
                   << " to the goal pose: (" << g.x << ", " << g.y << ", " << g.z << ")");

  return AbstractExecutionBase::start();
}


bool AbstractPlannerExecution::cancel()
{
  cancel_ = true; // force cancel immediately, as the call to cancel in the planner can take a while

  // returns false if cancel is not implemented or rejected by the planner (will run until completion)
  if (!planner_->cancel())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                        "Cancel planning failed or is not supported by the plugin. "
                            << "Wait until the current planning finished!");

    return false;
  }
  return true;
}

uint32_t AbstractPlannerExecution::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                            const geometry_msgs::msg::PoseStamped& goal, 
                                            double tolerance,
                                            std::vector<geometry_msgs::msg::PoseStamped>& plan, 
                                            double& cost,
                                            std::string& message)
{
  return planner_->makePlan(start, goal, tolerance, plan, cost, message);
}

void AbstractPlannerExecution::run()
{
  setState(STARTED, false);
  std::lock_guard<std::mutex> guard(planning_mtx_);
  int retries = 0;
  geometry_msgs::msg::PoseStamped current_start = start_;
  geometry_msgs::msg::PoseStamped current_goal = goal_;
  double current_tolerance = tolerance_;

  last_call_start_time_ = node_handle_->now();
  last_valid_plan_time_ = node_handle_->now();

  try
  {
    while (planning_ && rclcpp::ok())
    {
      // call the planner
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      double cost = 0.0;

      // lock goal start mutex
      goal_start_mtx_.lock();
      if (has_new_start_)
      {
        has_new_start_ = false;
        current_start = start_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "A new start pose is available. Planning "
                                                                           "with the new start pose!");
        const geometry_msgs::msg::Point& s = start_.pose.position;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                           "New planning start pose: (" << s.x << ", " << s.y << ", " << s.z << ")");
      }
      if (has_new_goal_)
      {
        has_new_goal_ = false;
        current_goal = goal_;
        current_tolerance = tolerance_;
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("AbstractControllerExecution"),
            "A new goal pose is available. Planning with the new goal pose and the tolerance: " << current_tolerance);
        const geometry_msgs::msg::Point& g = goal_.pose.position;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                           "New goal pose: (" << g.x << ", " << g.y << ", " << g.z << ")");
      }

      // unlock goal
      goal_start_mtx_.unlock();
      if (cancel_)
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "The global planner has been canceled!");
        setState(CANCELED, true);
      }
      else
      {
        setState(PLANNING, false);

        outcome_ = makePlan(current_start, current_goal, current_tolerance, plan, cost, message_);
        bool success = outcome_ < 10;

        std::lock_guard<std::mutex> guard(configuration_mutex_);

        if (cancel_ && !isPatienceExceeded())
        {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                             "The planner \"" << name_ << "\" has been canceled!");  // but not due to patience exceeded
          setState(CANCELED, true);
        }
        else if (success)
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "Successfully found a plan.");

          std::lock_guard<std::mutex> plan_mtx_guard(plan_mtx_);
          plan_ = plan;
          cost_ = cost;
          // estimate the cost based on the distance if its zero.
          if (cost_ == 0)
            cost_ = sumDistance(plan_.begin(), plan_.end());

          last_valid_plan_time_ = node_handle_->now();
          setState(FOUND_PLAN, true);
        }
        else if (max_retries_ > 0 && ++retries > max_retries_)
        {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                             "Planning reached max retries! (" << max_retries_ << ")");
          setState(MAX_RETRIES, true);
        }
        else if (isPatienceExceeded())
        {
          // Patience exceeded is handled at two levels: here to stop retrying planning when max_retries is
          // disabled, and on the navigation server when the planner doesn't return for more that patience seconds.
          // In the second case, the navigation server has tried to cancel planning (possibly without success, as
          // old nav_core-based planners do not support canceling), and we add here the fact to the log for info
          RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),
                             "Planning patience (" << patience_.seconds() << "s) has been exceeded"
                                                   << (cancel_ ? "; planner canceled!" : ""));
          setState(PAT_EXCEEDED, true);
        }
        else if (max_retries_ == 0)
        {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("AbstractControllerExecution"),"Planning could not find a plan!");
          setState(NO_PLAN_FOUND, true);
        }
        else
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "Planning could not find a plan! "
                                                                                "Trying again...");
        }
      }
    } // while (planning_ && ros::ok())
  }
  catch (const std::exception &ex)
  {
    // Planner thread interrupted; probably we have exceeded planner patience
    RCLCPP_WARN_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "Planner thread interrupted!");
    setState(STOPPED, true);
  }
  catch (...)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("AbstractControllerExecution"), "Unknown error occurred.");
    setState(INTERNAL_ERROR, true);
  }
}

} /* namespace mbf_abstract_nav */

