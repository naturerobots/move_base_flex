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
 *  planner_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <sstream>

#include "mbf_abstract_nav/planner_action.h"

namespace mbf_abstract_nav
{

PlannerAction::PlannerAction(
    const rclcpp::Node::SharedPtr& node,
    const std::string &name,
    const mbf_utility::RobotInformation::ConstPtr &robot_info)
  : AbstractActionBase(node, name, robot_info)
{
  // informative topics: current navigation goal
  current_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_goal", 1);
}

void PlannerAction::runImpl(GoalHandle &goal_handle, AbstractPlannerExecution &execution)
{
  const mbf_msgs::action::GetPath::Goal& goal = *(goal_handle.get_goal().get());

  mbf_msgs::action::GetPath::Result::SharedPtr result = std::make_shared<mbf_msgs::action::GetPath::Result>();
  geometry_msgs::msg::PoseStamped start_pose;

  result->path.header.frame_id = robot_info_->getGlobalFrame();

  double tolerance = goal.tolerance;
  bool use_start_pose = goal.use_start_pose;
  current_goal_pub_->publish(goal.target_pose);

  bool planner_active = true;

  if(use_start_pose)
  {
    start_pose = goal.start_pose;
    const geometry_msgs::msg::Point& p = start_pose.pose.position;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Use the given start pose (" << p.x << ", " << p.y << "), " << p.z << ").");
  }
  else
  {
    // get the current robot pose
    if (!robot_info_->getRobotPose(start_pose))
    {
      result->outcome = mbf_msgs::action::GetPath::Result::TF_ERROR;
      result->message = "Could not get the current robot pose!";
      goal_handle.abort(result);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result->message << " Canceling the action call.");
      return;
    }
    else
    {
      const geometry_msgs::msg::Point& p = start_pose.pose.position;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Got the current robot pose at ("
          << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  AbstractPlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::msg::PoseStamped> plan, global_plan;

  while (planner_active && rclcpp::ok())
  {
    // get the current state of the planning thread
    state_planning_input = execution.getState();

    switch (state_planning_input)
    {
      case AbstractPlannerExecution::INITIALIZED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: initialized");
        if (!execution.start(start_pose, goal.target_pose, tolerance))
        {
          result->outcome = mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
          result->message = "Another thread is still planning!";
          goal_handle.abort(result);
          RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result->message << " Canceling the action call.");
          planner_active = false;
        }
        break;

      case AbstractPlannerExecution::STARTED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: started");
        break;

      case AbstractPlannerExecution::STOPPED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: stopped");
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "Planning has been stopped rigorously!");
        result->outcome = mbf_msgs::action::GetPath::Result::STOPPED;
        result->message = "Global planner has been stopped!";
        goal_handle.abort(result);
        planner_active = false;
        break;

      case AbstractPlannerExecution::CANCELED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: canceled");
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Global planner has been canceled successfully");
        result->path.header.stamp = node_->now();
        result->outcome = mbf_msgs::action::GetPath::Result::CANCELED;
        result->message = "Global planner has been canceled!";
        goal_handle.canceled(result);
        planner_active = false;
        break;

        // in progress
      case AbstractPlannerExecution::PLANNING:
        if (execution.isPatienceExceeded())
        {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(name_), "Global planner patience has been exceeded! Cancel planning...");
          execution.cancel();
        }
        else
        {
          RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger(name_), *node_->get_clock(), 2000, "planner state: planning");
        }
        break;

        // found a new plan
      case AbstractPlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result->path.header.stamp = node_->now();
        plan = execution.getPlan();

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: found plan with cost: " << execution.getCost());

        if (!transformPlanToGlobalFrame(plan, global_plan))
        {
          result->outcome = mbf_msgs::action::GetPath::Result::TF_ERROR;
          result->message = "Could not transform the plan to the global frame!";

          RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result->message << " Canceling the action call.");
          goal_handle.abort(result);
          planner_active = false;
          break;
        }

        if (global_plan.empty())
        {
          result->outcome = mbf_msgs::action::GetPath::Result::EMPTY_PATH;
          result->message = "Global planner returned an empty path!";

          RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), result->message);
          goal_handle.abort(result);
          planner_active = false;
          break;
        }

        result->path.poses = global_plan;
        result->cost = execution.getCost();
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle.succeed(result);

        planner_active = false;
        break;

        // no plan found
      case AbstractPlannerExecution::NO_PLAN_FOUND:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "planner state: no plan found");
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle.abort(result);
        planner_active = false;
        break;

      case AbstractPlannerExecution::MAX_RETRIES:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Global planner reached the maximum number of retries");
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle.abort(result);
        planner_active = false;
        break;

      case AbstractPlannerExecution::PAT_EXCEEDED:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Global planner exceeded the patience time");
        result->outcome = mbf_msgs::action::GetPath::Result::PAT_EXCEEDED;
        result->message = "Global planner exceeded the patience time";
        goal_handle.abort(result);
        planner_active = false;
        break;

      case AbstractPlannerExecution::INTERNAL_ERROR:
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from planning
        planner_active = false;
        result->outcome = mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
        result->message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.abort(result);
        break;

      default:
        result->outcome = mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
        std::ostringstream ss;
        ss << "Internal error: Unknown state in a move base flex planner execution with the number: "
           << static_cast<int>(state_planning_input);
        result->message = ss.str();
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), result->message);
        goal_handle.abort(result);
        planner_active = false;
    }


    if (planner_active)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      execution.waitForStateUpdate(std::chrono::milliseconds(500));
    }
  }  // while (planner_active && ros::ok())

  if (!planner_active)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), "\"" << name_ << "\" action has been stopped!");
  }
}

bool PlannerAction::transformPlanToGlobalFrame(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                               std::vector<geometry_msgs::msg::PoseStamped>& global_plan)
{
  global_plan.clear();
  global_plan.reserve(plan.size());
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator iter;
  bool tf_success = false;
  for (iter = plan.begin(); iter != plan.end(); ++iter)
  {
    geometry_msgs::msg::PoseStamped global_pose;
    tf_success = mbf_utility::transformPose(node_, robot_info_->getTransformListener(), robot_info_->getGlobalFrame(),
                                            robot_info_->getTfTimeout(), *iter, global_pose);
    if (!tf_success)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(name_), "Can not transform pose from the \"" << iter->header.frame_id << "\" frame into the \""
                                                     << robot_info_->getGlobalFrame() << "\" frame !");
      return false;
    }
    global_plan.push_back(global_pose);
  }
  return true;
}

} /* namespace mbf_abstract_nav */
