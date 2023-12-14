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
 *  abstract_planner_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_

#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mbf_abstract_core/abstract_planner.h>

#include <mbf_utility/robot_information.h>
#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{

/**
 * @defgroup planner_execution Planner Execution Classes
 * @brief The planner execution classes are derived from the AbstractPlannerExecution and extend the functionality.
 *        The base planner execution code is located in the AbstractPlannerExecution.
 */

/**
 * @brief The AbstractPlannerExecution class loads and binds the global planner plugin. It contains a thread running
 *        the plugin in a cycle to plan and re-plan. An internal state is saved and will be pulled by the server, which
 *        controls the global planner execution. Due to a state change it wakes up all threads connected to the
 *        condition variable.
 *
 * @ingroup abstract_server planner_execution
 */
  class AbstractPlannerExecution : public AbstractExecutionBase
  {
  public:

    //! shared pointer type to the @ref planner_execution "planner execution".
    typedef std::shared_ptr<AbstractPlannerExecution > Ptr;

    /**
     * @brief Constructor
     * @param name Name of this execution
     * @param planner_ptr Pointer to the planner
     * @param robot_info Current robot state
     * @param config Initial configuration for this execution
     */
    AbstractPlannerExecution(const std::string& name, const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr,
                             const mbf_utility::RobotInformation::ConstPtr& robot_info,
                             const rclcpp::Node::SharedPtr& node_handle);

    /**
     * @brief Destructor
     */
    virtual ~AbstractPlannerExecution();
    
    /**
     * @brief Returns a new plan, if one is available.
     */
    std::vector<geometry_msgs::msg::PoseStamped> getPlan() const;

    /**
     * @brief Returns the last time a valid plan was available.
     * @return time, the last valid plan was available.
     */
    rclcpp::Time getLastValidPlanTime() const;

    /**
     * @brief Checks whether the patience was exceeded.
     * @return true, if the patience duration was exceeded.
     */
    bool isPatienceExceeded() const;

    /**
     * @brief Internal states
     */
    enum PlanningState
    {
      INITIALIZED,  ///< Planner initialized.
      STARTED,      ///< Planner started.
      PLANNING,     ///< Executing the plugin.
      FOUND_PLAN,   ///< Found a valid plan.
      MAX_RETRIES,  ///< Exceeded the maximum number of retries without a valid command.
      PAT_EXCEEDED, ///< Exceeded the patience time without a valid command.
      NO_PLAN_FOUND,///< No plan has been found (MAX_RETRIES and PAT_EXCEEDED are 0).
      CANCELED,     ///< The planner has been canceled.
      STOPPED,      ///< The planner has been stopped.
      INTERNAL_ERROR///< An internal error occurred.
    };

    /**
     * @brief Returns the current internal state
     * @return the current internal state
     */
    PlanningState getState() const;

    /**
     * @brief Gets planning frequency
     */
    double getFrequency() const { return frequency_; };

    /**
     * @brief Gets computed costs
     * @return The costs of the computed path
     */
    double getCost() const;

    /**
     * @brief Cancel the planner execution. This calls the cancel method of the planner plugin.
     * This could be useful if the computation takes too much time, or if we are aborting the navigation.
     * @return true, if the planner plugin tries / tried to cancel the planning step.
     */
    virtual bool cancel();

    /**
     * @brief Sets a new goal pose for the planner execution
     * @param goal the new goal pose
     * @param tolerance tolerance to the goal for the planning
     */
    void setNewGoal(const geometry_msgs::msg::PoseStamped &goal, double tolerance);

    /**
     * @brief Sets a new start pose for the planner execution
     * @param start new start pose
     */
    void setNewStart(const geometry_msgs::msg::PoseStamped &start);

    /**
     * @brief Sets a new star and goal pose for the planner execution
     * @param start new start pose
     * @param goal new goal pose
     * @param tolerance tolerance to the new goal for the planning
     */
    void setNewStartAndGoal(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance);

    /**
     * @brief Starts the planner execution thread with the given parameters.
     * @param start start pose for the planning
     * @param goal goal pose for the planning
     * @param tolerance tolerance to the goal pose for the planning
     * @return true, if the planner thread has been started, false if the thread is already running.
     */
    bool start(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
               double tolerance);

    /**
     * @brief Is called by the server thread to reconfigure the controller execution, if a user uses dynamic reconfigure
     *        to reconfigure the current state.
     * @param parameters Vector parameters that are being reconfigured.
     */
    rcl_interfaces::msg::SetParametersResult reconfigure(std::vector<rclcpp::Parameter> parameters);

  protected:

    //! the local planer to calculate the robot trajectory
    mbf_abstract_core::AbstractPlanner::Ptr planner_;

    //! the name of the loaded planner plugin
    std::string plugin_name_;

    /**
     * @brief The main run method, a thread will execute this method. It contains the main planner execution loop.
     */
    virtual void run();

    //! Helper method for cleaning up the state when the planner thread was interrupted.
    void handle_thread_interrupted();

  private:

    /**
     * @brief calls the planner plugin to make a plan from the start pose to the goal pose with the given tolerance,
     *        if a goal tolerance is enabled in the planner plugin.
     * @param start The start pose for planning
     * @param goal The goal pose for planning
     * @param tolerance The goal tolerance
     * @param plan The computed plan by the plugin
     * @param cost The computed costs for the corresponding plan
     * @param message An optional message which should correspond with the returned outcome
     * @return An outcome number, see also the action definition in the GetPath.action file
     */
    virtual uint32_t makePlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        double tolerance,
        std::vector<geometry_msgs::msg::PoseStamped> &plan,
        double &cost,
        std::string &message);

    /**
     * @brief Sets the internal state, thread communication safe
     * @param state the current state
     * @param signalling set true to trigger the condition-variable for state-update
     */
    void setState(PlanningState state, bool signalling);


    //! mutex to handle safe thread communication for the current state
    mutable std::mutex state_mtx_;

    //! mutex to handle safe thread communication for the plan and plan-costs
    mutable std::mutex plan_mtx_;

    //! mutex to handle safe thread communication for the goal and start pose.
    mutable std::mutex goal_start_mtx_;

    //! mutex to handle safe thread communication for the planning_ flag.
    mutable std::mutex planning_mtx_;

    //! dynamic reconfigure mutex for a thread safe communication
    mutable std::mutex configuration_mutex_;

    //! true, if a new goal pose has been set, until it is used.
    bool has_new_goal_;

    //! true, if a new start pose has been set, until it is used.
    bool has_new_start_;

    //! the last call start time, updated each cycle.
    rclcpp::Time last_call_start_time_;

    //! the last time a valid plan has been computed.
    rclcpp::Time last_valid_plan_time_;

    //! current global plan
    std::vector<geometry_msgs::msg::PoseStamped> plan_;

    //! current global plan cost
    double cost_;

    //! the current start pose used for planning
    geometry_msgs::msg::PoseStamped start_;
    

    //! the current goal pose used for planning
    geometry_msgs::msg::PoseStamped goal_;

    //! optional goal tolerance, in meters
    double tolerance_;

    //! planning cycle frequency (used only when running full navigation; we store here for grouping parameters nicely)
    double frequency_;

    //! planning patience duration time
    rclcpp::Duration patience_;

    //! planning max retries
    int max_retries_;

    //! main cycle variable of the execution loop
    bool planning_;

    //! robot frame used for computing the current robot pose
    std::string robot_frame_;

    //! the global frame in which the planner needs to plan
    std::string global_frame_;

    //! current internal state
    PlanningState state_;

    //! the node handle ptr
    rclcpp::Node::SharedPtr node_handle_;

    // dynamic configuration
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  };

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_ */
