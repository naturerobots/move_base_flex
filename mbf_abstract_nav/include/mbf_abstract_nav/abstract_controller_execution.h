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
 *  abstract_controller_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include <rclcpp/rate.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <mbf_utility/navigation_utility.h>
#include <mbf_abstract_core/abstract_controller.h>

#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{

/**
 * @defgroup controller_execution Controller Execution Classes
 * @brief The controller execution classes are derived from the AbstractControllerExecution and extends the
 *        functionality. The base controller execution code is located in the AbstractControllerExecution.
 */

/**
 * @brief The AbstractControllerExecution class loads and binds the controller plugin. It contains a thread
 *        running the plugin in a cycle to move the robot. An internal state is saved and will be pulled by
 *        server, to monitor controller execution. Due to a state change it wakes up all threads connected
 *        to the condition variable.
 *
 * @ingroup abstract_server controller_execution
 */
  class AbstractControllerExecution : public AbstractExecutionBase
  {
  public:

    static const double DEFAULT_CONTROLLER_FREQUENCY;

    typedef std::shared_ptr<AbstractControllerExecution > Ptr;

    /**
     * @brief Constructor
     * @param name Name of this execution
     * @param controller_ptr Pointer to the controller plugin
     * @param robot_info Current robot state
     * @param vel_pub Velocity publisher
     * @param goal_pub Current goal publisher
     * @param config Initial configuration for this execution
     */
    AbstractControllerExecution(const std::string& name,
                                const mbf_abstract_core::AbstractController::Ptr& controller_ptr,
                                const mbf_utility::RobotInformation& robot_info,
                                const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& vel_pub,
                                const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& goal_pub,
                                const rclcpp::Node::SharedPtr& node_handle);

    /**
     * @brief Destructor
     */
    virtual ~AbstractControllerExecution();

    /**
     * @brief Starts the controller, a valid plan should be given in advance.
     * @return false if the thread is already running, true if starting the controller succeeded!
     */
    virtual bool start();

    /**
     * @brief Sets a new plan to the controller execution
     * @param plan A vector of stamped poses.
     * @param tolerance_from_action flag that will be set to true when the new plan (action) has tolerance
     * @param action_dist_tolerance distance to goal tolerance specific for this new plan (action)
     * @param action_angle_tolerance angle to goal tolerance specific for this new plan (action)
     */
    void setNewPlan(
      const std::vector<geometry_msgs::msg::PoseStamped> &plan,
      bool tolerance_from_action = false,
      double action_dist_tolerance = 1.0,
      double action_angle_tolerance = 3.1415);

    /**
     * @brief Cancel the controller execution. Normally called upon aborting the navigation.
     * This calls the cancel method of the controller plugin. If the plugins returns true, it becomes
     * responsible of stopping, and we will keep requesting velocity commands until it returns CANCELED.
     * If it returns false (meaning cancel is not implemented, or that the controller defers handling it),
     * MBF will set the cancel_ flag to true, and wait for the control loop to stop.
     * @return true, if the controller handles the stooping, or the control loop stops within a cycle time.
     */
    virtual bool cancel();

    /**
     * @brief Internal states
     */
    enum ControllerState
    {
      INITIALIZED,     ///< Controller has been initialized successfully.
      STARTED,         ///< Controller has been started.
      PLANNING,        ///< Executing the plugin.
      NO_PLAN,         ///< The controller has been started without a plan.
      MAX_RETRIES,     ///< Exceeded the maximum number of retries without a valid command.
      PAT_EXCEEDED,    ///< Exceeded the patience time without a valid command.
      EMPTY_PLAN,      ///< Received an empty plan.
      INVALID_PLAN,    ///< Received an invalid plan that the controller plugin rejected.
      NO_LOCAL_CMD,    ///< Received no velocity command by the plugin, in the current cycle.
      GOT_LOCAL_CMD,   ///< Got a valid velocity command from the plugin.
      ARRIVED_GOAL,    ///< The robot arrived the goal.
      CANCELED,        ///< The controller has been canceled.
      STOPPED,         ///< The controller has been stopped!
      INTERNAL_ERROR,  ///< An internal error occurred.
      ROBOT_DISABLED   ///< The robot is stuck and ignored velocity command
    };

    /**
     * @brief Return the current state of the controller execution. Thread communication safe.
     * @return current state, enum value of ControllerState
     */
    ControllerState getState() const;

    /**
     * @brief Returns the time of the last plugin call
     * @return Time of the last plugin call
     */
    rclcpp::Time getLastPluginCallTime() const;

    /**
     * @brief Returns the last velocity command calculated by the plugin. Set by setVelocityCmd method.
     * Note that it doesn't need to be a valid command sent to the robot, as we report also failed calls
     * to the plugin on controller action feedback.
     * @return The last valid velocity command.
     */
    geometry_msgs::msg::TwistStamped getVelocityCmd() const;

    /**
     * @brief Checks whether the patience duration time has been exceeded, ot not
     * @return true, if the patience has been exceeded.
     */
    bool isPatienceExceeded() const;

    /**
     * @brief Sets the controller frequency
     * @param frequency The controller frequency
     * @return true, if the controller frequency has been changed / set succesfully, false otherwise
     */
    bool setControllerFrequency(double frequency);

    /**
     * @brief Is called by the server thread to reconfigure the controller execution,
     *        if a user uses dynamic reconfigure to reconfigure the current state.
     * @param config The dynamic reconfigure config.
     */
    rcl_interfaces::msg::SetParametersResult reconfigure(std::vector<rclcpp::Parameter> parameters);

    /**
     * @brief Returns whether the robot should normally move or not. True if the controller seems to work properly.
     * @return true, if the robot should normally move, false otherwise
     */
    bool isMoving() const;

  protected:

    /**
     * @brief Request plugin for a new velocity command, given the current position, orientation, and velocity of the
     * robot. We use this virtual method to give concrete implementations as move_base the chance to override it and do
     * additional stuff, for example locking the costmap.
     * @param pose the current pose of the robot.
     * @param velocity the current velocity of the robot.
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
     * @param message Optional more detailed outcome as a string.
     * @return Result code as described on ExePath action result and plugin's header.
     */
    virtual uint32_t computeVelocityCmd(const geometry_msgs::msg::PoseStamped &pose,
                                        const geometry_msgs::msg::TwistStamped &velocity,
                                        geometry_msgs::msg::TwistStamped &vel_cmd, std::string &message);

    /**
     * @brief Sets the velocity command, to make it available for another thread
     * @param vel_cmd_stamped current velocity command
     */
    void setVelocityCmd(const geometry_msgs::msg::TwistStamped &vel_cmd_stamped);

    /**
     * @brief Check if the robot is ignoring the cmd_vel longer than threshold time
     * @param cmd_vel the latest cmd_vel being published by the controller
     * @return true if cmd_vel is being ignored by the robot longer than tolerance time, false otherwise
     */
    bool checkCmdVelIgnored(const geometry_msgs::msg::Twist& cmd_vel);

    //! the name of the loaded plugin
    std::string plugin_name_;

    //! The local planer to calculate the velocity command
    mbf_abstract_core::AbstractController::Ptr controller_;

    //! The current cycle start time of the last cycle run. Will by updated each cycle.
    rclcpp::Time last_call_time_;

    //! The time the controller has been started.
    rclcpp::Time start_time_;

    //! The time the controller responded with a success output (output < 10).
    rclcpp::Time last_valid_cmd_time_;

    //! The time when the robot started ignoring velocity commands
    rclcpp::Time first_ignored_time_;

    //! The maximum number of retries
    int max_retries_;

    //! The time / duration of patience, before changing the state.
    rclcpp::Duration patience_;

    //! the frame of the robot, which will be used to determine its position.
    std::string robot_frame_;

    //! the global frame the robot is controlling in.
    std::string global_frame_;

    /**
     * @brief The main run method, a thread will execute this method. It contains the main controller execution loop.
     */
    virtual void run();

    /**
     * @brief Check if its safe to drive.
     * This method gets called at every controller cycle, stopping the robot if its not. When overridden by
     * child class, gives a chance to the specific execution implementation to stop the robot if it detects
     * something wrong on the underlying map.
     * @return Always true, unless overridden by child class.
     */
    virtual bool safetyCheck() { return true; };

  private:


    /**
     * @brief Publishes a velocity command with zero values to stop the robot.
     */
    void publishZeroVelocity();

    /**
     * @brief Checks whether the goal has been reached in the range of tolerance or not
     * @return true if the goal has been reached, false otherwise
     */
    bool reachedGoalCheck();

    /**
     * @brief Sets the controller state. This method makes the communication of the state thread safe.
     * @param state The current controller state.
     */
    void setState(ControllerState state);

    //! mutex to handle safe thread communication for the current value of the state
    mutable std::mutex state_mtx_;

    //! mutex to handle safe thread communication for the current plan
    mutable std::mutex plan_mtx_;

    //! mutex to handle safe thread communication for the current velocity command
    mutable std::mutex vel_cmd_mtx_;

    //! mutex to handle safe thread communication for the last plugin call time
    mutable std::mutex lct_mtx_;

    //! true, if a new plan is available. See hasNewPlan()!
    bool new_plan_;

    /**
     * @brief Returns true if a new plan is available, false otherwise! A new plan is set by another thread!
     * @return true, if a new plan has been set, false otherwise.
     */
    bool hasNewPlan();

    /**
     * @brief Gets the new available plan. This method is thread safe.
     * @return The plan
     */
    std::vector<geometry_msgs::msg::PoseStamped> getNewPlan();

    //! the last calculated velocity command
    geometry_msgs::msg::TwistStamped vel_cmd_stamped_;

    //! the last set plan which is currently processed by the controller
    std::vector<geometry_msgs::msg::PoseStamped> plan_;

    //! the loop_rate which corresponds with the controller frequency.
    std::shared_ptr<rclcpp::Rate> loop_rate_;

    //! publisher for the current velocity command
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    //! publisher for the current goal
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    //! publisher for the current goal
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

    //! the current controller state
    AbstractControllerExecution::ControllerState state_;

    //! time before a timeout used for tf requests
    double tf_timeout_;

    //! dynamic reconfigure config mutex, thread safe param reading and writing
    std::mutex configuration_mutex_;

    //! main controller loop variable, true if the controller is running, false otherwise
    bool moving_;

    //! whether move base flex should check for the goal tolerance or not.
    bool mbf_tolerance_check_;

    //! whether move base flex should force the robot to stop once the goal is reached.
    bool force_stop_at_goal_;

    //! whether move base flex should force the robot to stop on canceling navigation.
    bool force_stop_on_cancel_;

    //! whether move base flex should force the robot to stop while retrying controller calls
    //! (that is, until patience is exceeded or retries are exhausted).
    bool force_stop_on_retry_;

    //! distance tolerance to the given goal pose
    double dist_tolerance_;

    //! angle tolerance to the given goal pose
    double angle_tolerance_;

    //! current robot pose;
    geometry_msgs::msg::PoseStamped robot_pose_;

    //! whether check for action specific tolerance
    bool tolerance_from_action_;

    //! replaces parameter dist_tolerance_ for the action
    double action_dist_tolerance_;

    //! replaces parameter angle_tolerance_ for the action
    double action_angle_tolerance_;

    //! time tolerance for checking if the robot is ignoring cmd_vel
    double cmd_vel_ignored_tolerance_;

    //dynamic configuration
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    //node handle
    const rclcpp::Node::SharedPtr& node_handle_;
  };

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_ */
