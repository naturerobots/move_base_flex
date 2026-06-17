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

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLAN_REFINER_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLAN_REFINER_EXECUTION_H_

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mbf_abstract_core/abstract_plan_refiner.h>

#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{
/**
 * @defgroup plan_refiner_execution Plan Refiner Execution Classes
 * @brief The plan refiner execution classes are derived from the PlanRefinerExecution and extends the functionality.
 *        The base plan refiner execution code is located in the AbstractPlanRefinerExecution.
 */

/**
 * @brief The AbstractPlanRefinerExecution class loads and binds the plan refiner plugin.
 *        It contains a thread running the plugin, executing the plan refiner. An internal state
 *        is saved and will be pulled by the server, which controls the plan refiner execution.
 *        Due to a state change it wakes up all threads connected to the condition variable.
 *
 * @ingroup plan_refiner_execution
 */

class AbstractPlanRefinerExecution : public AbstractExecutionBase
{
public:
  typedef std::shared_ptr<AbstractPlanRefinerExecution> Ptr;

  /**
   * @brief Constructor
   * @param name Name of this execution
   * @param plan_refiner_ptr Pointer to the plan refiner plugin
   * @param robot_info Current robot state
   * @param node_handle Node handle for parameter access
   */
  AbstractPlanRefinerExecution(
    const std::string & name, const mbf_abstract_core::AbstractPlanRefiner::Ptr & plan_refiner_ptr,
    const mbf_utility::RobotInformation::ConstPtr & robot_info,
    const rclcpp::Node::SharedPtr & node_handle);

  /**
   * @brief Destructor
   */
  virtual ~AbstractPlanRefinerExecution();

  /**
   * @brief Internal states
   */
  enum PlanRefinerState
  {
    INITIALIZED,    ///< Plan refiner initialized.
    STARTED,        ///< Plan refiner started.
    REFINING,       ///< Executing the plugin.
    REFINED_PLAN,   ///< Found a valid refined plan.
    MAX_RETRIES,    ///< Exceeded the maximum number of retries without a valid command.
    NO_PLAN_FOUND,  ///< No refined plan has been found.
    CANCELED,       ///< The plan refiner has been canceled.
    STOPPED,        ///< The plan refiner has been stopped.
    INTERNAL_ERROR  ///< An internal error occurred.
  };

  /**
   * @brief Returns the current state, thread-safe communication
   * @return current internal state
   */
  PlanRefinerState getState() const;

  /**
   * @brief Get the position error
   * @return Error as float
   */
  float getPositionError() const;

  /**
   * @brief Get the orientation error
   * @return Error as float
   */
  float getOrientationError() const;

  /**
   * @brief Get the length ratio of the refined path to the reference path
   * @return Error as float
   */
  float getPathLengthRatio() const;

  /**
   * @brief Cancel the plan refiner execution. This calls the cancel method of the plan refiner plugin.
   * This could be useful if the computation takes too much time, or if we are aborting the navigation.
   * @return true, if the refiner plugin tries / tried to cancel the refining step.
   */
  bool cancel();

  /**
   * @brief Starts the plan refiner execution. This calls the start method of the plan refiner plugin.
   * @return true, if the refiner plugin tries / tried to start the refining step.
   */
  bool start(const std::vector<geometry_msgs::msg::PoseStamped> & plan);

  /**
   * @brief Sets a new plan for the plan refiner execution
   * @param plan the new plan to refine
   */
  void setNewPlan(const std::vector<geometry_msgs::msg::PoseStamped> & plan);

  /**
   * @brief Returns the current plan
   * @return the current plan
   */
  std::vector<geometry_msgs::msg::PoseStamped> getPlan() const;

protected:
  //! the plan refiner to refine the given path
  mbf_abstract_core::AbstractPlanRefiner::Ptr plan_refiner_;

  //! the name of the loaded plan refiner plugin
  std::string plugin_name_;

  /**
   * @brief The main run method, a thread will execute this method. It contains the main plan refiner execution loop.
   */
  virtual void run();

  //! Helper method for cleaning up the state when the plan refiner thread was interrupted.
  void handle_thread_interrupted();

private:
  /**
   * @brief Calls the plan refiner plugin to refine the given plan.
   * @param plan The plan to refine
   * @param refined_plan The computed refined plan by the plugin
   * @param position_error The computed position error between the original and refined plan end points
   * @param orientation_error The computed orientation error between the original and refined plan end points
   * @param path_length_ratio The computed path length ratio between the original and refined plan
   * @param message An optional message which should correspond with the returned outcome
   * @return An outcome number, see also the action definition in the RefinePath.action file
   */
  uint32_t refinePlan(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & refined_plan,
    float & position_error,
    float & orientation_error,
    float & path_length_ratio,
    std::string & message);

  /**
   * @brief Sets the internal state, thread communication safe
   * @param state the current state
   * @param signalling set true to trigger the condition-variable for state-update
   */
  void setState(PlanRefinerState state, bool signalling);

  //! mutex to handle safe thread communication for the current state
  mutable std::mutex state_mtx_;

  //! mutex to handle safe thread communication for the plan
  mutable std::mutex plan_mtx_;

  //! mutex to handle safe thread communication for the refining_ flag.
  mutable std::mutex refining_mtx_;

  //! current global plan
  std::vector<geometry_msgs::msg::PoseStamped> plan_;

  //! true, if a new plan has been set, until it is used.
  bool has_new_plan_;

  //! refining max retries
  int max_retries_;

  //! main cycle variable of the execution loop
  bool refining_;

  //! current position error
  float current_position_error_;

  //! current orientation error
  float current_orientation_error_;

  //! current length ratio
  float current_path_length_ratio_;

  //! robot frame used for computing the current robot pose
  std::string robot_frame_;

  //! the global frame in which the refiner needs to plan
  std::string global_frame_;

  //! current internal state
  PlanRefinerState state_;

  //! the node handle ptr
  rclcpp::Node::SharedPtr node_handle_;

  // dynamic configuration
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

};
}  /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_PLAN_REFINER_EXECUTION_H_ */
