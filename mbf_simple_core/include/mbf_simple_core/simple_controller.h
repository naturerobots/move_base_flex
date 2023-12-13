/*
 *  Copyright 2023, Sebastian Pütz
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
 *  simple_controller.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_SIMPLE_CORE__SIMPLE_CONTROLLER_H_
#define MBF_SIMPLE_CORE__SIMPLE_CONTROLLER_H_

#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_utility/types.h>

namespace mbf_simple_core {
  /**
   * @class LocalPlanner
   * @brief Provides an interface for local planners used in navigation.
   * All local planners written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class SimpleController : public mbf_abstract_core::AbstractController{
    public:

      typedef boost::shared_ptr< ::mbf_simple_core::SimpleController > Ptr;

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands
       * to send to the base.
       * @param pose the current pose of the robot.
       * @param velocity the current velocity of the robot.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on ExePath action result:
       *         SUCCESS           = 0
       *         1..9 are reserved as plugin specific non-error results
       *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
       *         CANCELED          = 101
       *         NO_VALID_CMD      = 102
       *         PAT_EXCEEDED      = 103
       *         COLLISION         = 104
       *         OSCILLATION       = 105
       *         ROBOT_STUCK       = 106
       *         MISSED_GOAL       = 107
       *         MISSED_PATH       = 108
       *         BLOCKED_GOAL      = 109
       *         BLOCKED_PATH      = 110
       *         INVALID_PATH      = 111
       *         TF_ERROR          = 112
       *         NOT_INITIALIZED   = 113
       *         INVALID_PLUGIN    = 114
       *         INTERNAL_ERROR    = 115
       *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
       *         MAP_ERROR         = 117  # The map is not running properly
       *         STOPPED           = 118  # The controller execution has been stopped rigorously
       *         121..149 are reserved as plugin specific errors
       */
      virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                               const geometry_msgs::TwistStamped& velocity,
                                               geometry_msgs::TwistStamped &cmd_vel,
                                               std::string &message) = 0;

      /**
       * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
       * @remark New on MBF API
       * @param xy_tolerance Distance tolerance in meters
       * @param yaw_tolerance Heading tolerance in radians
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance) = 0;

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes too much time
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel() = 0;

      /**
       * @brief Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       */
      virtual void initialize(std::string name, ::TF *tf) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~SimpleController(){}

    protected:
      SimpleController(){}

  };
}  /* namespace mbf_simple_core */

#endif  /* MBF_SIMPLE_CORE__SIMPLE_CONTROLLER_H_ */
