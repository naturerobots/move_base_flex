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
 *  simple_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_SIMPLE_CORE__SIMPLE_PLANNER_H_
#define MBF_SIMPLE_CORE__SIMPLE_PLANNER_H_

#include <mbf_abstract_core/abstract_planner.h>

namespace mbf_simple_core {
  /**
   * @class SimplePlanner
   * @brief Provides an interface for global planners used in navigation.
   * All global planners written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class SimplePlanner : public mbf_abstract_core::AbstractPlanner{
    public:

      typedef boost::shared_ptr< ::mbf_simple_core::SimplePlanner > Ptr;

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
       *        in x and y before failing
       * @param plan The plan... filled by the planner
       * @param cost The cost for the the plan
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on GetPath action result:
       *         SUCCESS         = 0
       *         1..9 are reserved as plugin specific non-error results
       *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
       *         CANCELED        = 51
       *         INVALID_START   = 52
       *         INVALID_GOAL    = 53
       *         BLOCKED_START   = 54
       *         BLOCKED_GOAL    = 55
       *         NO_PATH_FOUND   = 56
       *         PAT_EXCEEDED    = 57
       *         EMPTY_PATH      = 58
       *         TF_ERROR        = 59
       *         NOT_INITIALIZED = 60
       *         INVALID_PLUGIN  = 61
       *         INTERNAL_ERROR  = 62
       *         71..99 are reserved as plugin specific errors
       */
      virtual uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message) = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes too much time.
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel() = 0;

      /**
       * @brief Initialization function for the SimplePlanner
       * @param name The name of this planner
       */
      virtual void initialize(std::string name) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~SimplePlanner(){}

    protected:
      SimplePlanner(){}

  };
}  /* namespace mbf_simple_core */

#endif  /* MBF_SIMPLE_CORE__SIMPLE_PLANNER_H_ */
