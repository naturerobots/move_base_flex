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
 *  planner_action.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__PLANNER_ACTION_H_
#define MBF_ABSTRACT_NAV__PLANNER_ACTION_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>

#include <mbf_msgs/action/get_path.hpp>
#include <mbf_utility/robot_information.h>

#include "mbf_abstract_nav/abstract_action_base.hpp"
#include "mbf_abstract_nav/abstract_planner_execution.h"


namespace mbf_abstract_nav {


class PlannerAction : public AbstractActionBase<mbf_msgs::action::GetPath::Action, AbstractPlannerExecution>
{
 public:

  typedef std::shared_ptr<PlannerAction> Ptr;

  PlannerAction(
      const rclcpp::Node::SharedPtr& node,
      const std::string &name,
      const mbf_utility::RobotInformation &robot_info
  );

  void runImpl(GoalHandle &goal_handle, AbstractPlannerExecution &execution);

 protected:

  /**
   * @brief Transforms a plan to the global frame (global_frame_) coord system.
   * @param plan Input plan to be transformed.
   * @param global_plan Output plan, which is then transformed to the global frame.
   * @return true, if the transformation succeeded, false otherwise
   */
   bool transformPlanToGlobalFrame(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                   std::vector<geometry_msgs::msg::PoseStamped>& global_plan);

 private:

  //! Publisher to publish the current goal pose, which is used for path planning
  rclcpp::Publisher current_goal_pub_;

  //! Path sequence counter
  unsigned int path_seq_count_;
};

} /* mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__PLANNER_ACTION_H_ */
