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

#ifndef MBF_ABSTRACT_NAV__PLAN_REFINER_ACTION_H_
#define MBF_ABSTRACT_NAV__PLAN_REFINER_ACTION_H_

#include <mbf_msgs/action/refine_path.hpp>
#include <mbf_utility/robot_information.h>

#include "mbf_abstract_nav/abstract_action_base.hpp"
#include "mbf_abstract_nav/abstract_plan_refiner_execution.h"

namespace mbf_abstract_nav
{

class PlanRefinerAction : public AbstractActionBase<mbf_msgs::action::RefinePath,
    AbstractPlanRefinerExecution>
{
public:
  typedef std::shared_ptr<PlanRefinerAction> Ptr;

  PlanRefinerAction(
    const rclcpp::Node::SharedPtr & node, const std::string & name,
    const mbf_utility::RobotInformation::ConstPtr & robot_info);

  void runImpl(
    const GoalHandlePtr & goal_handle,
    AbstractPlanRefinerExecution & execution) override;

};

}  /* namespace mbf_abstract_nav */

#endif  /* MBF_ABSTRACT_NAV__PLAN_REFINER_ACTION_H_ */
