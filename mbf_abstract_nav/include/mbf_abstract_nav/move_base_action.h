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
 *  move_base_action.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#ifndef MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
#define MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mbf_msgs/action/move_base.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <mbf_msgs/action/recovery.hpp>

#include <mbf_utility/robot_information.h>


namespace mbf_abstract_nav
{

class MoveBaseAction
{
 public:

  //! Action clients for the MoveBase action
  typedef mbf_msgs::action::GetPath GetPath;
  typedef mbf_msgs::action::ExePath ExePath;
  typedef mbf_msgs::action::Recovery Recovery;

  typedef rclcpp_action::Client<mbf_msgs::action::MoveBase>::GoalHandle GoalHandle;

  MoveBaseAction(const std::string &name,
                 const mbf_utility::RobotInformation &robot_info,
                 const std::vector<std::string> &controllers,
                 const rclcpp::Node::ConstSharedPtr &node);

  ~MoveBaseAction();

  void start(GoalHandle &goal_handle);

  void cancel();

  rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter> &parameters);

 protected:

  void actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback);

  void actionGetPathDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::GetPathResultConstPtr &result);

  void actionExePathActive();

  void actionExePathDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::ExePathResultConstPtr &result);

  void actionRecoveryDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::RecoveryResultConstPtr &result);

  bool attemptRecovery();

  bool replanningActive() const;

  void replanningThread();

  /**
   * Utility method that fills move base action result with the result of any of the action clients.
   * @tparam ResultType
   * @param result
   * @param move_base_result
   */
  template <typename ResultType>
  void fillMoveBaseResult(const ResultType& result, mbf_msgs::action::MoveBase::Result& move_base_result)
  {
    // copy outcome and message from action client result
    move_base_result.outcome = result.outcome;
    move_base_result.message = result.message;
    move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
    move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
    move_base_result.final_pose = robot_pose_;
  }

  mbf_msgs::action::ExePath::Goal exe_path_goal_;
  mbf_msgs::action::GetPath::Goal get_path_goal_;
  mbf_msgs::action::Recovery::Goal recovery_goal_;

  geometry_msgs::msg::PoseStamped last_oscillation_pose_;
  rclcpp::Time last_oscillation_reset_;

  //! timeout after a oscillation is detected
  rclcpp::Duration oscillation_timeout_;

  //! minimal move distance to not detect an oscillation
  double oscillation_distance_;

  GoalHandle goal_handle_;

  std::string name_;

  //! current robot state
  const mbf_utility::RobotInformation &robot_info_;

  //! current robot pose; updated with exe_path action feedback
  geometry_msgs::msg::PoseStamped robot_pose_;

  //! current goal pose; used to compute remaining distance and angle
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp::Node::ConstSharedPtr node_;

  //! Action client used by the move_base action
  rclcpp_action::Client<ExePath>::SharedPtr action_client_exe_path_;

  //! Action client used by the move_base action
  rclcpp_action::Client<GetPath>::SharedPtr action_client_get_path_;

  //! Action client used by the move_base action
  rclcpp_action::Client<Recovery>::SharedPtr action_client_recovery_;

  //! current distance to goal (we will stop replanning if very close to avoid destabilizing the controller)
  double dist_to_goal_;

  //! Replanning period dynamically reconfigurable
  rclcpp::Duration replanning_period_;

  //! Replanning thread, running permanently
  boost::thread replanning_thread_;
  bool replanning_thread_shutdown_;

  //! true, if recovery behavior for the MoveBase action is enabled.
  bool recovery_enabled_;

  std::vector<std::string> recovery_behaviors_;

  std::vector<std::string>::iterator current_recovery_behavior_;

  const std::vector<std::string> &behaviors_;

  enum MoveBaseActionState
  {
    NONE,
    GET_PATH,
    EXE_PATH,
    RECOVERY,
    OSCILLATING,
    SUCCEEDED,
    CANCELED,
    FAILED
  };

  MoveBaseActionState action_state_;
  MoveBaseActionState recovery_trigger_;
};

} /* mbf_abstract_nav */

#endif //MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
