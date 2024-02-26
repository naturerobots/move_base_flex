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

#include <thread>

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

//! ExePath action topic name
const std::string name_action_exe_path = "~/exe_path";
//! GetPath action topic name
const std::string name_action_get_path = "~/get_path";
//! Recovery action topic name
const std::string name_action_recovery = "~/recovery";
//! MoveBase action topic name
const std::string name_action_move_base = "~/move_base";

class MoveBaseAction
{
 public:
  typedef std::shared_ptr<MoveBaseAction> Ptr;

  //! Action clients for the MoveBase action
  typedef mbf_msgs::action::GetPath GetPath;
  typedef mbf_msgs::action::ExePath ExePath;
  typedef mbf_msgs::action::Recovery Recovery;

  typedef rclcpp_action::ServerGoalHandle<mbf_msgs::action::MoveBase> GoalHandle;

  MoveBaseAction(const rclcpp::Node::SharedPtr &node, const std::string &name,
                 const mbf_utility::RobotInformation::ConstPtr &robot_info,
                 const std::vector<std::string> &controllers);

  ~MoveBaseAction();

  void start(std::shared_ptr<GoalHandle> goal_handle);

  void cancel();

  rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter> &parameters);

 protected:

  void actionGetPathGoalResponse(const rclcpp_action::ClientGoalHandle<GetPath>::ConstSharedPtr& get_path_goal_handle);
  void actionGetPathResult(const rclcpp_action::ClientGoalHandle<GetPath>::WrappedResult &result);

  void actionExePathGoalResponse(const rclcpp_action::ClientGoalHandle<ExePath>::ConstSharedPtr& exe_path_goal_handle);
  void actionExePathFeedback(const rclcpp_action::ClientGoalHandle<ExePath>::ConstSharedPtr& goal_handle, const ExePath::Feedback::ConstSharedPtr &feedback);
  void actionExePathResult(const rclcpp_action::ClientGoalHandle<ExePath>::WrappedResult &result);

  void actionRecoveryGoalResponse(const rclcpp_action::ClientGoalHandle<Recovery>::ConstSharedPtr& recovery_goal_handle);
  void actionRecoveryResult(const rclcpp_action::ClientGoalHandle<Recovery>::WrappedResult &result);
  void recoveryRejectedOrAborted(const rclcpp_action::ClientGoalHandle<Recovery>::WrappedResult &result); // TODO keep?

  //! Checks whether the move base client requested canceling of action. If so, returns true and handles goal state transition. Otherwise, returns false.
  //! Regularly call this function before doing further work (e.g. before calling the next exepath action), so canceling remains responsive.
  bool checkAndHandleMoveBaseActionCanceled();

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

  ExePath::Goal exe_path_goal_;
  rclcpp_action::Client<ExePath>::SendGoalOptions exe_path_send_goal_options_;
  GetPath::Goal get_path_goal_;
  std::shared_future<rclcpp_action::ClientGoalHandle<GetPath>::SharedPtr> get_path_goal_handle_;
  rclcpp_action::Client<GetPath>::SendGoalOptions get_path_send_goal_options_;
  Recovery::Goal recovery_goal_;
  rclcpp_action::Client<Recovery>::SendGoalOptions recovery_send_goal_options_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  geometry_msgs::msg::PoseStamped last_oscillation_pose_;
  rclcpp::Time last_oscillation_reset_;

  //! timeout after a oscillation is detected
  rclcpp::Duration oscillation_timeout_;

  //! minimal move distance to not detect an oscillation
  double oscillation_distance_;

  //! handle of the active MoveBase action goal, if one exists
  std::shared_ptr<GoalHandle> goal_handle_;

  std::string name_;

  //! current robot state
  mbf_utility::RobotInformation::ConstPtr robot_info_;

  //! current robot pose; updated with exe_path action feedback
  geometry_msgs::msg::PoseStamped robot_pose_;

  //! current goal pose; used to compute remaining distance and angle
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp::Node::SharedPtr node_;

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
  std::thread replanning_thread_;
  bool replanning_thread_shutdown_;

  //! true, if recovery behavior for the MoveBase action is enabled.
  bool recovery_enabled_;
  //! Gets set when a move base actions starts. These are the recovery behaviors that will be used by the move base action.
  std::vector<std::string> actions_recovery_behaviors_;
  //! Points to an element in actions_recovery_behaviors_. This is the current recovery behavior, we might try different behaviors for recovery one after another.
  std::vector<std::string>::iterator current_recovery_behavior_;
  //! All available recovery behaviors. Gets set in the constructor, will be used as default actions_recovery_behaviors_ if none are specified in the action goal.
  std::vector<std::string> available_recovery_behaviors_;

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
