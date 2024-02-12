/*
 *  Copyright 2018, Sebastian Pütz
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
 *  abstract_action.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_ACTION_BASE_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_ACTION_BASE_H_

#include <mutex>
#include <functional>

#include <memory>
#include <string>
#include <map>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mbf_utility/robot_information.h>

#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{

/**
 * Base class for managing multiple concurrent executions.
 *
 * @tparam Action an actionlib-compatible action
 * @tparam Execution a class implementing the AbstractExecutionBase
 *
 * Place the implementation specific code into AbstractActionBase::runImpl.
 * Also it is required, that you define MyExecution::Ptr as a shared pointer
 * for your execution.
 *
 */
template <typename Action, typename Execution>
class AbstractActionBase
{
 public:
  typedef std::shared_ptr<AbstractActionBase> Ptr;
  typedef typename rclcpp_action::ServerGoalHandle<Action> GoalHandle;
  typedef typename std::shared_ptr<GoalHandle> GoalHandlePtr;

  /// @brief POD holding info for one execution
  struct ConcurrencySlot{
    ConcurrencySlot() : thread_ptr(NULL), in_use(false){}
    typename Execution::Ptr execution;
    std::thread* thread_ptr; ///< Owned pointer to a thread
    GoalHandlePtr goal_handle;
    bool in_use;
  };

protected:
  // not part of the public interface
  typedef std::unordered_map<uint8_t, ConcurrencySlot> ConcurrencyMap;
public:

  /**
   * @brief Construct a new AbstractActionBase
   *
   * @param name name of the AbstractActionBase
   * @param robot_info robot information
   */
  AbstractActionBase(
      const rclcpp::Node::SharedPtr& node,
      const std::string &name,
      const mbf_utility::RobotInformation::ConstPtr &robot_info
  ) : node_(node), name_(name), robot_info_(robot_info){}

  virtual ~AbstractActionBase()
  {
    // cleanup threads used on executions
    // note: cannot call cancelAll, since our mutex is not recursive
    std::lock_guard<std::mutex> guard(slot_map_mtx_);
    for (auto& [slot_id, concurrency_slot] : concurrency_slots_)
    {
      // cancel and join all spawned threads.
      concurrency_slot.execution->cancel();
      if(concurrency_slot.thread_ptr->joinable())
        concurrency_slot.thread_ptr->join();
      // delete
      delete concurrency_slot.thread_ptr;
    }
  }

  virtual void start(
      const GoalHandlePtr &goal_handle,
      typename Execution::Ptr execution_ptr
  )
  {
    uint8_t slot_id = goal_handle->get_goal()->concurrency_slot;

    if(goal_handle->is_canceling())
    {
      typename Action::Result::SharedPtr result = std::make_shared<typename Action::Result>();
      goal_handle->canceled(result); // TODO why trigger cancel if the goal is already being cancelled?
    }
    else
    {
      std::lock_guard<std::mutex> guard(slot_map_mtx_);
      typename ConcurrencyMap::iterator slot_it = concurrency_slots_.find(slot_id);
      if (slot_it != concurrency_slots_.end())
      {
        if (slot_it->second.in_use) {
          // if there is already a plugin running on the same slot, cancel it
          slot_it->second.execution->cancel();
        }

        // TODO + WARNING: this will block the main thread for an arbitrary time during which we won't execute callbacks
        if (slot_it->second.thread_ptr->joinable()) {
          slot_it->second.thread_ptr->join();
        }
        // cleanup previous execution; otherwise we will leak threads
        delete concurrency_slots_[slot_id].thread_ptr;
      }
      else
      {
        // create a new map object in order to avoid costly lookups
        // note: currently unchecked
        slot_it = concurrency_slots_.insert(std::make_pair(slot_id, ConcurrencySlot())).first;
      }

      // fill concurrency slot with the new goal handle, execution, and working thread
      slot_it->second.in_use = true;
      slot_it->second.goal_handle = goal_handle;
      //slot_it->second.goal_handle->setAccepted(); TODO can probably be removed; accept should happen in handle_goal. Check: Move cancel-same-slot code there as well?
      slot_it->second.execution = execution_ptr;
      slot_it->second.thread_ptr = new std::thread(
        std::bind(&AbstractActionBase::run, this, std::ref(concurrency_slots_[slot_id])));
    }
  }

  virtual void cancel(GoalHandlePtr goal_handle)
  {
    uint8_t slot = goal_handle->get_goal()->concurrency_slot;

    std::lock_guard<std::mutex> guard(slot_map_mtx_);
    typename ConcurrencyMap::iterator slot_it = concurrency_slots_.find(slot);
    if (slot_it != concurrency_slots_.end())
    {
      concurrency_slots_[slot].execution->cancel();
    }
  }

  virtual void runImpl(GoalHandle &goal_handle, Execution& execution) {};

  virtual void run(ConcurrencySlot &slot)
  {
    slot.execution->preRun();
    runImpl(*slot.goal_handle, *slot.execution);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Finished action \"" << name_ << "\" run method, waiting for execution thread to finish.");
    slot.execution->join();
    // TODO reenable debug output, if possible. how do we get the state from ROS2 Action Server GoalHandle?
    //RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_), "Execution completed with goal status "
    //                       << (int)slot.goal_handle->getGoalStatus().status << ": "<< slot.goal_handle->getGoalStatus().text);
    slot.execution->postRun();
    slot.in_use = false;
  }

  virtual void cancelAll()
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(name_), "Cancel all goals for \"" << name_ << "\".");
    std::lock_guard<std::mutex> guard(slot_map_mtx_);
    for (auto& [slot_id, concurrency_slot] : concurrency_slots_)
    {
      concurrency_slot.execution->cancel();
    }
    for (auto& [slot_id, concurrency_slot] : concurrency_slots_)
    {
      if (concurrency_slot.thread_ptr->joinable()) concurrency_slot.thread_ptr->join();
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  const std::string name_;
  mbf_utility::RobotInformation::ConstPtr robot_info_;

  ConcurrencyMap concurrency_slots_;

  std::mutex slot_map_mtx_;

};

}

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_ACTION_BASE_H_ */
