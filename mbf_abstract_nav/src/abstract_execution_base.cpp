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
 *  abstract_execution_base.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{
AbstractExecutionBase::AbstractExecutionBase(const std::string& name, const mbf_utility::RobotInformation& robot_info, const rclcpp::Node::SharedPtr& node)
  : should_exit_(false), outcome_(255), cancel_(false), name_(name), robot_info_(robot_info), node_(node)
{ }

AbstractExecutionBase::~AbstractExecutionBase()
{
  if (thread_.joinable())
  {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }
}

bool AbstractExecutionBase::start()
{
  if (thread_.joinable())
  {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }

  should_exit_ = false;
  thread_ = std::thread(&AbstractExecutionBase::run, this);
  return true;
}

void AbstractExecutionBase::stop()
{
  RCLCPP_WARN_STREAM(node_->get_logger(),
                     "Try to stop the plugin \"" << name_ << "\" rigorously by notifying the thread!");

  {
    // Set the exit flag in a critical section
    std::unique_lock<std::mutex> lock(should_exit_mutex_);
    should_exit_ = true;
  }
}

void AbstractExecutionBase::join()
{
  if (thread_.joinable())
    thread_.join();
}

std::cv_status AbstractExecutionBase::waitForStateUpdate(std::chrono::microseconds const& duration)
{
  std::mutex mutex;
  std::unique_lock<std::mutex> lock(mutex);
  return condition_.wait_for(lock, duration);
}

uint32_t AbstractExecutionBase::getOutcome() const
{
  return outcome_;
}

const std::string& AbstractExecutionBase::getMessage() const
{
  return message_;
}

const std::string& AbstractExecutionBase::getName() const
{
  return name_;
}

} /* namespace mbf_abstract_nav */
