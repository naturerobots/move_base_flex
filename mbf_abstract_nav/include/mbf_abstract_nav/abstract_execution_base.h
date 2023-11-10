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
 *  abstract_execution_base.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_

#include <thread>
#include <chrono>
#include <condition_variable>
#include <mbf_utility/robot_information.h>
#include <string>


namespace mbf_abstract_nav
{
/**
 * @brief Base class for running concurrent navigation tasks.
 *
 * The class uses a dedicated thread to run potentially long-lasting jobs.
 * The user can use waitForStateUpdate to get notification about the progress
 * of the said job.
 */
class AbstractExecutionBase
{
 public:
   AbstractExecutionBase(const std::string& name, const mbf_utility::RobotInformation& robot_info);

   virtual ~AbstractExecutionBase();

   virtual bool start();

   virtual void stop();

   /**
    * @brief Cancel the plugin execution.
    * @return true, if the plugin tries / tried to cancel the computation.
    */
   virtual bool cancel()
   {
     return false;
   };

   void join();

   std::cv_status waitForStateUpdate(std::chrono::microseconds const& duration);

   /**
    * @brief Gets the current plugin execution outcome
    */
   uint32_t getOutcome() const;

   /**
    * @brief Gets the current plugin execution message
    */
   const std::string& getMessage() const;

   /**
    * @brief Returns the name of the corresponding plugin
    */
   const std::string& getName() const;

   /**
    * @brief Optional implementation-specific setup function, called right before execution.
    */
   virtual void preRun(){};

   /**
    * @brief Optional implementation-specific cleanup function, called right after execution.
    */
   virtual void postRun(){};

   /**
    * @brief Optional implementaiton-specific configuration function.
    */
   virtual rcl_interfaces::msg::SetParametersResult reconfigure(std::vector<rclcpp::Parameter> parameters)
   {
   }

protected:
  virtual void run(){};

  //! condition variable to wake up control thread
  std::condition_variable condition_;

  //! the controlling thread object
  std::thread thread_;

  //! mutex for locking the condition variable
  std::mutex mutex_;
  std::condition_variable cv_;
  bool shouldExit;

  //! mutex for locking the condition variable
  std::mutex mutex;
  std::condition_variable cv;
  bool shouldExit = false;

  //! flag for canceling controlling
  bool cancel_;

  //! the last received plugin execution outcome
  uint32_t outcome_;

  //! the last received plugin execution message
  std::string message_;

  //! the plugin name; not the plugin type!
  std::string name_;

  //! Reference to the current robot state
  const mbf_utility::RobotInformation& robot_info_;

private:
  void runWrapper();
};

} /* namespace mbf_abstract_nav */

#endif  /* MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_ */
