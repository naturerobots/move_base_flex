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
 *  simple_recovery.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_SIMPLE_CORE__SIMPLE_RECOVERY_H_
#define MBF_SIMPLE_CORE__SIMPLE_RECOVERY_H_

#include <mbf_abstract_core/abstract_recovery.h>
#include <mbf_utility/types.h>

namespace mbf_simple_core
{
/**
 * @class SimpleRecovery
 * @brief Provides an interface for recovery behaviors used in navigation.
 * All recovery behaviors written to work as MBF plugins must adhere to this interface. Alternatively, this
 * class can also operate as a wrapper for old API nav_corebased plugins, providing backward compatibility.
 */
class SimpleRecovery : public mbf_abstract_core::AbstractRecovery{
 public:

  typedef std::shared_ptr< ::mbf_simple_core::SimpleRecovery> Ptr;

  /**
   * @brief Initialization function for the SimpleRecovery
   * @param tf A pointer to a transform listener
   */
  virtual void initialize(std::string name, TF* tf) = 0;

  /**
   * @brief Runs the SimpleRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Virtual destructor for the interface
   */
  virtual ~SimpleRecovery(){}

  SimpleRecovery() = delete;

};
}  /* namespace mbf_simple_core */

#endif /* MBF_SIMPLE_CORE__SIMPLE_RECOVERY_H_ */
