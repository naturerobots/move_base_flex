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
 *  abstract_plugin_manager.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_

#include <functional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

namespace mbf_abstract_nav
{

template <typename PluginType>
class AbstractPluginManager
{
 public:

  typedef std::function<typename PluginType::Ptr(const std::string &plugin)> loadPluginFunction;
  typedef std::function<bool(const std::string& name, const typename PluginType::Ptr& plugin_ptr)> initPluginFunction;

  AbstractPluginManager(
      const std::string &param_name,
      const loadPluginFunction &loadPlugin,
      const initPluginFunction &initPlugin,
      const rclcpp::Node::SharedPtr& node_handle
  );

  //! Returns false if no plugins were loaded. Returns true if at least one plugin was loaded.
  bool loadPlugins();

  bool hasPlugin(const std::string &name) const;

  std::string getType(const std::string &name) const;

  std::vector<std::string> getLoadedNames() const;

  typename PluginType::Ptr getPlugin(const std::string &name);

  void clearPlugins();

 protected:
  //! maps plugin names to plugin types, as defined by ros params.
  std::unordered_map<std::string, std::string> plugins_type_;
  //! maps plugin names to instances of loaded plugins
  std::unordered_map<std::string, typename PluginType::Ptr> plugins_;
  const std::string param_name_;
  const loadPluginFunction loadPlugin_;
  const initPluginFunction initPlugin_;
  rclcpp::Node::SharedPtr node_handle_;
};

} /* namespace mbf_abstract_nav */

#include "impl/abstract_plugin_manager.tcc"
#endif /* MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_ */
