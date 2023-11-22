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

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_

#include "mbf_abstract_nav/abstract_plugin_manager.h"

namespace mbf_abstract_nav{

template <typename PluginType>
AbstractPluginManager<PluginType>::AbstractPluginManager(
    const std::string &param_name,
    const loadPluginFunction &loadPlugin,
    const initPluginFunction &initPlugin,
    const rclcpp::Node::SharedPtr &node_handle
)
  : param_name_(param_name), loadPlugin_(loadPlugin), initPlugin_(initPlugin), node_handle_(node_handle)
{
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::loadPlugins()
{
  std::map<std::string, rclcpp::Parameter> plugin_param_list;

  if (!node_handle_->get_parameters(param_name_, plugin_param_list))
  {
    RCLCPP_WARN(node_handle_->get_logger(),
                "No %s plugins configured! - Use the param \"%s\", which must be a list of tuples with a name and a "
                "type.",
                param_name_.c_str(), param_name_.c_str());
    return false;
  }

  try
  {
    for (int i = 0; i < plugin_param_list.size(); i++)
    {
      rclcpp::Parameter elem = plugin_param_list[i];

      std::string name = elem.get_name();
      std::string type = elem.get_type_name();

      if (plugins_.find(name) != plugins_.end())
      {
        RCLCPP_ERROR(node_handle_->get_logger(), "The plugin \"%s\" has already been loaded! Names must be unique!",
                     name.c_str());
        return false;
      }
      typename PluginType::Ptr plugin_ptr = loadPlugin_(type);
      if(plugin_ptr && initPlugin_(name, plugin_ptr))
      {

        plugins_.insert(
            std::pair<std::string, typename PluginType::Ptr>(name, plugin_ptr));

        plugins_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping
        names_.push_back(name);

        RCLCPP_INFO(node_handle_->get_logger(),
                   "The plugin with the type \"%s\" has been loaded successfully under the name \"%s\".", type.c_str(),
                   name.c_str());
      }
      else
      {
        RCLCPP_ERROR(node_handle_->get_logger(), "Could not load the plugin with the name \"%s\" and the type \"%s\"!",
                     name.c_str(), type.c_str());
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_handle_->get_logger(),
                 "Invalid parameter structure. The \"%s\" parameter has to be a list of structs with fields \"name\" "
                 "and \"type\" of !",
                 param_name_.c_str());
    RCLCPP_ERROR(node_handle_->get_logger(), "%s", e.what());
    return false;
  }
  // is there any plugin in the map?
  return plugins_.empty() ? false : true;
}

template <typename PluginType>
const std::vector<std::string>& AbstractPluginManager<PluginType>::getLoadedNames()
{
  return names_;
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::hasPlugin(const std::string &name)
{
  return static_cast<bool>(plugins_.count(name)); // returns 1 or 0;
}

template <typename PluginType>
std::string AbstractPluginManager<PluginType>::getType(const std::string &name)
{
  std::map<std::string, std::string>::iterator iter = plugins_type_.find(name);
  return iter->second;
}


template <typename PluginType>
typename PluginType::Ptr AbstractPluginManager<PluginType>::getPlugin(const std::string &name)
{
  typename std::map<std::string, typename PluginType::Ptr>::iterator new_plugin
      = plugins_.find(name);
  if(new_plugin != plugins_.end())
  {
    RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "Found plugin with the name \"" << name << "\".");
    return new_plugin->second;
  }
  else
  {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "The plugin with the name \"" << name << "\" has not yet been loaded!");
    return typename PluginType::Ptr(); // return null ptr
  }
}

template <typename PluginType>
void AbstractPluginManager<PluginType>::clearPlugins() {
  plugins_.clear();
  plugins_type_.clear();
  names_.clear();
}

} /* namespace mbf_abstract_nav */

#endif //MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_

