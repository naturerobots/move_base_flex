#include <memory>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <mbf_abstract_nav/abstract_plugin_manager.h>

using namespace std::placeholders;
using testing::ElementsAre;

struct TestPlugin {
  using Ptr = std::shared_ptr<TestPlugin>;

  TestPlugin(const std::string& plugin_type) 
    : plugin_type_(plugin_type)
    , plugin_name_("not initialized")
    , is_initialized_(false)
  {};

  std::string plugin_type_;
  std::string plugin_name_;
  bool is_initialized_;
};

struct PluginManagerTest : public testing::Test
{
protected:
  using PluginManagerType = mbf_abstract_nav::AbstractPluginManager<TestPlugin>;

  // define callbacks
  PluginManagerType::loadPluginFunction loadTestPlugin_ = [](const std::string& plugin_type) {
    return std::make_shared<TestPlugin>(plugin_type);
  };
  PluginManagerType::initPluginFunction initTestPlugin_ = [](const std::string& name, const TestPlugin::Ptr& plugin_ptr) {
    plugin_ptr->plugin_name_ = name;
    plugin_ptr->is_initialized_ = true;
    return true;
  };

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via launch file for example)
  void initNodeAndPluginManager(const rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) {
    node_ptr_ = std::make_shared<rclcpp::Node>("plugin_manager_test_node", "namespace", nodeOptions);
    plugin_manager_ptr_ = std::make_shared<PluginManagerType>(
      "test_plugins", loadTestPlugin_, initTestPlugin_, node_ptr_);
  }

  void TearDown() override {
    rclcpp::shutdown();
    plugin_manager_ptr_.reset();
    node_ptr_.reset();
  }

  std::shared_ptr<PluginManagerType> plugin_manager_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(PluginManagerTest, loadsNoPluginsWithDefaultConfig)
{
  initNodeAndPluginManager();
  EXPECT_EQ(plugin_manager_ptr_->loadPlugins(), false);
}

TEST_F(PluginManagerTest, throwsWhenAPluginIsMissingItsType)
{
  const std::vector<std::string> plugin_names{"myPlugin1"};
  EXPECT_THROW(
    initNodeAndPluginManager(rclcpp::NodeOptions()
      .append_parameter_override("test_plugins", plugin_names)),
    rclcpp::ParameterTypeException);
}

TEST_F(PluginManagerTest, throwsWhenAPluginIsMissingItsTypeMultiplePlugins)
{
  const std::vector<std::string> plugin_names{"pluginWithType", "pluginWithoutType"};
  EXPECT_THROW(
    initNodeAndPluginManager(rclcpp::NodeOptions()
      .append_parameter_override("test_plugins", plugin_names)
      .append_parameter_override("pluginWithType.type", "TestPlugin")),
    rclcpp::ParameterTypeException);
}

TEST_F(PluginManagerTest, populatesPluginNameToTypeMap)
{
  const std::vector<std::string> plugin_names{"plugin1", "plugin2", "plugin3"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("test_plugins", plugin_names)
    .append_parameter_override("plugin1.type", "TestPlugin")
    .append_parameter_override("plugin2.type", "SomeOtherTestPlugin")
    .append_parameter_override("plugin3.type", "TestPlugin") // same plugin type for different plugins is allowed
  );
  EXPECT_EQ(plugin_manager_ptr_->getType("plugin1"), "TestPlugin");
  EXPECT_EQ(plugin_manager_ptr_->getType("plugin2"), "SomeOtherTestPlugin");
  EXPECT_EQ(plugin_manager_ptr_->getType("plugin3"), "TestPlugin");
}

TEST_F(PluginManagerTest, doesNotShowNamesOfUnloadedPlugins)
{
  const std::vector<std::string> plugin_names{"plugin1"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("test_plugins", plugin_names)
    .append_parameter_override("plugin1.type", "TestPlugin")
  );
  EXPECT_EQ(plugin_manager_ptr_->getLoadedNames().size(), 0);
}

TEST_F(PluginManagerTest, loadsPlugins)
{
  const std::vector<std::string> plugin_names{"plugin1", "plugin2", "plugin3"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("test_plugins", plugin_names)
    .append_parameter_override("plugin1.type", "TestPlugin")
    .append_parameter_override("plugin2.type", "SomeOtherTestPlugin")
    .append_parameter_override("plugin3.type", "TestPlugin")
  );

  ASSERT_EQ(plugin_manager_ptr_->loadPlugins(), true);
  EXPECT_THAT(plugin_manager_ptr_->getLoadedNames(), ElementsAre("plugin1", "plugin2", "plugin3"));
}

TEST_F(PluginManagerTest, initializesPlugins)
{
  const std::vector<std::string> plugin_names{"plugin1", "plugin2", "plugin3"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("test_plugins", plugin_names)
    .append_parameter_override("plugin1.type", "TestPlugin")
    .append_parameter_override("plugin2.type", "SomeOtherTestPlugin")
    .append_parameter_override("plugin3.type", "TestPlugin")
  );
  ASSERT_EQ(plugin_manager_ptr_->loadPlugins(), true);

  EXPECT_TRUE(plugin_manager_ptr_->getPlugin("plugin1")->is_initialized_);
  EXPECT_TRUE(plugin_manager_ptr_->getPlugin("plugin2")->is_initialized_);
  EXPECT_TRUE(plugin_manager_ptr_->getPlugin("plugin3")->is_initialized_);
  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin1")->plugin_name_, "plugin1");
  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin2")->plugin_name_, "plugin2");
  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin3")->plugin_name_, "plugin3");
}

TEST_F(PluginManagerTest, getPluginInstances)
{
  const std::vector<std::string> plugin_names{"plugin1", "plugin2", "plugin3"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("test_plugins", plugin_names)
    .append_parameter_override("plugin1.type", "TestPlugin1")
    .append_parameter_override("plugin2.type", "TestPlugin2")
    .append_parameter_override("plugin3.type", "TestPlugin3")
  );
  ASSERT_EQ(plugin_manager_ptr_->loadPlugins(), true);

  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin1")->plugin_type_, "TestPlugin1");
  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin2")->plugin_type_, "TestPlugin2");
  EXPECT_EQ(plugin_manager_ptr_->getPlugin("plugin3")->plugin_type_, "TestPlugin3");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}