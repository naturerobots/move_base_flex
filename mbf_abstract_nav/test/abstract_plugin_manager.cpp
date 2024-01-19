#include <memory>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <mbf_abstract_nav/abstract_plugin_manager.h>

using namespace std::placeholders;

class TestPlugin {
public:
  using Ptr = std::shared_ptr<TestPlugin>;
protected:
  std::string plugin_name_;
};

struct PluginManagerTest : public testing::Test
{
protected:
  MOCK_METHOD(TestPlugin::Ptr, loadTestPlugin, (const std::string&), (const));
  MOCK_METHOD(bool, initTestPlugins, (const std::string& name, const typename TestPlugin::Ptr& plugin_ptr), (const));

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via launch file for example)
  void initNodeAndPluginManager(const rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) {
    node_ptr_ = std::make_shared<rclcpp::Node>("plugin_manager_test_node", "namespace", nodeOptions);
    plugin_manager_ptr_ = std::make_shared<PluginManagerType>("test_plugins", 
      std::bind(&PluginManagerTest::loadTestPlugin, this, _1),
      std::bind(&PluginManagerTest::initTestPlugins, this, _1, _2),
      node_ptr_);
  }

  void TearDown() override {
    rclcpp::shutdown();
    plugin_manager_ptr_.reset();
    node_ptr_.reset();
  }

  using PluginManagerType = mbf_abstract_nav::AbstractPluginManager<TestPlugin>;
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}