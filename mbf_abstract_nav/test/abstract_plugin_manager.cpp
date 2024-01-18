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
    node_ptr_ = std::make_shared<rclcpp::Node>("plugin_manager_test_node");
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
  EXPECT_EQ(plugin_manager_ptr_->loadPlugins(), false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// TODO fix error msg when no params are configured (it still talks about tuples)