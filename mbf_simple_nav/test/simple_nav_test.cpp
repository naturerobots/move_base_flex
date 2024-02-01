#include <optional>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <mbf_simple_nav/simple_navigation_server.h>

using namespace ::testing;

struct SimpleNavTest : public Test
{
protected:
  SimpleNavTest()
    : default_node_options_(
          rclcpp::NodeOptions()
              .append_parameter_override("planners", std::vector<std::string>{ "planner" })
              .append_parameter_override("planner.type", "mbf_simple_nav/TestPlanner")
              .append_parameter_override("controllers", std::vector<std::string>{ "controller" })
              .append_parameter_override("controller.type", "mbf_simple_nav/TestController")
              .append_parameter_override("recovery_behaviors", std::vector<std::string>{ "recovery" })
              .append_parameter_override("recovery.type", "mbf_simple_nav/TestRecovery"))
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via
  // launch file for example)
  void initNodeAndMeshMap(std::optional<rclcpp::NodeOptions> node_options)
  {
    node_ptr_ = std::make_shared<rclcpp::Node>("simple_nav", "", node_options.value_or(default_node_options_));
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    nav_server_ptr_ = std::make_shared<mbf_simple_nav::SimpleNavigationServer>(tf_buffer_ptr_, node_ptr_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    nav_server_ptr_.reset();
    tf_buffer_ptr_.reset();
    node_ptr_.reset();
  }

  std::shared_ptr<mbf_simple_nav::SimpleNavigationServer> nav_server_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
  const rclcpp::NodeOptions default_node_options_;
};

TEST_F(SimpleNavTest, loadsPluginsFromConfig)
{
  initNodeAndMeshMap({});
  ASSERT_TRUE(false);
}