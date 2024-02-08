#include <mbf_simple_core/simple_recovery.h>

namespace mbf_simple_nav
{

//! Recovery plugin for testing move base flex
class TestRecovery : public mbf_simple_core::SimpleRecovery
{
public:
  TestRecovery() = default;
  virtual ~TestRecovery() = default;

  virtual void initialize(
    const std::string name, TF * tf,
    const rclcpp::Node::SharedPtr & node_handle) {}

  virtual uint32_t runBehavior(std::string & message) {}

  virtual bool cancel() {}
};

}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestRecovery, mbf_simple_core::SimpleRecovery);
