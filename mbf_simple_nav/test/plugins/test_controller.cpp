#include <mbf_simple_core/simple_controller.h>
#include <mbf_msgs/action/exe_path.hpp>

namespace mbf_simple_nav
{

//! Planner plugin for testing move base flex
class TestController : public mbf_simple_core::SimpleController
{
public:
  TestController() = default;
  virtual ~TestController() = default;

  virtual void initialize(const std::string name, ::TF* tf, const rclcpp::Node::SharedPtr& node_handle) override{};
  virtual bool cancel() override{};

  virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) override
  {
    plan_ = plan;
    return true;
  };

  virtual uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                           const geometry_msgs::msg::TwistStamped& velocity,
                                           geometry_msgs::msg::TwistStamped& cmd_vel, std::string& message){

  };

  virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance)
  {
    return true;
  };

protected:
  std::vector<geometry_msgs::msg::PoseStamped> plan_;
};
}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestController, mbf_simple_core::SimpleController);