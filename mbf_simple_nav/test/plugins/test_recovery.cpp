#include <mbf_simple_core/simple_recovery.h>
#include <rclcpp/rclcpp.hpp>

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
    const rclcpp::Node::SharedPtr & node_handle)
  {
    config_.sim_server = node_handle->declare_parameter<std::string>(name + ".sim_server");
    set_robot_sim_params_client_ptr_ =
      node_handle->create_client<rcl_interfaces::srv::SetParameters>(
      config_.sim_server + "/set_parameters");
  }

  virtual uint32_t runBehavior(std::string & message)
  {
    using namespace std::chrono_literals;

    if (!set_robot_sim_params_client_ptr_->wait_for_service(1s)) {
      return 150; // failure, if there is no service to talk to
    }

    rcl_interfaces::msg::Parameter unstuck_parameter;
    unstuck_parameter.name = "is_robot_stuck";
    unstuck_parameter.value.bool_value = false;
    unstuck_parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    auto set_params_request_ptr =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>(
      rcl_interfaces::build<rcl_interfaces::srv::SetParameters::Request>().parameters(
        {unstuck_parameter}));

    // call set params service to unstuck robot
    auto request_future = set_robot_sim_params_client_ptr_->async_send_request(
      set_params_request_ptr);
    if (request_future.wait_for(1s) != std::future_status::ready) {
      RCLCPP_ERROR(rclcpp::get_logger("test_recovery"), "Service call is stuck");
      return 150; // failure, if setting params service is stuck
    }

    // service completed, did we succeed to unstuck robot?
    const auto robot_unstuck_result = request_future.get()->results.at(0);
    if (robot_unstuck_result.successful) {
      return 0; // success
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("test_recovery"),
        "Recovery failed: " << robot_unstuck_result.reason);
      return 150;
    }
  }

  virtual bool cancel() override {return true;}

protected:
  //! Service client for changing RobotSimulation parameters. Used for making the robot unstuck.
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_robot_sim_params_client_ptr_;
  struct
  {
    std::string sim_server;
  } config_;
};

}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestRecovery, mbf_simple_core::SimpleRecovery);
