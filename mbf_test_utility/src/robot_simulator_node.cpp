#include "robot_simulator.cpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  mbf_simple_nav::RobotSimulator::SharedPtr sim_node = std::make_shared<mbf_simple_nav::RobotSimulator>();
  rclcpp::spin(sim_node);
  return EXIT_SUCCESS;
}
