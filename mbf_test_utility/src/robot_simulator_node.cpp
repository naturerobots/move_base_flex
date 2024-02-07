#include "mbf_test_utility/robot_simulator.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  mbf_simple_nav::RobotSimulator::SharedPtr sim_node =
    std::make_shared<mbf_simple_nav::RobotSimulator>();
  rclcpp::spin(sim_node);
  return EXIT_SUCCESS;
}
