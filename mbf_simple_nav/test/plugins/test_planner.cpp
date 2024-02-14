#include <mbf_simple_core/simple_planner.h>
#include <mbf_msgs/action/get_path.hpp>

namespace mbf_simple_nav
{

//! Planner plugin for testing move base flex
class TestPlanner : public mbf_simple_core::SimplePlanner
{
  //! Returns a plan that consists only of the start pose and the goal pose
  virtual uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    double tolerance, std::vector<geometry_msgs::msg::PoseStamped> & plan, double & cost,
    std::string & message) override
  {
    plan.push_back(start);
    plan.push_back(goal);
    return mbf_msgs::action::GetPath::Result::SUCCESS;
  }
  virtual bool cancel() override {return true;}
  virtual void initialize(
    const std::string name,
    const rclcpp::Node::SharedPtr & node_handle) override {}
};

}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestPlanner, mbf_simple_core::SimplePlanner);
