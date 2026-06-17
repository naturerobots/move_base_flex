#include <mbf_simple_core/simple_plan_refiner.h>
#include <mbf_msgs/action/refine_path.hpp>

namespace mbf_simple_nav
{

//! Plan refiner plugin for testing move base flex
class TestPlanRefiner : public mbf_simple_core::SimplePlanRefiner
{
  //! Returns a refined plan that is equal to the input plan
  virtual uint32_t applyRefinement(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & refined_plan,
    float & position_error,
    float & orientation_error,
    float & path_length_ratio,
    std::string & message) override
  {
    refined_plan = plan;
    position_error = 0.0f;
    orientation_error = 0.0f;
    path_length_ratio = 1.0f;
    return mbf_msgs::action::RefinePath::Result::SUCCESS;
  }

  virtual bool cancel() override {return true;}
  virtual void initialize(
    const std::string name,
    const rclcpp::Node::SharedPtr & node_handle) override {}
};

}  // namespace mbf_simple_nav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mbf_simple_nav::TestPlanRefiner, mbf_simple_core::SimplePlanRefiner);
