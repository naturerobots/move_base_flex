#include <memory>
#include <functional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// dummy message
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_utility/robot_information.h>

#include <mbf_abstract_nav/abstract_action_base.hpp>
#include <mbf_abstract_nav/abstract_execution_base.h>

using namespace mbf_abstract_nav;

// mocked version of an execution
struct MockedExecution : public AbstractExecutionBase {
  typedef std::shared_ptr<MockedExecution> Ptr;

  MockedExecution(const mbf_utility::RobotInformation::ConstPtr& ri, const rclcpp::Node::SharedPtr& node) : AbstractExecutionBase("mocked_execution", ri, node) {}

  MOCK_METHOD(bool, cancel, (), (override));
};

using testing::Return;
using testing::Test;

// fixture with access to the AbstractActionBase's internals
struct AbstractActionBaseFixture
    : public AbstractActionBase<mbf_msgs::action::GetPath, MockedExecution>,
      public Test {
  TFPtr tf_;
  mbf_utility::RobotInformation::Ptr ri_;

  AbstractActionBaseFixture()
  : AbstractActionBase(nullptr, "action_base", nullptr) {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void initRosNode()
  {
    node_ = std::shared_ptr<rclcpp::Node>(new rclcpp::Node("test_node"));
    tf_ = std::make_shared<TF>(node_->get_clock());
    ri_ = std::make_shared<mbf_utility::RobotInformation>(node_, tf_, "global_frame", "local_frame", rclcpp::Duration(0,0));
  }

  void runImpl(const GoalHandlePtr &goal_handle, MockedExecution &execution) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50)); // runs in action thread(s)
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
};

TEST_F(AbstractActionBaseFixture, cancelAll)
{
  initRosNode();

  // spawn a bunch of threads
  for (unsigned char slot = 0; slot != 10; ++slot) {
    concurrency_slots_[slot].execution = std::make_shared<MockedExecution>(ri_, node_);
    // set the expectation
    EXPECT_CALL(*concurrency_slots_[slot].execution, cancel()).WillRepeatedly(Return(true));

    // set the in_use flag --> this should turn to false
    concurrency_slots_[slot].in_use = true;
    concurrency_slots_[slot].thread_ptr = new std::thread(
        std::bind(&AbstractActionBaseFixture::run, this, std::ref(concurrency_slots_[slot])));
  }

  // cancel all of slots
  cancelAll();

  // check the result
  for (ConcurrencyMap::iterator slot = concurrency_slots_.begin();
       slot != concurrency_slots_.end(); ++slot)
    ASSERT_FALSE(slot->second.in_use);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}