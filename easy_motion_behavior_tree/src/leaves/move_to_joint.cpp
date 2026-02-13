#include <easy_motion_behavior_tree/leaves/move_to_joint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/utils/moveit_error_code.h>


MoveToJoint::MoveToJoint(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<easy_motion_msgs::action::MoveToJoint>(name, conf, params)
{
}

bool MoveToJoint::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "MoveToJoint ticked.");

  auto joint_target = getInput<std::vector<double>>("joint_target");

  if (joint_target) {
    goal.joint_target = joint_target.value();
    return true;
  }
  return false;
}

BT::NodeStatus MoveToJoint::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived", name().c_str());
  RCLCPP_INFO(node_.lock()->get_logger(), "%s Result: %d", name().c_str(), (wr.result->result).val);
  int code = wr.result->result.val;

  if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    moveit::core::MoveItErrorCode error(code);  // wrap
    RCLCPP_INFO(
      node_.lock()->get_logger(), "%s failed with error code: %d (%s)",
      name().c_str(), code, moveit::core::error_code_to_string(error).c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveToJoint::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(),
    toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToJoint::onFeedback(
  const std::shared_ptr<const easy_motion_msgs::action::MoveToJoint::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToJoint will self register with name  "MoveToJoint".
CreateRosNodePlugin(MoveToJoint, "MoveToJoint");
