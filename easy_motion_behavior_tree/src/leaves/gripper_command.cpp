#include <easy_motion_behavior_tree/leaves/gripper_command.hpp>

GripperCommand::GripperCommand(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<control_msgs::action::GripperCommand>(name, conf, params)
{
}

bool GripperCommand::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "GripperCommand ticked.");

  // Required fields: position, max_effort
  double position;
  if (!getInput("position", position)) {
    throw BT::RuntimeError("Missing parameter [position]");
  }
  double max_effort;
  if (!getInput("max_effort", max_effort)) {
    throw BT::RuntimeError("Missing parameter [max_effort]");
  }

  goal.command.position = position;
  goal.command.max_effort = max_effort;
  return true;
}

BT::NodeStatus GripperCommand::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived.", name().c_str());
  if (wr.result->reached_goal) {
    return BT::NodeStatus::SUCCESS;
  } else if (wr.result->stalled) {
    RCLCPP_INFO(node_.lock()->get_logger(), "Gripper stalled");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus GripperCommand::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(),
    toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperCommand::onFeedback(
  const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class GripperCommand will self register with name  "GripperCommand".
CreateRosNodePlugin(GripperCommand, "GripperCommand");
