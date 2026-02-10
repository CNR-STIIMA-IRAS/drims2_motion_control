#include <easy_motion_behavior_tree/leaves/move_to_pose.hpp>
#include <moveit/utils/moveit_error_code.h>

MoveToPose::MoveToPose(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<easy_motion_msgs::action::MoveToPose>(name, conf, params)
{
}

bool MoveToPose::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "MoveToPose ticked.");

  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target");
  if (pose_target) {
    goal.pose_target = pose_target.value();
    return true;
  }
  bool cartesian_motion = false;
  if (!getInput("cartesian_motion", cartesian_motion)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [cartesian_motion], set it as false by default");
  }

  // If pose_target is not set, build it from components
  geometry_msgs::msg::PoseStamped pose;

  // Required fields: frame_id, position (3), orientation (4)
  if (!getInput("frame_id", pose.header.frame_id)) {
    throw BT::RuntimeError("Missing parameter [frame_id]");
  }

  std::vector<double> position;
  if (!getInput("position", position) || position.size() != 3) {
    throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
  }

  std::vector<double> orientation;
  if (!getInput("orientation", orientation) || orientation.size() != 4) {
    throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
  }

  pose.header.stamp = node_.lock()->now();  // timestamp current time
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation[0];
  pose.pose.orientation.y = orientation[1];
  pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];

  goal.pose_target = pose;
  goal.cartesian_motion = cartesian_motion;

  return true;
}

BT::NodeStatus MoveToPose::onResultReceived(const RosActionNode::WrappedResult & wr)
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

BT::NodeStatus MoveToPose::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(),
    toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToPose::onFeedback(
  const std::shared_ptr<const easy_motion_msgs::action::MoveToPose::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToPose will self register with name  "MoveToPose".
CreateRosNodePlugin(MoveToPose, "MoveToPose");
