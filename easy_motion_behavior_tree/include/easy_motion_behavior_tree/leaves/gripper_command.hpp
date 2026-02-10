#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <control_msgs/action/gripper_command.hpp>


class GripperCommand : public BT::RosActionNode<control_msgs::action::GripperCommand>
{
public:
  GripperCommand(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<double>("position"),
      BT::InputPort<double>("max_effort"),
    }
    );
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback) override;

};
