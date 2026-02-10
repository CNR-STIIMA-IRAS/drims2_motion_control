#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"

class CheckDiceFace : public BT::ConditionNode
{
public:
  CheckDiceFace(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("target_face"),
      BT::InputPort<int>("face")
    };
  }

  BT::NodeStatus tick() override
  {
    int target_face, face;

    if (!getInput("target_face", target_face)) {
      throw BT::RuntimeError("Missing parameter [target_face]");
    }

    if (!getInput("face", face)) {
      throw BT::RuntimeError("Missing parameter [face]");
    }

    if (target_face == face) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
};