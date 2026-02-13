#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_msgs/srv/dice_identification.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using DiceIdentificationSrv = easy_motion_msgs::srv::DiceIdentification;


class DiceIdentification : public BT::RosServiceNode<DiceIdentificationSrv>
{
public:
  explicit DiceIdentification(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosServiceNode<DiceIdentificationSrv>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::OutputPort<int>("face_number"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
    }
    );
  }

  bool setRequest(Request::SharedPtr & request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

};
