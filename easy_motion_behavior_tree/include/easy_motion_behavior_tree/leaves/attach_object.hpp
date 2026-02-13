#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_msgs/srv/attach_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using AttachObjectSrv = easy_motion_msgs::srv::AttachObject;


class AttachObject : public BT::RosServiceNode<AttachObjectSrv>
{
public:
  explicit AttachObject(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosServiceNode<AttachObjectSrv>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<std::string>("object_id"),
      BT::InputPort<std::string>("target_frame_id")
    }
    );
  }

  bool setRequest(Request::SharedPtr & request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

};
