#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_msgs/srv/detach_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using DetachObjectSrv = easy_motion_msgs::srv::DetachObject;


class DetachObject : public BT::RosServiceNode<DetachObjectSrv>
{
public:
  explicit DetachObject(
      const std::string & name,
      const BT::NodeConfig & conf,
      const BT::RosNodeParams & params): BT::RosServiceNode<DetachObjectSrv>(name, conf, params)
      {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<std::string>("object_id")
    }
    );
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

};
