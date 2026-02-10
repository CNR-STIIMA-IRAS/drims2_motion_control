#include <easy_motion_behavior_tree/leaves/detach_object.hpp>

bool DetachObject::setRequest(Request::SharedPtr& request)
{
  std::string object_id;
  if (!getInput("object_id", object_id)) {
    throw BT::RuntimeError("Missing parameter [object_id]");
  }
  request->object_id = object_id;
  return true;
}

BT::NodeStatus DetachObject::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "DetachObject service responce received.");
  if(response->success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus DetachObject::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "DetachObject error: %d", error);
  return BT::NodeStatus::FAILURE;
}


// Plugin registration.
// The class DetachObject will self register with name  "DetachObject".
CreateRosNodePlugin(DetachObject, "DetachObject");
