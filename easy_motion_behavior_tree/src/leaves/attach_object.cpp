#include <easy_motion_behavior_tree/leaves/attach_object.hpp>

bool AttachObject::setRequest(Request::SharedPtr & request)
{
  std::string object_id;
  if (!getInput("object_id", object_id)) {
    throw BT::RuntimeError("Missing parameter [object_id]");
  }
  std::string target_frame_id;
  if (!getInput("target_frame_id", target_frame_id)) {
    throw BT::RuntimeError("Missing parameter [target_frame_id]");
  }
  request->object_id = object_id;
  request->target_frame_id = target_frame_id;
  return true;
}

BT::NodeStatus AttachObject::onResponseReceived(const Response::SharedPtr & response)
{
  RCLCPP_INFO(logger(), "AttachObject service responce received.");
  if (response->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus AttachObject::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "AttachObject error: %d", error);
  return BT::NodeStatus::FAILURE;
}


// Plugin registration.
// The class AttachObject will self register with name  "AttachObject".
CreateRosNodePlugin(AttachObject, "AttachObject");
