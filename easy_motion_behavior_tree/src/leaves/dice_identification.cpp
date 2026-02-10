#include <easy_motion_behavior_tree/leaves/dice_identification.hpp>

bool DiceIdentification::setRequest(Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus DiceIdentification::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "DiceIdentification service responce received.");

  if(response->success)
  {
    // Set output ports
    setOutput("face_number", response->face_number);
    setOutput("pose", response->pose);
    RCLCPP_INFO(logger(), "DiceIdentification service succeeded.");
    RCLCPP_INFO(logger(), "Face number: %d", response->face_number);
    RCLCPP_INFO(logger(), "Pose: [%f, %f, %f, %f, %f, %f, %f]",
                response->pose.pose.position.x,
                response->pose.pose.position.y,
                response->pose.pose.position.z,
                response->pose.pose.orientation.x,
                response->pose.pose.orientation.y,
                response->pose.pose.orientation.z,
                response->pose.pose.orientation.w);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "DiceIdentification service response is failed.");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus DiceIdentification::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "DiceIdentification error: %d", error);
  return BT::NodeStatus::FAILURE;
}


// Plugin registration.
// The class DiceIdentification will self register with name  "DiceIdentification".
CreateRosNodePlugin(DiceIdentification, "DiceIdentification");
