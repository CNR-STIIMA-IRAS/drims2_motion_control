#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"


class CanTransform : public BT::ConditionNode
{
public:
  CanTransform(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // (facoltativo) puoi tenere anche il costruttore con RosNodeParams se ti serve altrove
  CanTransform(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
  : BT::ConditionNode(name, config), node_(params.nh)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("source_frame"),
      BT::InputPort<std::string>("target_frame"),
      BT::InputPort<int>("timeout_ms", 0, "0=now() / no wait")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string source_frame, target_frame;
    getInput("source_frame", source_frame);
    getInput("target_frame", target_frame);
    int timeout_ms = getInput<int>("timeout_ms").value();

    try {
      if (timeout_ms > 0) {
        auto ok = tf_buffer_->canTransform(
          target_frame, source_frame, tf2::TimePointZero,
          std::chrono::milliseconds(timeout_ms));
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      } else {
        auto ok = tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero);
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
