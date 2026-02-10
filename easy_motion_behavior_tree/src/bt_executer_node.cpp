#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/bt_utils.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("bt_executor_node", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::vector<std::string> plugins, ros_plugins;
  std::string bt_xml_file;
  std::string bt_package;

  node->declare_parameter("plugins", plugins);
  node->declare_parameter("ros_plugins", ros_plugins);
  node->declare_parameter("bt_package", bt_package);
  node->declare_parameter("bt_xml_file", bt_xml_file);

  node->get_parameter("plugins", plugins);
  node->get_parameter("ros_plugins", ros_plugins);
  node->get_parameter("bt_package", bt_package);
  node->get_parameter("bt_xml_file", bt_xml_file);


  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  BT::RosNodeParams params;
  params.nh = node;
  params.server_timeout = std::chrono::milliseconds(10000);
  params.wait_for_server_timeout = std::chrono::milliseconds(10000);

  for (const auto & plugin : plugins) {
    RCLCPP_INFO(node->get_logger(), "Loading BT Node: [%s]", plugin.c_str());
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  for (const auto & ros_plugin : ros_plugins) {
    RCLCPP_INFO(node->get_logger(), "Loading BT Node: [%s]", ros_plugin.c_str());
    RegisterRosNode(factory, loader.getOSName(ros_plugin), params);
  }

  std::string pkgpath = ament_index_cpp::get_package_share_directory(bt_package);
  std::string xml_file = pkgpath + "/trees/" + bt_xml_file;

  RCLCPP_INFO(node->get_logger(), "Loading BT: [%s]", xml_file.c_str());

  BT::Tree tree = factory.createTreeFromFile(xml_file);

  BT::Groot2Publisher publisher(tree);

  RCLCPP_INFO_STREAM(node->get_logger(), "Behavior tree succesfully created!");

  RCLCPP_INFO_STREAM(node->get_logger(), "Starting the execution of the behavior tree..");

  rclcpp::Rate rate(50);
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    executor.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
