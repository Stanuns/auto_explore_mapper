#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include <slam_toolbox/srv/detail/save_map__struct.hpp>

using namespace std::chrono_literals;
using namespace slam_toolbox;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // std::shared_ptr<rclcpp::Node> map_serializer_node = rclcpp::Node::make_shared("map_serializer_client");
  // rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr map_serializer_client =
  //   map_serializer_node->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");

  // auto serialize_pose_graph_request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

  // while (!map_serializer_client->wait_for_service(2s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the map serializer service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map serializer service not available, waiting again...");
  // }

  // serialize_pose_graph_request->filename = "~/AutoExploreMap";
  // auto result = map_serializer_client->async_send_request(serialize_pose_graph_request);


  std::shared_ptr<rclcpp::Node> map_saver_node = rclcpp::Node::make_shared("map_saver_client");
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr map_saver_client =
    map_saver_node->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

  auto save_map_request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();

  while (!map_saver_client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the SaveMap service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SaveMap service not available, waiting again...");
  }

  save_map_request->name.data = "~/AutoExploreMap";
  auto map_saver_result = map_saver_client->async_send_request(save_map_request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(map_saver_node, map_saver_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Save map Successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save map");
  }

  rclcpp::shutdown();
  return 0;
}