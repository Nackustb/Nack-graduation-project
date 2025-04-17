#include "ur5e_gripper_control/ur5e_gripper.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

void publish_pose_marker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const std::vector<double> &pose, int id, const std::string &ns) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  tf2::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.scale.x = 0.1;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0;
  marker.color.r = (ns == "grasp_pose") ? 1.0 : 0.0;
  marker.color.g = (ns == "grasp_pose") ? 0.0 : 1.0;
  pub->publish(marker);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<UR5eGripper>(node_options);
  node->init();

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("pose_markers", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::vector<double>> target_pose_list;
  node->get_target_pose_list(target_pose_list);
  std::string from_frame = "base_link";
  std::vector<std::string> to_frame_list = {"cube1", "cube2", "cube3", "cube4", "cube5", "cube6"};

  std::vector<std::vector<double>> cube_pose_list;
  for (size_t i = 0; i < to_frame_list.size(); i++) {
    std::vector<double> cube_pose;
    node->get_cube_pose(from_frame, to_frame_list[i], cube_pose);
    if (cube_pose.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("demo4"), "Failed to get pose for %s, skipping", to_frame_list[i].c_str());
      continue;
    }

    cube_pose[0] -= 0.025;
    //cube_pose[1] = 0.0;
    cube_pose[2] += 0.14;
    cube_pose[3] = 0.0;
    cube_pose[4] = M_PI;
    cube_pose[5] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("demo4"), "Adjusted cube pose for %s: x=%f, y=%f, z=%f",
                to_frame_list[i].c_str(), cube_pose[0], cube_pose[1], cube_pose[2]);
    cube_pose_list.push_back(cube_pose);
    publish_pose_marker(marker_pub, cube_pose, i, "grasp_pose");
  }

  for (size_t i = 0; i < std::min<size_t>(6, cube_pose_list.size()); i++) {
    bool grasp_success = node->plan_and_execute(cube_pose_list[i]);
    if (!grasp_success) {
      continue;
    }
    node->grasp(0.38);
    rclcpp::sleep_for(std::chrono::seconds(1));

    if (i < target_pose_list.size()) {
      bool place_success = node->plan_and_execute(target_pose_list[i]);
      if (place_success) {
        node->grasp(0);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    node->go_to_ready_position();
  }

  rclcpp::shutdown();
  return 0;
}
