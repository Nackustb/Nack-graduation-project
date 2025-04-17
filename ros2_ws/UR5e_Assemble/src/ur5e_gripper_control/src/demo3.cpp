#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/action/gripper_command.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo3");

void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
  } else {
    RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
  }
}
void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(LOGGER, "Got Feedback: Current position is %f", feedback->position);
}
void result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(LOGGER, "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(LOGGER, "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(LOGGER, "Unknown result code");
    return;
  }
  RCLCPP_INFO(LOGGER, "Goal is completed, current position is %f", result.result->position);
}

void str_list_2_double_list(const std::vector<std::string> & str_list, 
                            std::vector<std::vector<double>> & double_list){
  double_list.clear();
  for (auto & pose_str : str_list){
    std::vector<double> pose;
    std::stringstream ss(pose_str);
    std::string token;
    while (std::getline(ss, token, ',')){
      pose.push_back(std::stod(token));
    }
    double_list.push_back(pose);
  }
}

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto demo3_node = rclcpp::Node::make_shared("demo3", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(demo3_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::string> target_pose_str_list = 
    demo3_node->get_parameter("target_pose_list").as_string_array();
  std::vector<std::vector<double>> target_pose_list;
  str_list_2_double_list(target_pose_str_list, target_pose_list);

  const std::string gripper_action_name = "gripper_controller/gripper_cmd";
  auto gripper_action_client = rclcpp_action::create_client<GripperCommand>(demo3_node, gripper_action_name);

  while (!gripper_action_client->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(LOGGER, *demo3_node->get_clock(), 500, "Waiting for action server to be available...");
  }

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(demo3_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  RCLCPP_INFO(LOGGER, "Reference frame: %s", move_group.getPlanningFrame().c_str());

  moveit_msgs::msg::CollisionObject collision_table;
  collision_table.header.frame_id = move_group.getPlanningFrame();
  collision_table.id = "table";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.0;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.0;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;
  collision_table.primitives.push_back(primitive);
  collision_table.primitive_poses.push_back(box_pose);
  collision_table.operation = collision_table.ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_table);
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(3.0);

  tf2::Quaternion target_quat;
  target_quat.setRPY(target_pose_list[0][3], target_pose_list[0][4], target_pose_list[0][5]);
  target_quat.normalize();
  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = "base_link";
  target_pose_stamped.pose.position.x = target_pose_list[0][0];
  target_pose_stamped.pose.position.y = target_pose_list[0][1];
  target_pose_stamped.pose.position.z = target_pose_list[0][2];
  target_pose_stamped.pose.orientation.x = target_quat.x();
  target_pose_stamped.pose.orientation.y = target_quat.y();
  target_pose_stamped.pose.orientation.z = target_quat.z();
  target_pose_stamped.pose.orientation.w = target_quat.w();

  move_group.setJointValueTarget(target_pose_stamped);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success_plan = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success_plan){
    move_group.execute(plan);
  }

  auto gripper_goal_msg = GripperCommand::Goal();
  gripper_goal_msg.command.position = 0.38;
  gripper_goal_msg.command.max_effort = 1.0;
  if(!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
    rclcpp::shutdown();
    return 0;
  }
  RCLCPP_INFO(LOGGER, "Sending gripper goal");
  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&result_callback, std::placeholders::_1);
  gripper_action_client->async_send_goal(gripper_goal_msg, send_goal_options);

  rclcpp::shutdown();
  return 0;
}
