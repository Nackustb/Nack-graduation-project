#include "ur5e_gripper_control/single_ur5e_gripper.h"

SingleUR5eGripper::SingleUR5eGripper(const rclcpp::NodeOptions & options)
: rclcpp::Node("single_ur5e_gripper", options) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SingleUR5eGripper::init() {
    // 初始化 gripper action client
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);

    // 初始化 move_group
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    // 获取参数中的目标位姿字符串
    this->get_parameter("target_pose_str_list", target_pose_str_list_);
    str_list_2_double_list(target_pose_str_list_, target_pose_list_);
}

bool SingleUR5eGripper::plan_and_execute(const std::vector<double> & target_pose) {
    std::vector<double> joint_target_positions;
    get_joint_target_positions(move_group_, target_pose, "base_link", joint_target_positions);

    if (joint_target_positions.empty()) return false;

    move_group_->setJointValueTarget(joint_target_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
}

bool SingleUR5eGripper::grasp(double gripper_position) {
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
        return false;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = gripper_position;
    goal_msg.command.max_effort = 1.0;

    send_goal_options_.goal_response_callback = std::bind(&SingleUR5eGripper::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&SingleUR5eGripper::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&SingleUR5eGripper::result_callback, this, std::placeholders::_1);

    gripper_action_client_->async_send_goal(goal_msg, send_goal_options_);
    return true;
}

void SingleUR5eGripper::get_target_pose_list(std::vector<std::vector<double>> & target_pose_list) {
    target_pose_list = target_pose_list_;
}

void SingleUR5eGripper::get_cube_pose(const std::string & from_frame,
                                      const std::string & to_frame,
                                      std::vector<double> & cube_pose) {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    cube_pose.resize(6);
    cube_pose[0] = transformStamped.transform.translation.x;
    cube_pose[1] = transformStamped.transform.translation.y;
    cube_pose[2] = transformStamped.transform.translation.z;

    tf2::Quaternion quat(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    cube_pose[3] = roll;
    cube_pose[4] = pitch;
    cube_pose[5] = yaw;
}

void SingleUR5eGripper::go_to_ready_position() {
    move_group_->setNamedTarget("ready");
    move_group_->move();
}

void SingleUR5eGripper::str_list_2_double_list(const std::vector<std::string> & str_list,
                                               std::vector<std::vector<double>> & double_list) {
    for (const auto & str : str_list) {
        std::vector<double> values;
        std::stringstream ss(str);
        double value;
        while (ss >> value) {
            values.push_back(value);
        }
        double_list.push_back(values);
    }
}

void SingleUR5eGripper::goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the gripper action server.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the gripper action server.");
    }
}

void SingleUR5eGripper::feedback_callback(GoalHandleGripperCommand::SharedPtr,
                                          const std::shared_ptr<const GripperCommand::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Gripper position: %f", feedback->position);
}

void SingleUR5eGripper::result_callback(const GoalHandleGripperCommand::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Gripper action succeeded.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Gripper action was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Gripper action was canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
    }
}

