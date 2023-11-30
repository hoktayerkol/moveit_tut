#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
      shape_msgs::msg::SolidPrimitive primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    // We define the frame/pose for this cylinder so that it appears in the gripper.
    object_to_attach.header.frame_id = move_group.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;

    // First, we add the object to the world (without using a vector).
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    RCLCPP_INFO(LOGGER, "Attach the object to the robot");
    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");
    move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

    rclcpp::shutdown();
    return 0;
}