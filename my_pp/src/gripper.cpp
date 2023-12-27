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


    const auto& arm_group_name = "panda_arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "panda_hand";


    static const std::string PLANNING_GROUP = "hand";
    //create move group ang planning groups
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // get the current states of the jounts
    // not needed here
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // set hand target joint values
    move_group.setJointValueTarget(std::vector<double>{0.00,0.00});

    // create a plan object to store generated plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // generate a plan
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // check if the plan is succesfull
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // execute the plan
    move_group.execute(my_plan);




    rclcpp::shutdown();
    return 0;
}