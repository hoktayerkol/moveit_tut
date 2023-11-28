#include <memory>
#include <cstdio>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <thread>
//#include <eig3.hpp>

int main(int argc, char ** argv)
{
  

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>("hello_moveit", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;   /***************/
  auto move_group_interface = MoveGroupInterface(node, "manipulator");  /***************/

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  
  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text){
    auto const text_pose = []{
      auto  msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // text pose 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  //This prompt function blocks your program until the user presses the next button in RViz.
  auto const prompt = [&moveit_visual_tools](auto text){
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path = 
    [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator")](
        auto const trajectory){
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

  // Set a target Pose
  auto target_pose = [](float x, float y, float z){
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
  };

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger(); //after each change
  auto plan_it = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  };

  move_group_interface.setPoseTarget(target_pose(0.28, 0.5, 0.5));
  auto [success, plan] = plan_it();
  if (success){
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "planning failed!");
  }

  float h;
  for (int i=0; i<5; i++){

    int l;
    std::cout<<"enter a number to continue! \n";
    std::cin >> l;

    if (h == 0.0)
      h=0.28;
    else
      h=0.0;

    move_group_interface.setPoseTarget(target_pose(h, 0.5, 0.5));
    auto [success, plan] = plan_it();

    if (success){
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);    
      RCLCPP_INFO(logger, "execution successful!");  
    } else {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "planning failed!");
    }
    
  }
  



  rclcpp::shutdown();
  spinner.join();

  return 0;
}
