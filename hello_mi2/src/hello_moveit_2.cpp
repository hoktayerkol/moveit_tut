#include <memory>
#include <cstdio>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char ** argv)
{
  

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>("hello_moveit", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;   /***************/
  auto move_group_interface = MoveGroupInterface(node, "manipulator");  /***************/

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
  auto plan_it = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;   /***************/
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));    /***************/
    return std::make_pair(ok, msg);
  };

  move_group_interface.setPoseTarget(target_pose(0.28, 0.5, 0.5));    /***************/
  auto [success, plan] = plan_it();
  if (success){
    move_group_interface.execute(plan);   /***************/
  } else {
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
      move_group_interface.execute(plan);    
      RCLCPP_INFO(logger, "execution successful!");  
    } else {
      RCLCPP_ERROR(logger, "planning failed!");
    }
    
  }
  



  rclcpp::shutdown();


  return 0;
}
