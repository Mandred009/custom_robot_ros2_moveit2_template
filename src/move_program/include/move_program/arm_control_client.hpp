/*
 Author: Akshay Kumar Burusa
 Email: akshaykumar.burusa@wur.nl
 */

#ifndef ARM_CONTROL_CLIENT_HPP
#define ARM_CONTROL_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <random>

namespace arm_control_client
{
class ArmControlClient
{
protected:
  // ROS2 Node
  rclcpp::Node::SharedPtr node_;

  // MoveIt2 interface for motion planning
  moveit::planning_interface::MoveGroupInterface move_group_;

  // Workspace boundaries for the robot arm's end-effector
  float minx_, maxx_;
  float miny_, maxy_;
  float minz_, maxz_;

public:
  /* Constructor.
   * Input: ROS2 NodeHandle, workspace constraints in x, y, z axes.
   * Default values for workspace boundaries are set. */
  ArmControlClient(const rclcpp::Node::SharedPtr& node, 
                   const std::string& planning_group = "arm", 
                   float minx = 0.40, float maxx = 0.80, 
                   float miny = -0.40, float maxy = 0.40, 
                   float minz = 1.00, float maxz = 1.40);
  
  // Destructor
  ~ArmControlClient();

  /* Generates a random pose within the specified workspace boundaries.
   * Output: A randomly generated end-effector goal pose. */
  geometry_msgs::msg::Pose GetRandomPose();

  /* Moves the arm to the specified goal pose using MoveIt2.
   * Input: The target pose for the end-effector.
   * Output: (boolean) Success status of the movement. */
  bool MoveArmToGoal(const geometry_msgs::msg::Pose& goal_pose);
};
}  // namespace arm_control_client

#endif  // ARM_CONTROL_CLIENT_HPP
