#include <cmath>  // For M_PI and sin/cos
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

std::vector<geometry_msgs::msg::Pose> computeCircularPath(
    const geometry_msgs::msg::Pose &center, double radius, int num_points)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 0; i < num_points; ++i)
    {
        double angle = (static_cast<double>(i) / num_points) * 2 * M_PI; // Angle from 0 to 2π
        geometry_msgs::msg::Pose point;
        point.orientation = center.orientation; // Keep orientation fixed
        point.position.x = center.position.x + radius * cos(angle);
        point.position.y = center.position.y + radius * sin(angle);
        point.position.z = center.position.z; // z remains constant

        waypoints.push_back(point);
    }

    return waypoints;
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // Create a publisher for visualization markers
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);

    // Define initial pose and final pose
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.orientation.w = 1.0;
    initial_pose.position.x = 0.5;
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 1.0;

    geometry_msgs::msg::Pose final_pose;
    final_pose.orientation.w = 1.0;
    final_pose.position.x = 0.7;
    final_pose.position.y = 0.2;
    final_pose.position.z = 1.0;

    // Generate waypoints for linear path
    std::vector<geometry_msgs::msg::Pose> linear_waypoints;

    // Number of intermediate points (excluding start and end points)
    int num_points = 10;

    // Interpolate between initial and final points to generate intermediate poses
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points; // Interpolation parameter from 0 to 1

        geometry_msgs::msg::Pose interpolated_pose;
        interpolated_pose.orientation = initial_pose.orientation; // Keep orientation fixed
        interpolated_pose.position.x = initial_pose.position.x + t * (final_pose.position.x - initial_pose.position.x);
        interpolated_pose.position.y = initial_pose.position.y + t * (final_pose.position.y - initial_pose.position.y);
        interpolated_pose.position.z = initial_pose.position.z; // z remains constant

        linear_waypoints.push_back(interpolated_pose);
    }

    // Generate waypoints for circular path
    geometry_msgs::msg::Pose circle_center;
    circle_center.orientation.w = 1.0; // Keep orientation fixed
    circle_center.position.x = 0.6; // Center of the circle
    circle_center.position.y = 0.0;
    circle_center.position.z = 1.0;
    double radius = 0.1;
    int min_circle_points = 10; // Minimum number of points
    auto circular_waypoints = computeCircularPath(circle_center, radius, min_circle_points);

    // Create a marker array for linear waypoints
    visualization_msgs::msg::MarkerArray linear_marker_array;

    // Add linear waypoints markers
    for (size_t i = 0; i < linear_waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_0"; // Change to your robot's base frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "linear_waypoints";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = linear_waypoints[i];
        marker.scale.x = 0.05; // Size of the sphere
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f; // Green color
        marker.color.b = 0.0f;
        marker.color.a = 1.0f; // Opaque

        linear_marker_array.markers.push_back(marker);
    }

    // Publish the linear markers
    marker_pub->publish(linear_marker_array);

    // Plan and execute the linear Cartesian path
    moveit_msgs::msg::RobotTrajectory linear_trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        linear_waypoints, 0.01, 0.0, linear_trajectory); // Compute linear Cartesian path

    if (fraction > 0.9) {  // At least 90% of the path should be feasible
        RCLCPP_INFO(logger, "Linear Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);

        // Create a plan from the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan linear_plan;
        linear_plan.trajectory_ = linear_trajectory;

        // Execute the linear plan
        move_group_interface.execute(linear_plan);
    } else {
        RCLCPP_ERROR(logger, "Failed to compute linear Cartesian path (%.2f%% achieved)", fraction * 100.0);
    }

    // Clear linear markers by setting action to DELETE
    visualization_msgs::msg::MarkerArray clear_marker_array;
    for (std::vector<geometry_msgs::msg::Pose>::size_type i = 0; i < linear_waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_0"; // Same frame as before
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "linear_waypoints";
        marker.id = i; // Same ID as before
        marker.action = visualization_msgs::msg::Marker::DELETE; // Set action to DELETE

        clear_marker_array.markers.push_back(marker);
    }

    // Publish the clear marker array
    marker_pub->publish(clear_marker_array);

    // Create a marker array for circular waypoints
    visualization_msgs::msg::MarkerArray circular_marker_array;

    // Add circular waypoints markers
    for (size_t i = 0; i < circular_waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_0"; // Change to your robot's base frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "circular_waypoints";
        marker.id = static_cast<int>(i); // Unique ID for circular markers
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = circular_waypoints[i];
        marker.scale.x = 0.05; // Size of the sphere
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0f; // Blue color
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f; // Opaque

        circular_marker_array.markers.push_back(marker);
    }

    // Publish the circular markers
    marker_pub->publish(circular_marker_array);

    // Plan and execute the circular path
    moveit_msgs::msg::RobotTrajectory circular_trajectory;
    fraction = move_group_interface.computeCartesianPath(
        circular_waypoints, 0.01, 0.0, circular_trajectory); // Compute circular Cartesian path

    if (fraction > 0.9) {  // At least 90% of the path should be feasible
        RCLCPP_INFO(logger, "Circular Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);

        // Create a plan from the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan circular_plan;
        circular_plan.trajectory_ = circular_trajectory;

        // Execute the circular plan
        move_group_interface.execute(circular_plan);
    } else {
        RCLCPP_ERROR(logger, "Failed to compute circular Cartesian path (%.2f%% achieved)", fraction * 100.0);
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

