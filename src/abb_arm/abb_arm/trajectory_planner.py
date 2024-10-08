import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionClient(Node):
    # Define a class for the trajectory action client that inherits from the Node class.

    def __init__(self):
        # Initialize the action client node with a specific name.
        super().__init__('points_publisher_node_action_client')
        # Create an action client for the FollowJointTrajectory action type.
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_goal_linear(self):
        # Create a list to hold the trajectory points.
        points = []
        
        # Define the first joint trajectory point.
        point1_msg = JointTrajectoryPoint()
        point1_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1_msg.time_from_start = Duration(seconds=2.0).to_msg()  # Time to reach this point.

        # Define the second joint trajectory point.
        point2_msg = JointTrajectoryPoint()
        point2_msg.positions = [0.5, 0.0, 1.2, 0.0, 0.0, 0.0]
        point2_msg.time_from_start = Duration(seconds=6, nanoseconds=0).to_msg()  # Time to reach this point.

        # Define the third joint trajectory point.
        point3_msg = JointTrajectoryPoint()
        point3_msg.positions = [0.7, 0.2, 1.2, 0.0, 0.0, 0.0]
        point3_msg.time_from_start = Duration(seconds=12, nanoseconds=0).to_msg()  # Time to reach this point.

        # Commented out points that are not currently in use.
        # point4_msg = JointTrajectoryPoint()
        # point4_msg.positions = [1.0, 0.0, -0.52, 0.0, 0.0, 0.0] 
        # point4_msg.time_from_start = Duration(seconds=8, nanoseconds=0).to_msg()

        # point5_msg = JointTrajectoryPoint()
        # point5_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # point5_msg.time_from_start = Duration(seconds=10, nanoseconds=0).to_msg()

        # Append the defined points to the list of trajectory points.
        points.append(point1_msg)
        points.append(point2_msg)
        # points.append(point3_msg)
        # points.append(point4_msg)
        # points.append(point5_msg)

        # Define the joint names for the trajectory.
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        goal_msg = FollowJointTrajectory.Goal()
        # Set the goal time tolerance.
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        # Assign joint names and trajectory points to the goal message.
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        # Wait for the action server to be available.
        self.action_client.wait_for_server()
        # Send the goal to the action server and set the callback for feedback.
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # Add a callback to handle the goal response.
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the response from sending the goal.
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')  # Log if the goal was not accepted.
            return

        self.get_logger().info('Goal accepted')  # Log if the goal was accepted.

        # Get the result of the goal execution.
        self.get_result_future = goal_handle.get_result_async()
        # Add a callback to handle the result.
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Handle the result from executing the goal.
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))  # Log the result.
        rclpy.shutdown()  # Shut down the ROS2 node after receiving the result.

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server.
        feedback = feedback_msg.feedback  # Process the feedback if needed.


def main(args=None):
    # Main function to initialize the ROS2 node and action client.
    rclpy.init()
    
    action_client = TrajectoryActionClient()  # Create an instance of the action client.
    future = action_client.send_goal_linear()  # Send the linear goal.
    rclpy.spin(action_client)  # Keep the node running and processing callbacks.

if __name__ == '__main__':
    main()  # Execute the main function when the script is run.
