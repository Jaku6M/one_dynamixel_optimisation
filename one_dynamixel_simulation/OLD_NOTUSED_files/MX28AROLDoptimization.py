import os
import subprocess
import time
import json
import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryFeedback
import csv

def launch_gazebo_simulation():
    # Open a new terminal and launch Gazebo simulation
    subprocess.Popen(['gnome-terminal', '--', 'ros2', 'launch', 'one_dynamixel_simulation', 'MX28AR_gazebo.launch.py'])

def send_joint_trajectory_command():
    # Wait for 5 seconds before sending the joint trajectory command
    time.sleep(4)

    # Open a new terminal and send the joint trajectory command
    trajectory = {
        "trajectory": {
            "joint_names": ['joint1'],
            "points": [
                {"positions": [0.0], "time_from_start": {"sec": 1}},
                {"positions": [-1.5708], "time_from_start": {"sec": 3}},
                {"positions": [0.0], "time_from_start": {"sec": 5}},
                {"positions": [1.5708], "time_from_start": {"sec": 7}},
                {"positions": [0.0], "time_from_start": {"sec": 9}}
            ]
        }
    }

    # Convert the trajectory dictionary to a JSON-formatted string
    trajectory_json = json.dumps(trajectory)

    subprocess.Popen(['gnome-terminal', '--','ros2', 'action', 'send_goal', '/joint_trajectory_controller/follow_joint_trajectory', 'control_msgs/action/FollowJointTrajectory', '-f', trajectory_json])

class FeedbackSubscriber(Node):

    def __init__(self):
        super().__init__('feedback_subscriber')
        self.subscription = self.create_subscription(
            FollowJointTrajectoryFeedback,
            '/joint_trajectory_controller/feedback',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        self.csv_filename = 'feedback_data.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time (sec)', 'Position', 'Velocity'])

    def callback(self, msg):
        time_sec = msg.header.stamp.sec
        time_nsec = msg.header.stamp.nanosec
        time = time_sec + time_nsec / 1e9  # Convert nanoseconds to seconds

        position = msg.actual.positions[0] if msg.actual.positions else None
        velocity = msg.actual.velocities[0] if msg.actual.velocities else None

        # Write time, position, and velocity to CSV file
        self.csv_writer.writerow([time, position, velocity])
        self.get_logger().info(f"Recorded data at time {time}: Position={position}, Velocity={velocity}")

    def close_csv_file(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    feedback_subscriber = FeedbackSubscriber()
    try:
        rclpy.spin(feedback_subscriber)
    except KeyboardInterrupt:
        feedback_subscriber.close_csv_file()
        feedback_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
  launch_gazebo_simulation()
  send_joint_trajectory_command()
  main()