# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS Client Library for Python
import rclpy
import math
import requests
from rclpy.node import Node

from std_msgs.msg import String


class RobotPoseSubscriber(Node):
    def __init__(self):
        super().__init__('robot_pose_subscriber')

        # The node subscribes to messages of type std_msgs/String,
        # over a topic named: /addison
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # --- parsing robot pose ---
        info = msg.data.split(',')
        pose = [float(info[0]), float(info[1])]
        origin = [0, 0]
        isReached = self.check_reach_yet(pose, origin, 20)
        self.get_logger().info(f"Robot reached the goal: {isReached}")
        r = requests.get('https://www.google.com.tw/')
        if r.status_code == requests.codes.ok:
            print("OK")

    def check_reach_yet(self, p1, p2, r):
        # --- euclidean_distance ---
        distance = math.sqrt(((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2))
        self.get_logger().info(f"distance from origin: {distance}")
        return True if(distance < r) else False


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a subscriber
    robot_pose_subscriber = RobotPoseSubscriber()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(robot_pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_pose_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
