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
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from enum import Enum, auto
import math

class RobotState(Enum):
    COMPUTE = auto()
    RLEFT = auto()
    RRIGHT = auto()
    FORWARD = auto()

# Edit this variable based on namespace in current environment.
namespace = "/"

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('gz_publisher')
        
        # Pubs and Subs
        self.publisher_ = self.create_publisher(Twist, namespace+'diff_drive/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, namespace+'diff_drive/scan', self.laser_callback, 10)
        self.odo_subscription = self.create_subscription(Odometry, namespace+'diff_drive/odometry', self.odometry_callback, 10)
        
        # Timing Variables
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.check_distance = 50
        self.check_index = 0

        # Direction / Orientation / Motion
        self.theta = 0
        self.dtheta = self.theta
        self.twist_msg = Twist()
        self.state = RobotState.FORWARD
        self.distance = 0
        self.ddistance = 0
        self.correction = False

    def timer_callback(self):
        self.get_logger().info(f'CALLBACK: #{self.i}')
        msg = Twist()

        match self.state:
            case RobotState.COMPUTE:
                self.get_logger().info(f'Compute: #{self.theta, self.dtheta}')
                if not self.correction:
                    msg.angular.z = 0.5
                    if abs(self.theta - self.dtheta) > (0.5):
                        if self.distance > 5.2:
                            self.state = RobotState.FORWARD
                            self.check_index = self.i
                        else:
                            self.ddistance = self.distance
                            self.correction = True
                else:
                    msg.angular.z = -0.5
                    if abs(self.theta - self.dtheta) > (0.5):
                        self.correction = False
                        if self.ddistance < 2.3:
                            self.state = RobotState.RRIGHT
                        else:
                            self.check_index = self.i
                            self.state = RobotState.FORWARD

            case RobotState.RLEFT:
                self.get_logger().info(f'RLEFT: #{self.theta, self.dtheta}')
                msg.angular.z = 0.5
                
                if abs(self.theta - self.dtheta) > (0.5):
                    self.state = RobotState.FORWARD
                    self.check_index = self.i

            case RobotState.RRIGHT:
                self.get_logger().info(f'RRIGHT: #{self.theta - self.dtheta}')
                msg.angular.z = -0.5
                if abs(self.dtheta - self.theta) > (0.5):
                    self.state = RobotState.FORWARD
                    self.check_index = self.i

            case _:
                self.get_logger().info(f'FORWARD: #{self.i}')

                if abs(self.distance) < 3:
                    self.dtheta = self.theta
                    self.state = RobotState.COMPUTE
                else:
                    msg.linear.x = 5.0

        self.publisher_.publish(msg)
        self.i += 1

    def laser_callback(self, msg):
        #self.get_logger().info(f'DISTANCE: {msg.ranges}')
        self.distance = msg.ranges[0]

    def odometry_callback(self, msg):
        
        self.theta = msg.pose.pose.orientation.z
        if self.theta < 0:
            self.theta += 1
        else:
            self.theta *= 2
        #self.get_logger().info(f'THETA: {self.theta}')


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
