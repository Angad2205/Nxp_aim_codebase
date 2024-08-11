# Copyright 2024 NXP

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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import math
import time

from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_10_PERCENT = SPEED_MAX * 0.10
SPEED_20_PERCENT = SPEED_MAX * 0.20
SPEED_30_PERCENT = SPEED_MAX * 0.30
SPEED_35_PERCENT = SPEED_MAX * 0.35
SPEED_40_PERCENT = SPEED_MAX * 0.40
SPEED_45_PERCENT = SPEED_MAX * 0.45
SPEED_60_PERCENT = SPEED_MAX * 0.60
SPEED_65_PERCENT = SPEED_MAX * 0.65
SPEED_50_PERCENT = SPEED_MAX * 0.50
SPEED_55_PERCENT = SPEED_MAX * 0.55
SPEED_70_PERCENT = SPEED_MAX * 0.70
SPEED_75_PERCENT = SPEED_MAX * 0.75
SPEED_80_PERCENT = SPEED_MAX * 0.80

THRESHOLD_OBSTACLE_VERTICAL = 0.8
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25


class LineFollower(Node):
    """ Initializes line follower node with the required publishers and subscriptions.

        Returns:
            None
    """
    def __init__(self):
        super().__init__('line_follower')


        # Subscription for edge vectors.
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT)

        # Publisher for joy (for moving the rover in manual mode).
        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT)

        # Subscription for traffic status.
        self.subscription_traffic = self.create_subscription(
            TrafficStatus,
            '/traffic_status',
            self.traffic_status_callback,
            QOS_PROFILE_DEFAULT)

        # Subscription for LIDAR data.
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)

        self.traffic_status = TrafficStatus()

        self.obstacle_detected = False
        self.ramp_detected = False

        # Turn values
        self.turn_rightOfRight = 0.25
        self.turn_rightOfMiddle = 0.50
        self.turn_rightOfLeft = 0.75
        self.turn_leftOfRight = -0.25
        self.turn_leftOfMiddle = -0.50
        self.turn_leftOfLeft = -0.75
        self.turn_frontRight = 1.0
        self.turn_frontLeft = -1.0

    """ Operates the rover in manual mode by publishing on /cerebri/in/joy.

        Args:
            speed: the speed of the car in float. Range = [-1.0, +1.0];
                Direction: forward for positive, reverse for negative.
            turn: steer value of the car in float. Range = [-1.0, +1.0];
                Direction: left turn for positive, right turn for negative.

        Returns:
            None
    """
    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()

        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

        msg.axes = [0.0, speed, 0.0, turn]

        self.publisher_joy.publish(msg)

    """ Analyzes edge vectors received from /edge_vectors to achieve line follower application.
        It checks for existence of ramps & obstacles on the track through instance members.
        These instance members are updated by the lidar_callback using LIDAR data.
        The speed and turn are calculated to move the rover using rover_move_manual_mode.

        Args:
            message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

        Returns:
            None
    """
    def edge_vectors_callback(self, message):
        speed = SPEED_MAX
        turn = TURN_MIN

        vectors = message
        half_width = vectors.image_width / 2

        # NOTE: participants may improve algorithm for line follower.
        if (vectors.vector_count == 0):  # none.
            speed = SPEED_35_PERCENT

        if (vectors.vector_count == 1):  # curve.
            # Calculate the magnitude of the x-component of the vector.
            deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
            turn = (deviation) / (vectors.image_width)
            #turn = deviation
            speed = SPEED_50_PERCENT

        if (vectors.vector_count == 2):  # straight.
            # Calculate the middle point of the x-components of the vectors.
            middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
            middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
            middle_x = (middle_x_left + middle_x_right) / 2
            deviation = half_width - middle_x
            turn = deviation / half_width
            speed = SPEED_60_PERCENT

        if self.ramp_detected is True:
            # TODO: participants need to decide action on detection of ramp/bridge.
            speed = SPEED_20_PERCENT
            print("ramp/bridge detected")

        if self.obstacle_detected is True:
            # TODO: participants need to decide action on detection of obstacle.
            print("obstacle detected")

        if (self.traffic_status.stop_sign is True):
            speed = SPEED_MIN
            print("stop sign detected")

        self.rover_move_manual_mode(speed, turn)

    """ Updates instance member with traffic status message received from /traffic_status.

        Args:
            message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

        Returns:
            None
    """
    def traffic_status_callback(self, message):
        self.traffic_status = message

    """ Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

        Args:
            message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

        Returns:
            None
    """
    def lidar_callback(self, message):
        # TODO: participants need to implement logic for detection of ramps and obstacles.

        shield_vertical = 4
        shield_horizontal = 1
        theta = math.atan(shield_vertical / shield_horizontal)

        # Get the middle half of the ranges array returned by the LIDAR.
        length = float(len(message.ranges))
        ranges = message.ranges[int(length / 4): int(3 * length / 4)]

        # Separate the ranges into the part in the front and the part on the sides.
        length = float(len(ranges))
        front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
        right_ranges = ranges[0: int(length * theta / PI)]
        left_ranges = ranges[int(length * (PI - theta) / PI):]

        # Divide right and left ranges into three parts.
        third_length_right = int(len(right_ranges) / 3)
        rightOfRight = right_ranges[:third_length_right]
        rightOfMiddle = right_ranges[third_length_right:2 * third_length_right]
        rightOfLeft = right_ranges[2 * third_length_right:]

        third_length_left = int(len(left_ranges) / 3)
        leftOfRight = left_ranges[:third_length_left]
        leftOfMiddle = left_ranges[third_length_left:2 * third_length_left]
        leftOfLeft = left_ranges[2 * third_length_left:]

        # Divide front ranges into two parts.
        half_length_front = int(len(front_ranges) / 2)
        frontRight = front_ranges[:half_length_front]
        frontLeft = front_ranges[half_length_front:]

        # Process right ranges.
        for i in range(len(rightOfRight)):
            if (rightOfRight[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_rightOfRight)
                return

        for i in range(len(rightOfMiddle)):
            if (rightOfMiddle[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_rightOfMiddle)
                return

        for i in range(len(rightOfLeft)):
            if (rightOfLeft[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_rightOfLeft)
                return

        # Process left ranges.
        for i in range(len(leftOfLeft)):
            if (leftOfLeft[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_leftOfLeft)
                return

        for i in range(len(leftOfMiddle)):
            if (leftOfMiddle[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_leftOfMiddle)
                return

        for i in range(len(leftOfRight)):
            if (leftOfRight[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_leftOfRight)
                return

        # Process front ranges.
        for i in range(len(frontRight)):
            if (frontRight[i] < THRESHOLD_OBSTACLE_VERTICAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_frontRight)
                return

        for i in range(len(frontLeft)):
            if (frontLeft[i] < THRESHOLD_OBSTACLE_VERTICAL):
                self.rover_move_manual_mode(SPEED_60_PERCENT, self.turn_frontLeft)
                return

        self.obstacle_detected = False


def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

