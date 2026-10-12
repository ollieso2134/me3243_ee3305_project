
import math
import threading
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool, String


ROOM_WAYPOINTS = {
    1: (-4.20, 3.50),
    2: (-0.50, 3.20),
    3: (3.20, 3.20),
    4: (6.80, 3.20),
    5: (7.80, -1.50),
    6: (-4.30, -1.00),
}


class NavigationManager(Node):
    def __init__(self):
        super().__init__("me3243_navigation_manager")

        self.lock = threading.RLock()

        self.pub_room = self.create_publisher(
            Int32, "/room_goal", 10
        )

        self.pub_stop = self.create_publisher(
            Bool, "/stop", 10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.sub_mode = self.create_subscription(
            String,
            "/control_mode",
            self.mode_callback,
            10,
        )

        self.active = False
        self.status = "idle"
        self.current_room = None

        self.robot_position = None
        self.control_mode = "unknown"

        self.goal_start_time = None
        self.arrival_tolerance = 0.15

    def mode_callback(self, msg):
        with self.lock:
            self.control_mode = msg.data.lower()

        self.get_logger().info(
            f"Control mode: {self.control_mode}"
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        with self.lock:
            self.robot_position = (x, y)

            if not self.active or self.current_room is None:
                return

            # Avoid immediate false arrival from old odometry.
            if (
                self.goal_start_time is not None
                and time.monotonic() - self.goal_start_time < 1.0
            ):
                return

            goal_x, goal_y = ROOM_WAYPOINTS[self.current_room]

            distance = math.hypot(
                goal_x - x,
                goal_y - y
            )

            if distance < self.arrival_tolerance:
                self.active = False
                self.status = "succeeded"

                self.get_logger().info(
                    f"Reached Room {self.current_room}"
                )

    def navigate(self, room):
        if type(room) is not int or room not in ROOM_WAYPOINTS:
            return False

        with self.lock:
            if self.active:
                self.get_logger().warn(
                    "Navigation already in progress."
                )
                return False

            if self.robot_position is None:
                self.get_logger().warn(
                    "No odometry received yet."
                )
                return False

            # The dashboard publishes manual/voice mode.
            if self.control_mode != "voice":
                self.get_logger().warn(
                    "Voice mode is not enabled."
                )
                return False

            self.current_room = room
            self.active = True
            self.status = "running"
            self.goal_start_time = time.monotonic()

            msg = Int32()
            msg.data = room
            self.pub_room.publish(msg)

            self.get_logger().info(
                f"Voice navigation requested: Room {room}"
            )

            return True

    def cancel(self):
        with self.lock:
            self.active = False
            self.status = "canceled"
            self.current_room = None
            self.goal_start_time = None

            msg = Bool()
            msg.data = True
            self.pub_stop.publish(msg)

            self.get_logger().warn(
                "Navigation stop requested."
            )

            return True
