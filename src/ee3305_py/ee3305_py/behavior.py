from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, String, Bool
from visualization_msgs.msg import Marker, MarkerArray


class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(10))
        self.declare_parameter("plan_frequency", float(2))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.plan_frequency_ = self.get_parameter("plan_frequency").value

        # Room Waypoints Definition (Matches House Floorplan Rooms 1-6)
        self.room_waypoints_ = {
            1: {"name": "Room 1 (Top-Left)", "x": -4.20, "y": 3.50},
            2: {"name": "Room 2 (Upper-Middle)", "x": -0.50, "y": 3.20},
            3: {"name": "Room 3 (Upper-Right-Middle)", "x": 3.20, "y": 3.20},
            4: {"name": "Room 4 (Top-Right)", "x": 6.80, "y": 3.20},
            5: {"name": "Room 5 (Bottom-Right)", "x": 7.80, "y": -1.50},
            6: {"name": "Room 6 (Bottom-Left)", "x": -4.30, "y": -1.00},
        }

        # Handles: Topic Subscribers
        # Goal pose subscriber (from RViz 2D Goal Pose tool)
        self.sub_goal_pose_ = self.create_subscription(
            PoseStamped, "goal_pose", self.callbackSubGoalPose_, 10
        )

        # Room goal subscriber (send room number 1-6)
        self.sub_room_goal_ = self.create_subscription(
            Int32, "room_goal", self.callbackSubRoomGoal_, 10
        )

        # Patrol mode subscriber (toggle continuous patrol)
        self.sub_patrol_ = self.create_subscription(
            Bool, "patrol", self.callbackSubPatrol_, 10
        )

        # Stop / Emergency Stop subscriber
        self.sub_stop_ = self.create_subscription(
            Bool, "stop", self.callbackSubStop_, 10
        )

        # Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackSubOdom_, 10
        )

        # Handles: Topic Publishers
        # Path request publisher
        self.pub_path_request_ = self.create_publisher(
            Path, "path_request", 10
        )

        # Command velocity publisher (for emergency halts)
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, "cmd_vel", 10
        )

        # Goal pose publisher (broadcasts dispatched room target to RViz2 and Web GUI)
        self.pub_goal_pose_ = self.create_publisher(
            PoseStamped, "goal_pose", 10
        )

        # Room markers publisher (visual 3D labels for RViz2)
        qos_marker = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_room_markers_ = self.create_publisher(
            MarkerArray, "room_markers", qos_marker
        )

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)
        self.timer_plan_ = self.create_timer(
            1.0 / self.plan_frequency_, self.callbackTimerPlan_
        )
        self.timer_markers_ = self.create_timer(1.0, self.publishRoomMarkers_)

        # Other Instance Variables
        self.received_goal_coords_ = False
        self.received_rbt_coords_ = False
        self.goal_reached_ = True
        self.current_room_id_ = None
        self.patrol_active_ = False
        self.patrol_index_ = 0
        self.patrol_sequence_ = [1, 2, 3, 4, 5, 6]
        self.rbt_x_ = 0.0
        self.rbt_y_ = 0.0
        self.goal_x_ = 0.0
        self.goal_y_ = 0.0

        # Initial Marker Broadcast
        self.publishRoomMarkers_()

    # Callbacks =============================================================

    # Goal pose subscriber callback (manual click from RViz).
    def callbackSubGoalPose_(self, msg: PoseStamped):
        self.patrol_active_ = False  # cancel automated patrol if user clicks manual goal
        self.current_room_id_ = None
        self.received_goal_coords_ = True
        self.goal_reached_ = False

        self.goal_x_ = msg.pose.position.x
        self.goal_y_ = msg.pose.position.y

        self.get_logger().info(
            f"Received New Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
        )
        self.callbackTimerPlan_()

    # Room goal subscriber callback (e.g., ros2 topic pub /room_goal std_msgs/Int32 "{data: 3}")
    def callbackSubRoomGoal_(self, msg: Int32):
        room_id = msg.data
        if room_id not in self.room_waypoints_:
            self.get_logger().warn(
                f"Unknown Room ID {room_id}. Valid rooms are 1 to 6: {list(self.room_waypoints_.keys())}"
            )
            return

        self.patrol_active_ = False
        self.setRoomGoal_(room_id)

    # Patrol toggle callback
    def callbackSubPatrol_(self, msg: Bool):
        self.patrol_active_ = msg.data
        if self.patrol_active_:
            self.patrol_index_ = 0
            first_room = self.patrol_sequence_[self.patrol_index_]
            self.get_logger().info(f"Patrol Mode ACTIVATED! Visiting sequence: {self.patrol_sequence_}")
            self.setRoomGoal_(first_room)
        else:
            self.get_logger().info("Patrol Mode DEACTIVATED — halting robot immediately.")
            self.stopNavigation_()

    # Stop subscriber callback
    def callbackSubStop_(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("Emergency / Mission STOP signal received — halting robot.")
            self.stopNavigation_()

    # Cancels active navigation and halts robot
    def stopNavigation_(self):
        self.received_goal_coords_ = False
        self.goal_reached_ = True
        self.current_room_id_ = None
        self.patrol_active_ = False

        # Request planner to clear path
        empty_req = Path()
        empty_req.header.stamp = self.get_clock().now().to_msg()
        empty_req.header.frame_id = "map"
        self.pub_path_request_.publish(empty_req)

        # Publish zero velocity command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.pub_cmd_vel_.publish(cmd)

    # Sets destination to a specific numbered room
    def setRoomGoal_(self, room_id: int):
        room = self.room_waypoints_[room_id]
        self.current_room_id_ = room_id
        self.goal_x_ = room["x"]
        self.goal_y_ = room["y"]
        self.received_goal_coords_ = True
        self.goal_reached_ = False

        # Broadcast goal pose to RViz2 and Web GUI
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = self.goal_x_
        goal_msg.pose.position.y = self.goal_y_
        goal_msg.pose.orientation.w = 1.0
        self.pub_goal_pose_.publish(goal_msg)

        self.get_logger().info(
            f"--> DISPATCHED MISSION: Moving to {room['name']} @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})"
        )
        self.callbackTimerPlan_()

    # Odometry subscriber callback.
    def callbackSubOdom_(self, msg: Odometry):
        self.received_rbt_coords_ = True
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

    # Callback for timer: monitor goal progress and handle patrol progression
    def callbackTimer_(self):
        if not self.received_rbt_coords_ or not self.received_goal_coords_:
            return

        dx = self.goal_x_ - self.rbt_x_
        dy = self.goal_y_ - self.rbt_y_
        dist = hypot(dx, dy)

        goal_is_close = dist < 0.15

        if goal_is_close and not self.goal_reached_:
            room_name = (
                self.room_waypoints_[self.current_room_id_]["name"]
                if self.current_room_id_ in self.room_waypoints_
                else f"Goal @ ({self.goal_x_:.2f}, {self.goal_y_:.2f})"
            )
            self.get_logger().info(f"[SUCCESS] Reached {room_name}!")
            self.goal_reached_ = True

            # If patrol mode is active, advance to next room
            if self.patrol_active_:
                self.patrol_index_ = (self.patrol_index_ + 1) % len(self.patrol_sequence_)
                next_room = self.patrol_sequence_[self.patrol_index_]
                self.get_logger().info(f"Patrol next destination -> Room {next_room}")
                self.setRoomGoal_(next_room)

        elif not goal_is_close and self.goal_reached_:
            self.goal_reached_ = False

    # Publish path requests periodically to update the global planner
    def callbackTimerPlan_(self):
        if not self.received_goal_coords_ or not self.received_rbt_coords_:
            return

        msg_path_request = Path()
        msg_path_request.header.stamp = self.get_clock().now().to_msg()
        msg_path_request.header.frame_id = "map"

        # Robot coordinates at [0]
        rbt_pose = PoseStamped()
        rbt_pose.header.stamp = msg_path_request.header.stamp
        rbt_pose.header.frame_id = "map"
        rbt_pose.pose.position.x = self.rbt_x_
        rbt_pose.pose.position.y = self.rbt_y_
        msg_path_request.poses.append(rbt_pose)

        # Goal coordinates at [1]
        goal_pose = PoseStamped()
        goal_pose.header.stamp = msg_path_request.header.stamp
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self.goal_x_
        goal_pose.pose.position.y = self.goal_y_
        msg_path_request.poses.append(goal_pose)

        self.pub_path_request_.publish(msg_path_request)

    # Publishes 3D discs and floating text labels for all 6 rooms in RViz2
    def publishRoomMarkers_(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        colors = [
            (0.1, 0.8, 0.9),  # Room 1: Cyan
            (0.2, 0.9, 0.3),  # Room 2: Green
            (1.0, 0.8, 0.1),  # Room 3: Amber/Gold
            (0.9, 0.3, 0.9),  # Room 4: Magenta
            (0.3, 0.5, 1.0),  # Room 5: Royal Blue
            (1.0, 0.4, 0.2),  # Room 6: Coral/Orange
        ]

        for room_id, info in self.room_waypoints_.items():
            r_col, g_col, b_col = colors[(room_id - 1) % len(colors)]

            # 1. Base Cylinder Disc on the floor
            disc = Marker()
            disc.header.frame_id = "map"
            disc.header.stamp = now
            disc.ns = "room_discs"
            disc.id = room_id
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.position.x = info["x"]
            disc.pose.position.y = info["y"]
            disc.pose.position.z = 0.02
            disc.pose.orientation.w = 1.0
            disc.scale.x = 0.60
            disc.scale.y = 0.60
            disc.scale.z = 0.04
            disc.color.r = r_col
            disc.color.g = g_col
            disc.color.b = b_col
            disc.color.a = 0.65
            marker_array.markers.append(disc)

            # 2. Floating 3D Text Label above the disc
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = now
            text.ns = "room_labels"
            text.id = 100 + room_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = info["x"]
            text.pose.position.y = info["y"]
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35  # Text height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"Room {room_id}"
            marker_array.markers.append(text)

        self.pub_room_markers_.publish(marker_array)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

