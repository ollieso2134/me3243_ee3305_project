import io
import os
import json
import threading
from math import atan2
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Int32, Bool, String


class WebGUINode(Node):

    def __init__(self, node_name="web_gui"):
        super().__init__(node_name)

        # Telemetry State
        self.rbt_x_ = -2.0
        self.rbt_y_ = -0.5
        self.rbt_yaw_ = 0.0
        self.goal_x_ = None
        self.goal_y_ = None
        self.path_points_ = []
        self.patrol_active_ = False
        self.control_mode_ = "manual"

        # Map state & in-memory raster
        self.map_received_ = False
        self.map_png_bytes_ = None
        self.map_w_ = 315
        self.map_h_ = 227
        self.map_res_ = 0.05
        self.map_ox_ = -5.75
        self.map_oy_ = -5.17

        self.room_waypoints_ = [
            {"id": 1, "name": "Room 1 (Top-Left)", "x": -4.20, "y": 3.50, "color": "#38bdf8"},
            {"id": 2, "name": "Room 2 (Kitchen)", "x": -0.50, "y": 3.20, "color": "#10b981"},
            {"id": 3, "name": "Room 3 (Upper-Mid-Right)", "x": 3.20, "y": 3.20, "color": "#f59e0b"},
            {"id": 4, "name": "Room 4 (Large Bedroom)", "x": 6.80, "y": 3.20, "color": "#ec4899"},
            {"id": 5, "name": "Room 5 (Bottom-Right)", "x": 7.80, "y": -1.50, "color": "#3b82f6"},
            {"id": 6, "name": "Room 6 (Bottom-Left)", "x": -4.30, "y": -1.00, "color": "#f97316"},
        ]

        # Subscribers
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackOdom_, 10
        )
        self.sub_path_ = self.create_subscription(
            Path, "path", self.callbackPath_, 10
        )
        self.sub_goal_pose_ = self.create_subscription(
            PoseStamped, "goal_pose", self.callbackGoalPose_, 10
        )
        self.sub_room_goal_ = self.create_subscription(
            Int32, "room_goal", self.callbackRoomGoal_, 10
        )
        self.sub_patrol_ = self.create_subscription(
            Bool, "patrol", self.callbackPatrol_, 10
        )

        # Map Subscriber (latched / transient local QoS)
        qos_map = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_map_ = self.create_subscription(
            OccupancyGrid, "map", self.callbackMap_, qos_map
        )

        # Publishers
        self.pub_goal_pose_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.pub_room_goal_ = self.create_publisher(Int32, "room_goal", 10)
        self.pub_patrol_ = self.create_publisher(Bool, "patrol", 10)
        self.pub_stop_ = self.create_publisher(Bool, "stop", 10)
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.pub_path_ = self.create_publisher(Path, "path", 10)
        self.pub_control_mode_ = self.create_publisher(String, "control_mode", 10)

        self.get_logger().info("Web GUI ROS 2 Node initialized with /map subscription.")

    def callbackOdom_(self, msg: Odometry):
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        delta_y = 2.0 * (q.w * q.z + q.x * q.y)
        delta_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rbt_yaw_ = atan2(delta_y, delta_x)

    def callbackPath_(self, msg: Path):
        self.path_points_ = [
            [p.pose.position.x, p.pose.position.y] for p in msg.poses
        ]

    def callbackGoalPose_(self, msg: PoseStamped):
        self.goal_x_ = msg.pose.position.x
        self.goal_y_ = msg.pose.position.y

    def callbackRoomGoal_(self, msg: Int32):
        room_id = msg.data
        for rm in self.room_waypoints_:
            if rm["id"] == room_id:
                self.goal_x_ = rm["x"]
                self.goal_y_ = rm["y"]
                self.get_logger().info(f"Received external Room Goal {room_id}: Target set to ({self.goal_x_:.2f}, {self.goal_y_:.2f})")
                break

    def callbackPatrol_(self, msg: Bool):
        self.patrol_active_ = msg.data

    def callbackMap_(self, msg: OccupancyGrid):
        self.map_received_ = True
        self.map_w_ = msg.info.width
        self.map_h_ = msg.info.height
        self.map_res_ = msg.info.resolution
        self.map_ox_ = msg.info.origin.position.x
        self.map_oy_ = msg.info.origin.position.y

        try:
            w = self.map_w_
            h = self.map_h_
            raw = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # OccupancyGrid row 0 is bottom (min y); flip vertically for image coordinates (row 0 = top)
            grid = np.flipud(raw)

            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            # Unknown (-1 or 50) -> Pure black #000000
            rgba[(grid == -1) | (grid == 50)] = [0, 0, 0, 255]
            # Free space (0) -> Subtle tactical floor #0d0d0d
            rgba[grid == 0] = [13, 13, 13, 255]
            # Obstacles / walls (100 or >= 65) -> Crisp white #ffffff
            rgba[grid >= 65] = [255, 255, 255, 255]

            img = Image.fromarray(rgba)
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            self.map_png_bytes_ = buf.getvalue()
            self.get_logger().info(
                f"Received /map topic ({w}x{h}, res={self.map_res_:.2f}m, origin=({self.map_ox_:.2f}, {self.map_oy_:.2f})). Rendered map in-memory ({len(self.map_png_bytes_)} bytes)!"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing /map message: {e}")

    def sendRoomGoal(self, room_id: int):
        stop_msg = Bool()
        stop_msg.data = False
        self.pub_stop_.publish(stop_msg)

        for rm in self.room_waypoints_:
            if rm["id"] == room_id:
                self.goal_x_ = rm["x"]
                self.goal_y_ = rm["y"]
                break

        msg = Int32()
        msg.data = room_id
        self.pub_room_goal_.publish(msg)
        self.get_logger().info(f"Published Room Goal: Room {room_id}")

    def sendGoalCoords(self, x: float, y: float):
        stop_msg = Bool()
        stop_msg.data = False
        self.pub_stop_.publish(stop_msg)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.pub_goal_pose_.publish(msg)
        self.goal_x_ = x
        self.goal_y_ = y
        self.get_logger().info(f"Published Manual Goal @ ({x:.2f}, {y:.2f})")

    def togglePatrol(self, active: bool):
        msg = Bool()
        msg.data = active
        self.pub_patrol_.publish(msg)
        self.patrol_active_ = active

        if active:
            stop_msg = Bool()
            stop_msg.data = False
            self.pub_stop_.publish(stop_msg)
            self.get_logger().info("Published Patrol Mode: True")

        if not active:
            # When stopping patrol, immediately cancel goal, clear path and halt
            self.goal_x_ = None
            self.goal_y_ = None
            self.path_points_ = []

            # Broadcast stop signal to controller and behavior
            stop_msg = Bool()
            stop_msg.data = True
            self.pub_stop_.publish(stop_msg)

            # Clear path
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = "map"
            self.pub_path_.publish(empty_path)

            # Publish zero velocity
            stop_twist = TwistStamped()
            stop_twist.header.stamp = self.get_clock().now().to_msg()
            stop_twist.twist.linear.x = 0.0
            stop_twist.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(stop_twist)
            self.get_logger().info("Patrol mode STOPPED: halted robot and cleared target.")
        else:
            self.get_logger().info("Published Patrol Mode: True")

    def emergencyStop(self):
        self.goal_x_ = None
        self.goal_y_ = None
        self.path_points_ = []
        self.patrol_active_ = False

        # Stop patrol
        patrol_msg = Bool()
        patrol_msg.data = False
        self.pub_patrol_.publish(patrol_msg)

        # Broadcast stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.pub_stop_.publish(stop_msg)

        # Clear path
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = "map"
        self.pub_path_.publish(empty_path)

        # Publish zero velocity command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.pub_cmd_vel_.publish(msg)

        self.get_logger().warn("EMERGENCY STOP Triggered via Web GUI: robot halted, path cleared.")

    def setControlMode(self, mode: str):
        self.control_mode_ = mode.lower()
        msg = String()
        msg.data = self.control_mode_
        self.pub_control_mode_.publish(msg)
        self.get_logger().info(f"Control Mode switched to: {self.control_mode_.upper()}")

    def getStatusDict(self):
        goal_dict = None
        if self.goal_x_ is not None and self.goal_y_ is not None:
            goal_dict = {"x": self.goal_x_, "y": self.goal_y_}
        return {
            "robot": {
                "x": self.rbt_x_,
                "y": self.rbt_y_,
                "yaw": self.rbt_yaw_
            },
            "goal": goal_dict,
            "path": self.path_points_,
            "rooms": self.room_waypoints_,
            "patrol": self.patrol_active_,
            "control_mode": self.control_mode_,
            "map_info": {
                "received": self.map_received_,
                "width": self.map_w_,
                "height": self.map_h_,
                "resolution": self.map_res_,
                "ox": self.map_ox_,
                "oy": self.map_oy_,
            },
            "connected": True
        }


# Global reference for HTTP handler
global_node = None
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


class WebHandler(BaseHTTPRequestHandler):

    def _set_headers(self, content_type="application/json", status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_OPTIONS(self):
        self._set_headers(status=204)

    def do_GET(self):
        global global_node
        clean_path = self.path.split("?")[0]
        if clean_path == "/" or clean_path == "/index.html":
            file_path = os.path.join(STATIC_DIR, "index.html")
            if os.path.exists(file_path):
                self._set_headers("text/html; charset=utf-8")
                with open(file_path, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, "index.html not found")
        elif clean_path in ["/map.png", "/dark_map.png"]:
            if global_node and global_node.map_png_bytes_:
                self._set_headers("image/png")
                self.wfile.write(global_node.map_png_bytes_)
            else:
                self.send_error(404, "Map not ready yet")
        elif self.path == "/api/status":
            if global_node:
                data = global_node.getStatusDict()
                self._set_headers("application/json")
                self.wfile.write(json.dumps(data).encode("utf-8"))
            else:
                self.send_error(500, "Node not ready")
        else:
            self.send_error(404, "Endpoint not found")

    def do_POST(self):
        global global_node
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else "{}"
        try:
            req_data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            req_data = {}

        if self.path == "/api/send_room":
            room_id = int(req_data.get("room", 1))
            if global_node:
                global_node.sendRoomGoal(room_id)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "room": room_id}).encode("utf-8"))

        elif self.path == "/api/send_goal":
            gx = float(req_data.get("x", 0.0))
            gy = float(req_data.get("y", 0.0))
            if global_node:
                global_node.sendGoalCoords(gx, gy)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "x": gx, "y": gy}).encode("utf-8"))

        elif self.path == "/api/patrol":
            active = bool(req_data.get("active", False))
            if global_node:
                global_node.togglePatrol(active)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "patrol": active}).encode("utf-8"))

        elif self.path == "/api/stop":
            if global_node:
                global_node.emergencyStop()
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "stopped"}).encode("utf-8"))

        elif self.path == "/api/mode":
            mode = str(req_data.get("mode", "manual"))
            if global_node:
                global_node.setControlMode(mode)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "mode": mode}).encode("utf-8"))
        else:
            self.send_error(404, "Unknown API endpoint")

    def log_message(self, format, *args):
        # Suppress spammy HTTP polling logs in terminal
        pass


def run_http_server(port=8080):
    server = ThreadingHTTPServer(("0.0.0.0", port), WebHandler)
    server.serve_forever()


def main(args=None):
    global global_node
    rclpy.init(args=args)
    global_node = WebGUINode()

    # Start HTTP server in a daemon thread
    port = 8080
    http_thread = threading.Thread(target=run_http_server, args=(port,), daemon=True)
    http_thread.start()

    global_node.get_logger().info(
        f"\n=======================================================\n"
        f"  TURTLEBOT3 WEB MISSION CONTROL IS LIVE!\n"
        f"  Open in Browser: http://localhost:{port} (or http://127.0.0.1:{port})\n"
        f"======================================================="
    )

    try:
        rclpy.spin(global_node)
    except KeyboardInterrupt:
        pass
    finally:
        global_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
