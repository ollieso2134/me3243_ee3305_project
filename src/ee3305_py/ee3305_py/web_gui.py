import io
import os
import json
import threading
import tempfile
import time
import subprocess
from collections import deque
from urllib.parse import urlsplit, parse_qs
from math import atan2
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import numpy as np
from PIL import Image

from ee3305_py.planner_comparison import compare_all
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Int32, Bool, String, Empty
from visualization_msgs.msg import Marker, MarkerArray


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
        self.planner_algorithm_ = "dijkstra"
        self.controller_mode_ = "rpp"

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

        # Separate read-only comparison costmap.
        self.compare_costmap_ = None
        self.compare_map_info_ = None
        self.compare_lock_ = threading.RLock()

        self.declare_parameter("max_access_cost", 90)
        self.compare_max_cost_ = self.get_parameter(
            "max_access_cost"
        ).value

        self.sub_compare_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackCompareCostmap_,
            qos_map,
        )

        # AI chat bridge state: ROS messages never block the HTTP server.
        self.ai_lock_ = threading.RLock()
        self.ai_events_ = deque(maxlen=200)
        self.ai_next_event_id_ = 0
        self.ai_ready_ = False
        self.sub_ai_event_ = self.create_subscription(String, "ai_chat_event", self.callbackAIEvent_, 10)
        self.sub_ai_ready_ = self.create_subscription(Bool, "ai_ready", self.callbackAIReady_, 10)
        self.pub_ai_text_ = self.create_publisher(String, "ai_text_command", 10)
        self.pub_ai_audio_ = self.create_publisher(String, "ai_audio_path", 10)
        self.pub_ai_stop_ = self.create_publisher(Bool, "ai_stop", 10)

        # Publishers
        self.pub_goal_pose_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.pub_room_goal_ = self.create_publisher(Int32, "room_goal", 10)
        self.pub_patrol_ = self.create_publisher(Bool, "patrol", 10)
        self.pub_stop_ = self.create_publisher(Bool, "stop", 10)
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.pub_path_ = self.create_publisher(Path, "path", 10)
        self.pub_control_mode_ = self.create_publisher(String, "control_mode", 10)
        self.pub_planner_algorithm_ = self.create_publisher(
            String, "planner_algorithm", 10
        )
        self.pub_controller_mode_ = self.create_publisher(
            String, "controller_mode", 10
        )

        # Dynamic Obstacle State & Publishers (Method A Canvas Placement)
        self.dynamic_obstacles_ = []
        self.obstacle_counter_ = 0
        self.obs_lock_ = threading.RLock()
        self.pub_spawn_obs_ = self.create_publisher(Point, "spawn_obstacle_pt", 10)
        self.pub_clear_obs_ = self.create_publisher(Empty, "clear_obstacles", 10)
        self.pub_dynamic_markers_ = self.create_publisher(MarkerArray, "dynamic_obstacles", 10)

        # Gazebo Harmonic world spawn offset relative to /map frame:
        # In sim_truth, robot spawns at Gazebo (-2.0, -0.5), which is mapped to (0.0, 0.0) in /map.
        # Thus Gazebo world coords = (map_x - 2.00, map_y - 0.50).
        self.declare_parameter("gz_spawn_offset_x", -2.0)
        self.declare_parameter("gz_spawn_offset_y", -0.5)
        self.gz_offset_x_ = float(self.get_parameter("gz_spawn_offset_x").value)
        self.gz_offset_y_ = float(self.get_parameter("gz_spawn_offset_y").value)

        self.get_logger().info(
            f"Web GUI ROS 2 Node initialized with /map and /dynamic_obstacles (Gazebo offset: [{self.gz_offset_x_:.2f}, {self.gz_offset_y_:.2f}])."
        )

    def callbackAIReady_(self, msg):
        with self.ai_lock_:
            self.ai_ready_ = bool(msg.data)
            self.ai_ready_seen_ = time.monotonic()

    def callbackAIEvent_(self, msg):
        try:
            event = json.loads(msg.data)
            role = str(event.get("role", "system"))
            content = str(event.get("text", ""))[:2000]
        except (ValueError, TypeError):
            return
        if role not in ("user", "robot", "system") or not content:
            return
        with self.ai_lock_:
            self.ai_next_event_id_ += 1
            self.ai_events_.append({"id": self.ai_next_event_id_, "role": role, "text": content})

    def aiEvents_(self, after):
        with self.ai_lock_:
            ready = self.ai_ready_ and time.monotonic() - getattr(self, "ai_ready_seen_", 0) < 4.0
            return {"ready": ready, "events": [e for e in self.ai_events_ if e["id"] > after]}

    def sendAIText_(self, text):
        msg = String(); msg.data = text
        self.pub_ai_text_.publish(msg)

    def sendAIAudio_(self, raw, suffix):
        # Browser audio is stored in a per-request temporary file on the ROS host.
        fd, filename = tempfile.mkstemp(prefix="me3243_ai_", suffix=suffix)
        try:
            with os.fdopen(fd, 'wb') as f:
                f.write(raw)
            msg = String(); msg.data = filename
            self.pub_ai_audio_.publish(msg)
        except Exception:
            try: os.unlink(filename)
            except OSError: pass
            raise

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

    def callbackCompareCostmap_(self, msg):
        with self.compare_lock_:
            self.compare_costmap_ = list(msg.data)
            self.compare_map_info_ = (
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
                msg.info.origin.position.x,
                msg.info.origin.position.y,
            )

    def comparePlannersForRoom(self, room_id):
        room = next(
            (r for r in self.room_waypoints_ if r["id"] == room_id),
            None,
        )
        if room is None:
            raise ValueError("Invalid room")

        with self.compare_lock_:
            if self.compare_costmap_ is None:
                raise RuntimeError("Global costmap not received")

            costmap = self.compare_costmap_[:]
            w, h, res, ox, oy = self.compare_map_info_

        start = (self.rbt_x_, self.rbt_y_)
        goal = (room["x"], room["y"])

        result = compare_all(
            costmap, w, h, res, ox, oy,
            start, goal, self.compare_max_cost_,
        )

        result["room"] = room_id
        result["room_name"] = room["name"]
        return result

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

        ai_msg = Bool(); ai_msg.data = True
        self.pub_ai_stop_.publish(ai_msg)
        self.get_logger().warn("EMERGENCY STOP Triggered via Web GUI: robot halted, path cleared.")

    def setControlMode(self, mode: str):
        self.control_mode_ = mode.lower()
        msg = String()
        msg.data = self.control_mode_
        self.pub_control_mode_.publish(msg)
        self.get_logger().info(f"Control Mode switched to: {self.control_mode_.upper()}")

    def setPlannerAlgorithm(self, algorithm: str):
        algorithm = algorithm.strip().lower()
        if algorithm not in ("dijkstra", "astar", "theta"):
            return False

        msg = String()
        msg.data = algorithm
        self.pub_planner_algorithm_.publish(msg)

        self.planner_algorithm_ = algorithm
        self.get_logger().info(
            f"Path planner switched to: {algorithm.upper()}"
        )
        return True

    def setControllerMode(self, mode: str):
        mode = mode.strip().lower()
        if mode not in ("standard", "improved", "rpp"):
            return False

        msg = String()
        msg.data = mode
        self.pub_controller_mode_.publish(msg)

        self.controller_mode_ = mode
        self.get_logger().info(
            f"Controller mode switched to: {mode.upper()}"
        )
        return True

    def spawnObstacle(self, x: float, y: float):
        gz_x = round(float(x) + self.gz_offset_x_, 3)
        gz_y = round(float(y) + self.gz_offset_y_, 3)

        with self.obs_lock_:
            self.obstacle_counter_ += 1
            obs_name = f"dyn_obs_{self.obstacle_counter_}"
            obs = {
                "id": self.obstacle_counter_,
                "name": obs_name,
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "gz_x": gz_x,
                "gz_y": gz_y,
                "radius": 0.20
            }
            self.dynamic_obstacles_.append(obs)

        # 1. Publish point for planner & controller (in ROS /map frame)
        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.0
        self.pub_spawn_obs_.publish(pt)

        # 2. Publish RViz2 visualization marker
        self.publishObstacleMarkers_()

        # 3. Asynchronously spawn 3D collision cylinder in Gazebo Harmonic
        def _spawn_gz(name, gx, gy, mx, my):
            sdf_str = (
                f'<sdf version="1.8">'
                f'<model name="{name}">'
                f'<static>true</static>'
                f'<link name="link">'
                f'<collision name="col"><geometry><cylinder><radius>0.20</radius><length>0.60</length></cylinder></geometry></collision>'
                f'<visual name="vis"><geometry><cylinder><radius>0.20</radius><length>0.60</length></cylinder></geometry>'
                f'<material><ambient>0.95 0.1 0.1 1</ambient><diffuse>0.95 0.1 0.1 1</diffuse></material></visual>'
                f'</link></model></sdf>'
            )
            cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-world", "default",
                "-name", str(name),
                "-string", sdf_str,
                "-x", f"{gx:.3f}",
                "-y", f"{gy:.3f}",
                "-z", "0.30"
            ]
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=4.0, env=os.environ.copy())
                if res.returncode == 0:
                    self.get_logger().info(
                        f"[GAZEBO SPAWNED] Model '{name}' spawned in 3D world at Gazebo ({gx:.2f}, {gy:.2f}) [Map: ({mx:.2f}, {my:.2f})]"
                    )
                else:
                    self.get_logger().warn(f"[GAZEBO SPAWN ERROR] {res.stderr.strip() or res.stdout.strip()}")
            except Exception as e:
                self.get_logger().warn(f"[GAZEBO SPAWN EXCEPTION] {e}")

        threading.Thread(target=_spawn_gz, args=(obs_name, gz_x, gz_y, x, y), daemon=True).start()
        self.get_logger().info(
            f"[DYNAMIC OBSTACLE] Spawned {obs_name} at Map ({x:.2f}, {y:.2f}) | Gazebo ({gz_x:.2f}, {gz_y:.2f})"
        )
        return obs

    def clearAllObstacles(self):
        with self.obs_lock_:
            obs_names = [o["name"] for o in self.dynamic_obstacles_]
            self.dynamic_obstacles_.clear()

        # 1. Publish clear signal to planner & controller
        self.pub_clear_obs_.publish(Empty())

        # 2. Clear RViz2 markers
        marker_arr = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_arr.markers.append(del_marker)
        self.pub_dynamic_markers_.publish(marker_arr)

        # 3. Asynchronously remove all spawned models from Gazebo Harmonic
        def _remove_gz(names):
            for name in names:
                cmd = [
                    "gz", "service", "-s", "/world/default/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", f'name: "{name}", type: 2'
                ]
                try:
                    res = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0, env=os.environ.copy())
                    self.get_logger().info(f"[GAZEBO REMOVED] Model '{name}' removed from 3D world: {res.stdout.strip()}")
                except Exception as e:
                    self.get_logger().warn(f"[GAZEBO REMOVE EXCEPTION] {e}")

        threading.Thread(target=_remove_gz, args=(obs_names,), daemon=True).start()
        self.get_logger().info(f"[DYNAMIC OBSTACLE] Cleared {len(obs_names)} dynamic obstacle(s).")
        return {"status": "ok", "cleared": len(obs_names)}

    def publishObstacleMarkers_(self):
        marker_arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        with self.obs_lock_:
            for obs in self.dynamic_obstacles_:
                # Cylinder marker
                m = Marker()
                m.header.stamp = now
                m.header.frame_id = "map"
                m.ns = "dynamic_obstacles"
                m.id = obs["id"]
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = obs["x"]
                m.pose.position.y = obs["y"]
                m.pose.position.z = 0.25
                m.pose.orientation.w = 1.0
                m.scale.x = 0.36
                m.scale.y = 0.36
                m.scale.z = 0.50
                m.color.r = 1.0
                m.color.g = 0.1
                m.color.b = 0.1
                m.color.a = 0.85
                marker_arr.markers.append(m)

                # Text label
                lbl = Marker()
                lbl.header.stamp = now
                lbl.header.frame_id = "map"
                lbl.ns = "dynamic_obstacles_text"
                lbl.id = obs["id"] + 1000
                lbl.type = Marker.TEXT_VIEW_FACING
                lbl.action = Marker.ADD
                lbl.pose.position.x = obs["x"]
                lbl.pose.position.y = obs["y"]
                lbl.pose.position.z = 0.65
                lbl.pose.orientation.w = 1.0
                lbl.scale.z = 0.15
                lbl.color.r = 1.0
                lbl.color.g = 0.8
                lbl.color.b = 0.0
                lbl.color.a = 1.0
                lbl.text = f"OBSTACLE #{obs['id']}"
                marker_arr.markers.append(lbl)

        self.pub_dynamic_markers_.publish(marker_arr)

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
            "planner_algorithm": self.planner_algorithm_,
            "controller_mode": self.controller_mode_,
            "dynamic_obstacles": list(self.dynamic_obstacles_),
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
        elif clean_path == "/api/ai/events":
            if not global_node:
                self.send_error(503, "Node not ready"); return
            try:
                after = max(0, int(parse_qs(urlsplit(self.path).query).get("after", [0])[0]))
            except (ValueError, TypeError):
                after = 0
            payload = global_node.aiEvents_(after)
            self._set_headers("application/json")
            self.wfile.write(json.dumps(payload).encode("utf-8"))
        elif clean_path == "/api/status":
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
        try:
            content_len = int(self.headers.get("Content-Length", 0))
        except ValueError:
            self.send_error(400, "Invalid Content-Length"); return
        if content_len > 8 * 1024 * 1024:
            self.send_error(413, "Request too large (8 MB maximum)"); return
        if self.path == "/api/ai/audio":
            if not global_node or global_node.control_mode_ != "voice":
                self.send_error(409, "AI voice mode is not active"); return
            if not 128 <= content_len <= 8 * 1024 * 1024:
                self.send_error(400, "Invalid audio size"); return
            kind = self.headers.get("Content-Type", "").split(";")[0].strip().lower()
            suffix = {"audio/webm":".webm", "audio/ogg":".ogg", "audio/wav":".wav", "audio/mp4":".m4a"}.get(kind)
            if not suffix:
                self.send_error(415, "Unsupported audio format"); return
            raw = self.rfile.read(content_len)
            global_node.sendAIAudio_(raw, suffix)
            self._set_headers(); self.wfile.write(b'{"status":"queued"}')
            return
        body = self.rfile.read(content_len).decode("utf-8", errors="replace") if content_len > 0 else "{}"
        try:
            req_data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            req_data = {}

        if self.path == "/api/ai/text":
            value = req_data.get("text", "")
            if not isinstance(value, str) or not value.strip() or len(value) > 1000:
                self.send_error(400, "Invalid text"); return
            if not global_node or global_node.control_mode_ != "voice":
                self.send_error(409, "AI voice mode is not active"); return
            global_node.sendAIText_(value.strip())
            self._set_headers(); self.wfile.write(b'{"status":"queued"}')
        elif self.path == "/api/send_room":
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

        elif self.path == "/api/compare_planners":
            if not global_node:
                self.send_error(503, "Node not ready")
                return

            try:
                room_id = int(req_data.get("room", 0))
                result = global_node.comparePlannersForRoom(room_id)
            except (ValueError, TypeError):
                self.send_error(400, "Invalid room")
                return
            except RuntimeError as exc:
                self.send_error(503, str(exc))
                return
            except Exception as exc:
                global_node.get_logger().error(
                    f"Planner comparison failed: {exc}"
                )
                self.send_error(500, "Comparison failed")
                return

            self._set_headers("application/json")
            self.wfile.write(
                json.dumps(result).encode("utf-8")
            )

        elif self.path == "/api/planner":
            if not global_node:
                self.send_error(503, "Node not ready")
                return

            algorithm = req_data.get("algorithm")
            if not isinstance(algorithm, str):
                self.send_error(400, "Invalid planner algorithm")
                return

            if not global_node.setPlannerAlgorithm(algorithm):
                self.send_error(400, "Invalid planner algorithm")
                return

            self._set_headers("application/json")
            self.wfile.write(
                json.dumps({
                    "status": "ok",
                    "algorithm": global_node.planner_algorithm_,
                }).encode("utf-8")
            )

        elif self.path == "/api/controller":
            if not global_node:
                self.send_error(503, "Node not ready")
                return

            mode = req_data.get("mode")
            if not isinstance(mode, str):
                self.send_error(400, "Invalid controller mode")
                return

            if not global_node.setControllerMode(mode):
                self.send_error(400, "Invalid controller mode")
                return

            self._set_headers("application/json")
            self.wfile.write(
                json.dumps({
                    "status": "ok",
                    "mode": global_node.controller_mode_,
                }).encode("utf-8")
            )

        elif self.path == "/api/mode":
            mode = str(req_data.get("mode", "manual"))
            if mode not in ("manual", "voice"):
                self.send_error(400, "Invalid mode"); return
            if global_node:
                global_node.setControlMode(mode)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "mode": mode}).encode("utf-8"))

        elif self.path == "/api/spawn_obstacle":
            try:
                gx = float(req_data.get("x", 0.0))
                gy = float(req_data.get("y", 0.0))
            except (ValueError, TypeError):
                self.send_error(400, "Invalid coordinates")
                return

            obs = {}
            if global_node:
                obs = global_node.spawnObstacle(gx, gy)
            self._set_headers("application/json")
            self.wfile.write(json.dumps({"status": "ok", "obstacle": obs}).encode("utf-8"))

        elif self.path == "/api/clear_obstacles":
            res = {"status": "ok", "cleared": 0}
            if global_node:
                res = global_node.clearAllObstacles()
            self._set_headers("application/json")
            self.wfile.write(json.dumps(res).encode("utf-8"))
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
