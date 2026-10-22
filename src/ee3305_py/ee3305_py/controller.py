from math import hypot, atan2, inf, cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


from rcl_interfaces.msg import SetParametersResult


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Baseline Pure Pursuit Parameters
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.2))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))

        # Advanced Controller Mode: "standard" | "improved" | "rpp"
        self.declare_parameter("controller_mode", "rpp")

        # Regulated & Improved Parameters
        self.declare_parameter("min_lin_vel", float(0.05))
        self.declare_parameter("k_curv", float(0.6))
        self.declare_parameter("decel_dist", float(0.6))
        self.declare_parameter("rotate_threshold", float(0.785))  # ~45 deg
        self.declare_parameter("rotate_tolerance", float(0.26))   # ~15 deg
        self.declare_parameter("rotate_kp", float(1.8))
        self.declare_parameter("adaptive_lookahead", True)
        self.declare_parameter("min_lookahead", float(0.20))
        self.declare_parameter("max_lookahead", float(0.45))
        self.declare_parameter("lookahead_time", float(1.5))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value

        self.controller_mode_ = str(self.get_parameter("controller_mode").value).strip().lower()
        if self.controller_mode_ not in ("standard", "improved", "rpp"):
            self.controller_mode_ = "rpp"

        self.min_lin_vel_ = self.get_parameter("min_lin_vel").value
        self.k_curv_ = self.get_parameter("k_curv").value
        self.decel_dist_ = self.get_parameter("decel_dist").value
        self.rotate_threshold_ = self.get_parameter("rotate_threshold").value
        self.rotate_tolerance_ = self.get_parameter("rotate_tolerance").value
        self.rotate_kp_ = self.get_parameter("rotate_kp").value
        self.adaptive_lookahead_ = bool(self.get_parameter("adaptive_lookahead").value)
        self.min_lookahead_ = self.get_parameter("min_lookahead").value
        self.max_lookahead_ = self.get_parameter("max_lookahead").value
        self.lookahead_time_ = self.get_parameter("lookahead_time").value

        # Dynamic Reconfigure Callback
        self.add_on_set_parameters_callback(self.callbackParameters_)

        # Handles: Topic Subscribers
        # Path subscriber
        self.sub_path_ = self.create_subscription(
            Path, "path", self.callbackSubPath_, 10
        )

        # Stop / Emergency Stop subscriber
        self.sub_stop_ = self.create_subscription(
            Bool, "stop", self.callbackSubStop_, 10
        )

        # Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackSubOdom_, 10
        )

        # Controller mode switch subscriber (standard | improved | rpp)
        self.sub_controller_mode_ = self.create_subscription(
            String, "controller_mode", self.callbackSubControllerMode_, 10
        )

        # 2D LiDAR subscriber for frontal collision safety shield
        self.sub_scan_ = self.create_subscription(
            LaserScan, "scan", self.callbackSubScan_, qos_profile_sensor_data
        )

        # Handles: Topic Publishers
        # Command velocities publisher
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, "cmd_vel", 10
        )

        # Lookahead point publisher
        self.pub_lookahead_ = self.create_publisher(
            PoseStamped, "lookahead", 10
        )
        
        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Internal State Tracking
        self.received_odom_ = False
        self.received_path_ = False
        self.is_stopped_ = False
        self.is_rotating_to_heading_ = False
        self.current_lin_vel_ = 0.0
        self.rbt_x_ = 0.0
        self.rbt_y_ = 0.0
        self.rbt_yaw_ = 0.0
        self.path_poses_ = []

        # 2D LiDAR Frontal Safety Shield parameters
        self.frontal_obstacle_dist_ = inf
        self.safety_shield_active_ = False
        self.safety_shield_dist_ = 0.32  # meters threshold for active forward brake
        self.safety_shield_cone_ = 0.61  # ~35 deg half-angle frontal corridor

        self.get_logger().info(
            f"Controller Node initialized in [{self.controller_mode_.upper()}] mode with LiDAR Safety Shield."
        )

    # Dynamic Reconfigure Parameter Callback
    def callbackParameters_(self, params):
        for param in params:
            if param.name == "lookahead_distance":
                self.lookahead_distance_ = float(param.value)
                self.get_logger().info(f"[PARAM CHANGED] lookahead_distance = {self.lookahead_distance_:.3f} m")
            elif param.name == "lookahead_lin_vel":
                self.lookahead_lin_vel_ = float(param.value)
                self.get_logger().info(f"[PARAM CHANGED] lookahead_lin_vel = {self.lookahead_lin_vel_:.3f} m/s")
            elif param.name == "stop_thres":
                self.stop_thres_ = float(param.value)
                self.get_logger().info(f"[PARAM CHANGED] stop_thres = {self.stop_thres_:.3f} m")
            elif param.name == "max_lin_vel":
                self.max_lin_vel_ = float(param.value)
                self.get_logger().info(f"[PARAM CHANGED] max_lin_vel = {self.max_lin_vel_:.3f} m/s")
            elif param.name == "max_ang_vel":
                self.max_ang_vel_ = float(param.value)
                self.get_logger().info(f"[PARAM CHANGED] max_ang_vel = {self.max_ang_vel_:.3f} rad/s")
            elif param.name == "controller_mode":
                mode = str(param.value).strip().lower()
                if mode in ("standard", "improved", "rpp"):
                    self.controller_mode_ = mode
                    self.get_logger().info(f"[PARAM CHANGED] controller_mode = {self.controller_mode_.upper()}")
        return SetParametersResult(successful=True)

    # Callbacks =============================================================

    # Controller Mode selection callback
    def callbackSubControllerMode_(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in ("standard", "improved", "rpp"):
            self.get_logger().warn(f"Unsupported controller mode '{mode}'. Choose 'standard', 'improved', or 'rpp'.")
            return
        if mode == self.controller_mode_:
            return
        self.controller_mode_ = mode
        self.is_rotating_to_heading_ = False
        self.get_logger().info(f"Active Controller switched to: [{self.controller_mode_.upper()}]")
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:
            self.path_poses_ = []
            self.received_path_ = False
            self.is_stopped_ = True
            self.is_rotating_to_heading_ = False
            self.current_lin_vel_ = 0.0
            msg_stop = TwistStamped()
            msg_stop.header.stamp = self.get_clock().now().to_msg()
            msg_stop.twist.linear.x = 0.0
            msg_stop.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_stop)
            self.get_logger().info("Received empty path: controller halted.")
            return

        self.is_stopped_ = False
        self.path_poses_ = msg.poses
        self.received_path_ = True

    # Stop subscriber callback
    def callbackSubStop_(self, msg: Bool):
        if msg.data:
            self.path_poses_ = []
            self.received_path_ = False
            self.is_stopped_ = True
            self.is_rotating_to_heading_ = False
            self.current_lin_vel_ = 0.0
            msg_stop = TwistStamped()
            msg_stop.header.stamp = self.get_clock().now().to_msg()
            msg_stop.twist.linear.x = 0.0
            msg_stop.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_stop)
            self.get_logger().warn("Controller received STOP signal: cleared path and halted robot.")
        else:
            self.is_stopped_ = False

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        delta_y = 2.0 * (q.w * q.z + q.x * q.y)
        delta_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rbt_yaw_ = atan2(delta_y, delta_x)

        self.received_odom_ = True

    # 2D LiDAR subscriber callback: real-time frontal safety shield
    def callbackSubScan_(self, msg: LaserScan):
        angle = msg.angle_min
        angle_inc = msg.angle_increment
        min_frontal = inf

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                norm_angle = atan2(sin(angle), cos(angle))
                if abs(norm_angle) <= self.safety_shield_cone_:
                    if r < min_frontal:
                        min_frontal = r
            angle += angle_inc

        self.frontal_obstacle_dist_ = min_frontal
        self.safety_shield_active_ = (min_frontal < self.safety_shield_dist_)

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot
        min_dist = inf
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            dist = hypot(pose.pose.position.x - self.rbt_x_, pose.pose.position.y - self.rbt_y_)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Determine target lookahead distance
        if self.controller_mode_ == "rpp" and self.adaptive_lookahead_:
            # Regulated Pure Pursuit: velocity-scaled adaptive lookahead
            target_L = self.current_lin_vel_ * self.lookahead_time_ + self.min_lookahead_
            target_L = max(self.min_lookahead_, min(target_L, self.max_lookahead_))
        else:
            # Baseline & Improved: fixed configured lookahead distance
            target_L = self.lookahead_distance_

        # From closest point, scan forward to find first point >= target_L away
        lookahead_idx = len(self.path_poses_) - 1
        for i in range(closest_idx, len(self.path_poses_)):
            dist = hypot(self.path_poses_[i].pose.position.x - self.rbt_x_,
                         self.path_poses_[i].pose.position.y - self.rbt_y_)
            if dist >= target_L:
                lookahead_idx = i
                break

        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish lookahead marker for RViz2
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        return lookahead_x, lookahead_y

    # Main Control Loop
    def callbackTimer_(self):
        if self.is_stopped_ or not self.received_odom_ or not self.received_path_ or len(self.path_poses_) == 0:
            return

        goal_pose = self.path_poses_[-1].pose.position
        dist_to_goal = hypot(goal_pose.x - self.rbt_x_, goal_pose.y - self.rbt_y_)

        # Stop threshold arrival check
        if dist_to_goal < self.stop_thres_:
            self.current_lin_vel_ = 0.0
            self.is_rotating_to_heading_ = False
            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = 0.0
            msg_cmd_vel.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            return

        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        # Displacement vector in map frame
        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        L = hypot(dx, dy)

        if L < 1e-4:
            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = 0.0
            msg_cmd_vel.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            return

        # Heading error to lookahead point (normalized to [-pi, pi])
        angle_to_lookahead = atan2(dy, dx)
        heading_error = atan2(sin(angle_to_lookahead - self.rbt_yaw_), cos(angle_to_lookahead - self.rbt_yaw_))

        # Lateral displacement in robot frame: y' = -dx * sin(yaw) + dy * cos(yaw)
        y_robot = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)
        curvature = 2.0 * y_robot / (L * L)

        lin_vel = self.lookahead_lin_vel_
        ang_vel = 0.0

        # =====================================================================
        # CONTROLLER EXECUTION MODES
        # =====================================================================

        if self.controller_mode_ == "standard":
            # Baseline Pure Pursuit (Task 1): constant linear speed, unconstrained turns
            lin_vel = self.lookahead_lin_vel_
            ang_vel = lin_vel * curvature

        elif self.controller_mode_ == "improved":
            # Improved Pure Pursuit: Curvature-Adaptive Scaling + Smooth Arrival Deceleration
            # 1. Curvature-Adaptive Linear Velocity Scaling
            v_curv = max(self.min_lin_vel_, self.lookahead_lin_vel_ / (1.0 + self.k_curv_ * abs(curvature)))

            # 2. Smooth Goal Arrival Deceleration
            if dist_to_goal < self.decel_dist_:
                v_approach = max(self.min_lin_vel_, self.lookahead_lin_vel_ * (dist_to_goal / self.decel_dist_))
            else:
                v_approach = self.lookahead_lin_vel_

            lin_vel = min(self.lookahead_lin_vel_, v_curv, v_approach)
            ang_vel = lin_vel * curvature

        elif self.controller_mode_ == "rpp":
            # Regulated Pure Pursuit (RPP): Full Framework
            # Feature 1: Rotate-To-Heading on large initial orientation error
            if self.is_rotating_to_heading_:
                if abs(heading_error) <= self.rotate_tolerance_:
                    self.is_rotating_to_heading_ = False
                else:
                    lin_vel = 0.0
                    ang_vel = max(-self.max_ang_vel_, min(self.rotate_kp_ * heading_error, self.max_ang_vel_))

            elif abs(heading_error) > self.rotate_threshold_:
                self.is_rotating_to_heading_ = True
                lin_vel = 0.0
                ang_vel = max(-self.max_ang_vel_, min(self.rotate_kp_ * heading_error, self.max_ang_vel_))

            if not self.is_rotating_to_heading_:
                # Feature 2: Curvature Regulation (Adaptive decay + turning radius limit)
                v_curv = max(self.min_lin_vel_, self.lookahead_lin_vel_ / (1.0 + self.k_curv_ * abs(curvature)))
                # Enforce kinematic turn radius limit: v <= (max_ang_vel * 0.85) / (|curvature| + 1e-5)
                kinematic_limit = (self.max_ang_vel_ * 0.85) / (abs(curvature) + 1e-5)
                v_curv = min(v_curv, max(self.min_lin_vel_, kinematic_limit))

                # Feature 3: Proportional Goal Approach Deceleration
                if dist_to_goal < self.decel_dist_:
                    v_approach = max(self.min_lin_vel_, self.lookahead_lin_vel_ * (dist_to_goal / self.decel_dist_))
                else:
                    v_approach = self.lookahead_lin_vel_

                lin_vel = min(self.lookahead_lin_vel_, v_curv, v_approach)
                ang_vel = lin_vel * curvature

        # 2D LiDAR Frontal Safety Shield: Clamp forward motion if obstacle in front
        if self.safety_shield_active_ and lin_vel > 0.0:
            lin_vel = 0.0
            self.get_logger().warn(
                f"[SAFETY SHIELD] Obstacle detected {self.frontal_obstacle_dist_:.2f} m ahead! Braking linear drive, pivoting to clear.",
                throttle_duration_sec=1.0,
            )

        # Velocity saturation and publication
        lin_vel = max(-self.max_lin_vel_, min(lin_vel, self.max_lin_vel_))
        ang_vel = max(-self.max_ang_vel_, min(ang_vel, self.max_ang_vel_))
        self.current_lin_vel_ = lin_vel

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
