from math import hypot, atan2, inf, cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value

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

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False
        self.is_stopped_ = False
        self.rbt_x_ = 0.0
        self.rbt_y_ = 0.0
        self.rbt_yaw_ = 0.0
        self.path_poses_ = []

    # Callbacks =============================================================
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:
            self.path_poses_ = []
            self.received_path_ = False
            self.is_stopped_ = True
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

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot
        min_dist = inf
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            dist = hypot(pose.pose.position.x - self.rbt_x_, pose.pose.position.y - self.rbt_y_)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # From the closest point, iterate towards the goal and find the first point that is at least a lookahead distance away.
        # Return the goal point if no such lookahead point can be found
        lookahead_idx = len(self.path_poses_) - 1
        for i in range(closest_idx, len(self.path_poses_)):
            dist = hypot(self.path_poses_[i].pose.position.x - self.rbt_x_,
                         self.path_poses_[i].pose.position.y - self.rbt_y_)
            if dist >= self.lookahead_distance_:
                lookahead_idx = i
                break

        # Get the lookahead coordinates
        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Return the coordinates
        return lookahead_x, lookahead_y

    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        if self.is_stopped_ or not self.received_odom_ or not self.received_path_ or len(self.path_poses_) == 0:
            return  # return silently if stopped or path or odom is not received.

        # get lookahead point
        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        # stop the robot if close to the goal point
        goal_pose = self.path_poses_[-1].pose.position
        dist_to_goal = hypot(goal_pose.x - self.rbt_x_, goal_pose.y - self.rbt_y_)
        if dist_to_goal < self.stop_thres_:
            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = 0.0
            msg_cmd_vel.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            return

        # get displacement and distance to lookahead point
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

        # get coordinates in robot frame: y' = -dx * sin(yaw) + dy * cos(yaw)
        y_robot = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)

        # get curvature: c = 2 * y' / (L^2)
        curvature = 2.0 * y_robot / (L * L)

        # calculate velocities
        lin_vel = self.lookahead_lin_vel_
        ang_vel = lin_vel * curvature

        # saturate / clamp velocities
        lin_vel = max(-self.max_lin_vel_, min(lin_vel, self.max_lin_vel_))
        ang_vel = max(-self.max_ang_vel_, min(ang_vel, self.max_ang_vel_))

        # publish velocities
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
