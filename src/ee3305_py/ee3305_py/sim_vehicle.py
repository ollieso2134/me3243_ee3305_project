import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class SimVehicleNode(Node):
    def __init__(self, node_name="sim_vehicle"):
        super().__init__(node_name)

        self.declare_parameter("x", -2.0)
        self.declare_parameter("y", -0.5)
        self.declare_parameter("yaw", 0.0)

        self.x_ = float(self.get_parameter("x").value)
        self.y_ = float(self.get_parameter("y").value)
        self.yaw_ = float(self.get_parameter("yaw").value)

        self.lin_vel_ = 0.0
        self.ang_vel_ = 0.0
        self.last_cmd_time_ = self.get_clock().now()

        self.sub_cmd_ = self.create_subscription(
            TwistStamped, "cmd_vel", self.callbackCmdVel_, 10
        )
        self.pub_odom_ = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.dt_ = 0.05  # 20 Hz
        self.timer_ = self.create_timer(self.dt_, self.callbackTimer_)

        self.get_logger().info(
            f"Vehicle Kinematics Simulator active at ({self.x_:.2f}, {self.y_:.2f}, yaw={self.yaw_:.2f})"
        )

    def callbackCmdVel_(self, msg: TwistStamped):
        self.lin_vel_ = msg.twist.linear.x
        self.ang_vel_ = msg.twist.angular.z
        self.last_cmd_time_ = self.get_clock().now()

    def callbackTimer_(self):
        now = self.get_clock().now()
        # Safety timeout: halt vehicle if no command received for > 0.5s
        if (now - self.last_cmd_time_).nanoseconds > 500_000_000:
            self.lin_vel_ = 0.0
            self.ang_vel_ = 0.0

        # Unicycle kinematics integration
        self.x_ += self.lin_vel_ * math.cos(self.yaw_) * self.dt_
        self.y_ += self.lin_vel_ * math.sin(self.yaw_) * self.dt_
        self.yaw_ += self.ang_vel_ * self.dt_
        self.yaw_ = math.atan2(math.sin(self.yaw_), math.cos(self.yaw_))

        # Build Odometry message
        msg_odom = Odometry()
        msg_odom.header.stamp = now.to_msg()
        msg_odom.header.frame_id = "odom"
        msg_odom.child_frame_id = "base_footprint"

        msg_odom.pose.pose.position.x = self.x_
        msg_odom.pose.pose.position.y = self.y_
        msg_odom.pose.pose.position.z = 0.0

        half_yaw = self.yaw_ * 0.5
        msg_odom.pose.pose.orientation.x = 0.0
        msg_odom.pose.pose.orientation.y = 0.0
        msg_odom.pose.pose.orientation.z = math.sin(half_yaw)
        msg_odom.pose.pose.orientation.w = math.cos(half_yaw)

        msg_odom.twist.twist.linear.x = self.lin_vel_
        msg_odom.twist.twist.angular.z = self.ang_vel_

        self.pub_odom_.publish(msg_odom)

        # Broadcast TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = msg_odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0
        t.transform.rotation = msg_odom.pose.pose.orientation

        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
