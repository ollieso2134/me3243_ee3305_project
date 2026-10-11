from heapq import heappush, heappop
from math import hypot, floor, inf

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        if self.g == other.g:
            return (self.c, self.r) < (other.c, other.r)
        return self.g < other.g


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # Path request subscriber
        self.sub_path_request_ = self.create_subscription(
            Path, "path_request", self.callbackSubPathRequest_, 10
        )

        # Handles: Publishers
        # Path publisher
        self.pub_path_ = self.create_publisher(
            Path, "path", 10
        )

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False
        self.rbt_x_ = 0.0
        self.rbt_y_ = 0.0
        self.goal_x_ = 0.0
        self.goal_y_ = 0.0
        self.costmap_ = []
        self.costmap_resolution_ = 0.05
        self.costmap_origin_x_ = 0.0
        self.costmap_origin_y_ = 0.0
        self.costmap_rows_ = 0
        self.costmap_cols_ = 0

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):
        if len(msg.poses) < 2:
            return

        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y

        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        self.costmap_ = msg.data
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        c = int(floor((x - self.costmap_origin_x_) / self.costmap_resolution_))
        r = int(floor((y - self.costmap_origin_y_) / self.costmap_resolution_))
        return c, r

    # Converts cell column and cell row to world coordinates (cell center).
    def CRToXY_(self, c, r):
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_
        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        return c < 0 or c >= self.costmap_cols_ or r < 0 or r >= self.costmap_rows_

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        # Convert start and goal to grid coordinates
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)

        # Bounds checks
        if self.outOfMap_(rbt_c, rbt_r):
            self.get_logger().warn(
                f"Start position ({start_x:.3f}, {start_y:.3f}) is outside map bounds!"
            )
            return

        if self.outOfMap_(goal_c, goal_r):
            self.get_logger().warn(
                f"Goal position ({goal_x:.3f}, {goal_y:.3f}) is outside map bounds!"
            )
            return

        # Check if start or goal is on an obstacle (cost > max_access_cost)
        goal_idx = self.CRToIndex_(goal_c, goal_r)
        goal_cost = self.costmap_[goal_idx]
        if goal_cost < 0 or goal_cost > self.max_access_cost_:
            self.get_logger().warn(
                f"Goal ({goal_x:.3f}, {goal_y:.3f}) is inaccessible (cost={goal_cost} > {self.max_access_cost_})!"
            )
            return

        # If already at goal
        if rbt_c == goal_c and rbt_r == goal_r:
            msg_path = Path()
            msg_path.header.stamp = self.get_clock().now().to_msg()
            msg_path.header.frame_id = "map"
            pose = PoseStamped()
            pose.header = msg_path.header
            pose.pose.position.x = goal_x
            pose.pose.position.y = goal_y
            msg_path.poses.append(pose)
            self.pub_path_.publish(msg_path)
            return

        # Initializations
        start_node = DijkstraNode(rbt_c, rbt_r)
        start_node.g = 0.0
        open_list = []
        heappush(open_list, start_node)

        # Track best g-cost and expanded nodes
        min_g = {(rbt_c, rbt_r): 0.0}
        expanded = set()

        # 8-neighbor directions (cardinal and diagonal)
        neighbors = [
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
        ]

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:
            # Poll cheapest node
            node = heappop(open_list)

            # Skip if already expanded
            coord = (node.c, node.r)
            if coord in expanded:
                continue
            expanded.add(coord)

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                # Backtrack path
                curr = node
                path_poses = []
                while curr is not None:
                    px, py = self.CRToXY_(curr.c, curr.r)
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    path_poses.append(pose)
                    curr = curr.parent

                # Reverse path so start is at index 0 and goal is at the end
                path_poses.reverse()

                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"
                for p in path_poses:
                    p.header.stamp = msg_path.header.stamp
                msg_path.poses = path_poses

                # Publish path
                self.pub_path_.publish(msg_path)
                self.get_logger().info(
                    f"Dijkstra Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f}, {goal_y:7.3f}) with {len(path_poses)} waypoints."
                )
                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in neighbors:
                nb_c = node.c + dc
                nb_r = node.r + dr
                nb_coord = (nb_c, nb_r)

                # Skip if already expanded
                if nb_coord in expanded:
                    continue

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Ignore if cell cost exceeds max_access_cost or is unknown (-1)
                nb_idx = self.CRToIndex_(nb_c, nb_r)
                map_cost = self.costmap_[nb_idx]
                if map_cost < 0 or map_cost > self.max_access_cost_:
                    continue

                # Calculate step distance: Euclidean distance between node and neighbor
                step_dist = hypot(dc * self.costmap_resolution_, dr * self.costmap_resolution_)
                g_tilde = node.g + step_dist * (map_cost + 1)

                if g_tilde < min_g.get(nb_coord, inf):
                    min_g[nb_coord] = g_tilde
                    nb_node = DijkstraNode(nb_c, nb_r)
                    nb_node.g = g_tilde
                    nb_node.parent = node
                    heappush(open_list, nb_node)

        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
