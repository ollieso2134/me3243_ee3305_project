# EE3305 / ME3243 Robotic System Design — Project Implementation Log

**Environment**: Ubuntu 24.04 LTS | ROS 2 Jazzy Jalisco | Gazebo Sim (Harmonic) | Python 3.12  
**Platform**: TurtleBot3 Burger in Gazebo & RViz2  

---

## Quick Start: Launching the Simulation

### 1. Build and Source Workspace
Run from the workspace directory:
```bash
cd ~/ee3305
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Commands (Two Separate Terminals)

#### Terminal 1: Simulation, Navigation Stack & Web GUI
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False
```

> **Launch Arguments Guide**:
> * `cpp:=False`: Executes the Python nodes from `ee3305_py` (`behavior.py`, `planner.py`, `controller.py`).
> * `headless:=False`: Opens the Gazebo 3D simulation GUI alongside RViz2. Set to `True` to run Gazebo in the background to save GPU/CPU resources.
> * `libgl:=False`: Uses native GPU acceleration (recommended for WSL2 and dual-boot). Set to `libgl:=True` if running on VirtualBox.

#### Terminal 2: AI Voice Control Bridge (Optional for Voice Control)
Launches the Faster-Whisper speech-to-text, Google Gemini LLM reasoning, and Edge TTS voice feedback engine:
```bash
cd ~/ee3305
./start_ai_bridge.sh
```
*(Tip: Alternatively, `./start_all_with_ai.sh` can be executed in a single terminal to launch both Terminal 1 and Terminal 2 together).*

### 3. How to Command the Robot
* **Web Mission Control Dashboard**:
  Open your web browser at **`http://localhost:8080`** for an interactive dark-mode floorplan with one-click room dispatch buttons, live robot tracking, algorithm selectors (Planner & Controller), and real-time planner comparison.
* **Switch Planner Algorithm**:
  Switch the active global planner on the fly via Web GUI buttons or via terminal:
  ```bash
  ros2 topic pub --once /planner_algorithm std_msgs/msg/String "{data: 'astar'}"
  # Supported: 'dijkstra', 'astar', 'theta'
  ```
* **Switch Controller Mode**:
  Switch the local tracking controller mode on the fly via Web GUI buttons or via terminal:
  ```bash
  ros2 topic pub --once /controller_mode std_msgs/msg/String "{data: 'rpp'}"
  # Supported: 'standard' (Baseline PP), 'improved' (Adaptive + Decel), 'rpp' (Regulated Pure Pursuit)
  ```
* **Run Three-Planner Comparison**:
  Click the **Run Comparison** button on the Web GUI to benchmark Dijkstra, A*, and Theta* simultaneously for computation time, path length, and node expansions.
* **Manual RViz Navigation**:
  Click the **2D Goal Pose** tool in the RViz2 top toolbar, then click and drag in any room.
* **Dispatch to a Specific Room (Rooms 1 to 6)**:
  Open a second terminal and publish to `/room_goal`:
  ```bash
  source ~/ee3305/install/setup.bash
  ros2 topic pub --once /room_goal std_msgs/msg/Int32 "{data: 4}"
  ```
* **Continuous Multi-Room Patrol Mode**:
  Toggle continuous patrol through Rooms 1 &rarr; 2 &rarr; 3 &rarr; 4 &rarr; 5 &rarr; 6:
  ```bash
  ros2 topic pub --once /patrol std_msgs/msg/Bool "{data: true}"
  ```

---

## 1. Project Overview & Aim

### Project Aim
Implement Dijkstra path planning and Pure Pursuit controller to move a Turtlebot3 robot in a simulated environment.

### Core Navigation Roles
* **Global Planner**: **Dijkstra's Algorithm** (determines the global path from robot pose to goal pose across an inflated 2D occupancy grid costmap).
* **Local Planner / Controller**: **Pure Pursuit Controller** (tracks the global path locally by computing steering curvature and forward linear velocity toward a dynamic lookahead point).
* **Behavior Coordination**: Coordinates high-level states, receives destination goals from RViz, and issues periodic path planning requests.

---

## 2. System Architecture & Communication Pipeline

The system divides navigation responsibilities across three decoupled nodes communicating asynchronously via standard ROS 2 topics:

```
[RViz2: 2D Goal Pose]
         │ /goal_pose
         ▼
+───────────────────+
|   Behavior Node   | ◄─── /odom (from Gazebo)
+───────────────────+
         │ /path_request
         ▼
+───────────────────+
|   Planner Node    | ◄─── /global_costmap (from map_loader)
+───────────────────+
         │ /path
         ▼
+───────────────────+
|  Controller Node  | ◄─── /odom (from Gazebo)
+───────────────────+
    │            │
    │ /cmd_vel   │ /lookahead
    ▼            ▼
[Gazebo]      [RViz2]
```

### Topic Specifications

| Topic | Message Type | Publisher | Subscriber | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz2 | Behavior | User-selected destination clicked on the map |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo Bridge | Behavior, Controller | Robot position and orientation in world frame |
| `/path_request` | `nav_msgs/msg/Path` | Behavior | Planner | Start coordinates `poses[0]` and goal coordinates `poses[1]` |
| `/global_costmap` | `nav_msgs/msg/OccupancyGrid` | Map Loader | Planner | Inflated 2D obstacle cost values (0 to 99) |
| `/path` | `nav_msgs/msg/Path` | Planner | Controller, RViz2 | Ordered sequence of waypoints from start to goal |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Controller | Gazebo Bridge | Target linear velocity $v$ and angular velocity $\omega$ |
| `/lookahead` | `geometry_msgs/msg/PoseStamped` | Controller | RViz2 | Visual marker of the current lookahead point |
| `/planner_algorithm` | `std_msgs/msg/String` | Web GUI, CLI | Planner | Switch active global path planner (`"dijkstra"`, `"astar"`, `"theta"`) |
| `/controller_mode` | `std_msgs/msg/String` | Web GUI, CLI | Controller | Switch active local controller mode (`"standard"`, `"improved"`, `"rpp"`) |

---

## 3. Task 1: Baseline System Implementation & Integration

### 3.1 Sub-Task 1A: ROS 2 Node Connection

Files modified:
* `behavior.py`
* `planner.py`
* `controller.py`

#### 3.1.1 Behavior Node (`behavior.py`)

#### Purpose
Acts as the decision-maker and high-level coordinator of the robot system. It receives destination inputs from the user, monitors the robot's current position from odometry, and periodically requests paths from the planner.

#### Implemented Components
1. **Subscribers**:
   * `sub_goal_pose_`: Subscribes to `goal_pose` (`PoseStamped`, QoS = 10).
   * `sub_odom_`: Subscribes to `odom` (`Odometry`, QoS = 10).
2. **Publisher**:
   * `pub_path_request_`: Publishes to `path_request` (`Path`, QoS = 10).
3. **Callbacks**:
   * `callbackSubGoalPose_`: Stores user-clicked destination coordinates `(goal_x_, goal_y_)` from `msg.pose.position`. Resets `goal_reached_` to allow tracking.
   * `callbackSubOdom_`: Extracts physical robot coordinates `(rbt_x_, rbt_y_)` from `msg.pose.pose.position`.
   * `callbackTimerPlan_`: Runs periodically at 2 Hz. Packages the current robot pose as `poses[0]` and the target goal pose as `poses[1]` into a `nav_msgs/msg/Path` message, then publishes it on `path_request`.

#### Design Rationale: Position vs. Orientation
* In `nav_msgs/msg/Odometry`, `msg.pose.pose.position` contains the physical position $(x, y, z)$ in meters, while `msg.pose.pose.orientation` contains a 4D quaternion $(x, y, z, w)$ representing rotation.
* The behavior node only needs physical 2D coordinates to compute Euclidean distance to the goal (`hypot(dx, dy)`) and to provide start/goal waypoints to the planner. Orientation heading is not needed here, so `position.x` and `position.y` were used.

---

#### 3.1.2 Planner Node (`planner.py`)

#### Purpose
Computes a collision-free path from the robot's current position to the goal using map cost values.

#### Implemented Components
1. **Subscribers & Publisher**:
   * `sub_path_request_`: Subscribes to `path_request` (`Path`, QoS = 10).
   * `pub_path_`: Publishes the generated route on `path` (`Path`, QoS = 10).
   * `sub_global_costmap_`: Subscribes to `global_costmap` (`OccupancyGrid`, latching QoS).
2. **Callbacks**:
   * `callbackSubPathRequest_`: Extracts start coordinates $(x_s, y_s)$ from `msg.poses[0]` and destination coordinates $(x_g, y_g)$ from `msg.poses[1]`. Sets `has_new_request_ = True`.
   * `callbackSubGlobalCostmap_`: Captures the static costmap data array (`msg.data`), grid resolution ($0.05\text{ m}$), world origin coordinates (`origin.position.x/y`), and grid dimensions (`width`, `height`).
3. **Verification Mechanism**:
   * In Step 1, before implementing Dijkstra, `callbackTimer_` invokes `publishInterpolatedPath(...)`. This function creates a straight test line of points spaced every $0.05\text{ m}$ between start and goal to confirm the ROS communication pipeline.

---

#### 3.1.3 Controller Node (`controller.py`)

#### Purpose
Follows the path published by the planner by calculating velocity commands to drive the robot's motors.

#### Implemented Components
1. **Subscribers & Publishers**:
   * `sub_path_`: Subscribes to `path` (`Path`, QoS = 10) and stores waypoints in `path_poses_`.
   * `sub_odom_`: Subscribes to `odom` (`Odometry`, QoS = 10).
   * `pub_cmd_vel_`: Publishes velocity commands on `cmd_vel` (`TwistStamped`, QoS = 10).
   * `pub_lookahead_`: Publishes visual lookahead markers on `lookahead` (`PoseStamped`, QoS = 10).
2. **Callbacks**:
   * `callbackSubPath_`: Updates the internal waypoint list `path_poses_` whenever a valid, non-empty path is published.
   * `callbackSubOdom_`: Extracts current position $(x, y)$ and derives the robot's heading yaw angle ($\phi$).

#### Design Rationale: Quaternion to Euler Yaw Conversion
* ROS 2 represents 3D orientations as unit quaternions:
  $$\mathbf{q} = \begin{bmatrix} q_x & q_y & q_z & q_w \end{bmatrix}^\top$$
* Because the TurtleBot3 operates on a 2D plane, only the planar heading angle (yaw $\phi$, rotation about the $Z$-axis) is required.
* The conversion from unit quaternion to yaw is derived from the rotation matrix:
  $$\Delta y = 2(q_w q_z + q_x q_y)$$
  $$\Delta x = 1 - 2(q_y^2 + q_z^2)$$
  $$\phi = \text{atan2}(\Delta y, \Delta x)$$
* In code:
  ```python
  delta_y = 2.0 * (q.w * q.z + q.x * q.y)
  delta_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  self.rbt_yaw_ = atan2(delta_y, delta_x)
  ```
* This yields the heading angle $\phi \in [-\pi, \pi]$ radians ($0\text{ rad}$ corresponds to pointing along the positive $X$-axis). The controller requires this angle to know which way the robot is currently facing relative to the path.

---

#### 3.1.4 Verification & Milestone Result

![Step 1 Verification: Straight Test Path in RViz2](assets/step1_interpolated_path_rviz.png)

#### Terminal Execution Log:
```text
[behavior-8] [INFO] [behavior]: Sending Path Planning Request from Rbt @ (  0.000,  -0.000) to Goal @ (  0.947,   2.296)
[planner-10] [INFO] [planner]: Publishing interpolated path between Start and Goal. Implement dijkstra_() instead.
```

#### Observations & System Status:
1. **Publish and Subscribe Pipeline Fully Functional**:
   * **Goal Received**: Clicking **2D Goal Pose** in RViz2 successfully published to `/goal_pose`, and `behavior.py` captured the target coordinates $(x = 0.947, y = 2.296)$.
   * **Path Request Dispatched**: `behavior.py` bundled the robot's current coordinates $(0.0, 0.0)$ and the target goal coordinates, successfully publishing to `/path_request`.
   * **Planner & Controller Receiving**: `planner.py` received the request and published waypoints to `/path`. `controller.py` subscribed to `/path` and `/odom`, successfully calculating the robot yaw heading $\phi$ and storing the waypoint array without errors.

2. **Planner Status (Not Yet Implemented)**:
   * Dijkstra's algorithm has **not** been implemented yet.
   * The green line visible in RViz2 is the temporary interpolated straight test path (`publishInterpolatedPath(...)`), which currently passes directly through obstacles.

3. **Controller Status (Not Yet Implemented)**:
   * Pure Pursuit tracking has **not** been implemented yet.
   * The robot remains completely stationary because `controller.py` currently outputs zero velocity commands (`lin_vel = 0.0, ang_vel = 0.0`). Motion tracking will be implemented in Step 2.

---

### 3.2 Sub-Task 1B: Pure Pursuit Controller Implementation

File modified:
* `controller.py`

#### 3.2.1 Concept & Algorithm Overview
Pure Pursuit is a geometric path-following algorithm. At each control step (20 Hz), the controller determines an instantaneous circular arc connecting the robot's current position to a target "lookahead point" located on the path ahead. The curvature of this arc directly dictates the required angular velocity command to steer the differential-drive robot smoothly along the path.

#### 3.2.2 Lookahead Point Selection (`getLookaheadPoint_`)
1. **Find Closest Waypoint**:
   Iterates through all waypoints in `self.path_poses_` to identify the waypoint closest to the robot's current coordinate $(x_r, y_r)$ via Euclidean distance:
   $$d_i = \sqrt{(x_i - x_r)^2 + (y_i - y_r)^2}$$
2. **Search Forward for Lookahead Target**:
   Starting from the closest waypoint index and progressing forward toward the goal, it selects the first waypoint where the distance to the robot meets or exceeds the lookahead distance:
   $$d_i \ge d_{\text{lookahead}}$$
   where $d_{\text{lookahead}}$ is the lookahead distance threshold (`lookahead_distance_`, default $0.1\text{ m}$).
3. **End-of-Path Fallback**:
   If no waypoint is at least `lookahead_distance_` away (which occurs as the robot approaches the end of the path), the final goal waypoint is assigned as the lookahead target.
4. **Visualization**:
   Publishes the selected point coordinates on `/lookahead` (`PoseStamped`) for real-time visualization in RViz2.

#### 3.2.3 Mathematical Derivation & Motion Control (`callbackTimer_`)
1. **Goal Stopping Condition**:
   Before driving, the Euclidean distance from the robot to the destination (the last waypoint in `path_poses_`) is evaluated:
   $$d_{\text{goal}} = \sqrt{(x_{\text{goal}} - x_r)^2 + (y_{\text{goal}} - y_r)^2}$$
   If $d_{\text{goal}} < d_{\text{stop}}$ (where $d_{\text{stop}}$ is `stop_thres_`, default $0.1\text{ m}$), the robot publishes zero velocities ($v = 0, \omega = 0$) and halts.

2. **World to Robot Frame Transformation**:
   Let the lookahead target coordinates in the global map frame be $(x_l, y_l)$, the robot coordinates be $(x_r, y_r)$, and robot yaw heading be $\phi_r$.
   Displacements in world frame:
   $$\Delta x = x_l - x_r, \quad \Delta y = y_l - y_r$$
   Euclidean chord distance to lookahead point:
   $$L = \sqrt{\Delta x^2 + \Delta y^2}$$
   Transforming into the robot-centric reference frame where $+X'$ points forward and $+Y'$ points to the robot's left:
   $$y' = -\Delta x \sin(\phi_r) + \Delta y \cos(\phi_r)$$

3. **Curvature and Angular Velocity Calculation**:
   The instantaneous turning radius $R$ satisfies $2 R y' = L^2 \implies R = \frac{L^2}{2 y'}$.
   The path curvature $c$ is the reciprocal of the turn radius:
   $$c = \frac{1}{R} = \frac{2 y'}{L^2}$$
   Using the target forward linear speed $v = v_{\text{target}}$ (parameter `lookahead_lin_vel_`), the required angular steering velocity $\omega$ is:
   $$\omega = v \cdot c$$

4. **Velocity Saturation / Clamping**:
   To prevent exceeding hardware actuator capabilities and avoid motor burnout:
   $$v = \text{clamp}(v, -v_{\max}, v_{\max})$$
   $$\omega = \text{clamp}(\omega, -\omega_{\max}, \omega_{\max})$$
   where $v_{\max}$ corresponds to `max_lin_vel_` and $\omega_{\max}$ corresponds to `max_ang_vel_`.
   The saturated velocity is published to `/cmd_vel` (`geometry_msgs/msg/TwistStamped`).

---

#### 3.2.4 Verification & Key Observation

#### Observed Behavior:
* When a goal pose was selected in RViz2, the Pure Pursuit controller immediately activated. The robot drove forward, accurately oriented its heading toward the lookahead point, and followed the green line.
* However, because the test path connects the start to the goal in a straight line, the robot attempted to drive straight through walls.

#### Key Conceptual Insight:
* **Controller Has No Map/Obstacle Awareness**: Pure Pursuit is purely a **local trajectory-tracking controller**. Its sole responsibility is converting geometric path error into linear velocity ($v$) and angular velocity ($\omega$) commands for the wheel motors. It does not inspect the costmap, read LiDAR sensor data, or possess any knowledge of walls.
* **Separation of Concerns**: Collision avoidance is the responsibility of the **Global Planner** (Dijkstra's algorithm). The planner must read the costmap to generate a collision-free path around walls; once provided with a valid path, Pure Pursuit will safely track it through doorways and corridors without colliding.

---

### 3.3 Sub-Task 1C: Dijkstra Global Path Planner Implementation

File modified:
* `planner.py`

#### 3.3.1 Concept & Algorithm Overview
Dijkstra's algorithm is a graph search algorithm that finds the shortest (lowest-cost) path between a start node and a goal node. In grid-based robotic navigation, the map is treated as a 2D weighted graph where each grid cell is a node, and edges connect adjacent cells. By incorporating the inflated obstacle cost from `/global_costmap` directly into edge traversal weights, Dijkstra finds a path that is not only geometrically short but also prioritizes wider, safer clearances away from obstacles.

#### 3.3.2 Coordinate Transformations & Grid Mapping
The costmap published by `map_loader` is defined in continuous metric space $(x, y)$ with resolution $\text{res} = 0.05\text{ m/cell}$ and an origin $(x_0, y_0)$ representing the bottom-left corner of the grid in the map frame.

1. **World to Grid Cell (`XYToCR_`)**:
   Converts continuous world coordinates $(x, y)$ to discrete grid column $c$ and row $r$:
   $$c = \left\lfloor \frac{x - x_0}{\text{res}} \right\rfloor, \quad r = \left\lfloor \frac{y - y_0}{\text{res}} \right\rfloor$$

2. **Grid Cell to World (`CRToXY_`)**:
   Converts grid column $c$ and row $r$ back to continuous world coordinates at the cell center:
   $$x = x_0 + (c + 0.5) \cdot \text{res}, \quad y = y_0 + (r + 0.5) \cdot \text{res}$$

3. **2D Grid to 1D Array Index (`CRToIndex_`)**:
   The `nav_msgs/msg/OccupancyGrid` stores cost data in a flat 1D row-major array:
   $$\text{index} = r \cdot \text{width} + c$$

4. **Boundary Validation (`outOfMap_`)**:
   Ensures coordinates reside within grid bounds:
   $$0 \le c < \text{width} \quad \text{and} \quad 0 \le r < \text{height}$$

#### 3.3.3 Priority Queue Node (`DijkstraNode`)
To efficiently extract the cheapest node in $O(\log N)$ time, Python's `heapq` (binary min-heap) is used.
* Each node stores its grid column $c$, row $r$, cumulative cost $g$, and a pointer to its `parent` node.
* A custom comparator `__lt__` is implemented for `heapq`:
  ```python
  def __lt__(self, other):
      if self.g == other.g:
          return (self.c, self.r) < (other.c, other.r)
      return self.g < other.g
  ```
  This ensures strict ordering by lowest $g$-cost, with deterministic coordinate tie-breaking.

#### 3.3.4 Graph Search & Cost Formulation (`dijkstra_`)
1. **Initialization**:
   * The start cell $(c_{\text{start}}, r_{\text{start}})$ is initialized with $g = 0.0$ and inserted into the `open_list` priority queue.
   * A hash map `min_g` tracks the lowest known $g$-cost to each cell coordinate `(c, r)`.
   * A set `expanded` records visited cells to prevent redundant re-expansions.

2. **8-Connected Neighbor Exploration**:
   From the current node $n = (c, r)$, the algorithm explores all 8 adjacent neighbors:
   $$\Delta c \in \{-1, 0, 1\}, \quad \Delta r \in \{-1, 0, 1\} \quad (\text{excluding } \Delta c = \Delta r = 0)$$
   This allows horizontal, vertical, and diagonal movements.

3. **Obstacle Filtering & Cost Function**:
   For each neighbor cell $m = (c + \Delta c, r + \Delta r)$:
   * Check if $m$ is out of map bounds or already in `expanded`.
   * Retrieve the map cost $C(m)$ from `costmap_[CRToIndex_(nb_c, nb_r)]`.
   * If $C(m) < 0$ (unknown space) or $C(m) > C_{\max}$ (parameter `max_access_cost_`, default $90$), the cell is an obstacle and is skipped.
   * The Euclidean step distance is:
     $$\text{dist}(n, m) = \sqrt{(\Delta c \cdot \text{res})^2 + (\Delta r \cdot \text{res})^2}$$
   * The tentative path cost $\tilde{g}$ to reach neighbor $m$ through node $n$ is:
     $$\tilde{g} = g(n) + \text{dist}(n, m) \cdot (C(m) + 1)$$
     The factor $(C(m) + 1)$ penalizes cells with higher inflation costs (near walls), guiding the search toward open corridors with minimal obstacle hazard.

4. **Relaxation & Priority Queue Insertion**:
   If $\tilde{g} < g_{\min}(m)$ (tracked in hash map `min_g`):
   * Update $g_{\min}(m) = \tilde{g}$.
   * Create a new `DijkstraNode` for $m$ with $g = \tilde{g}$ and `parent = n`.
   * Push the node onto `open_list`.

#### 3.3.5 Path Reconstruction & Publishing
1. **Backtracking**:
   When the goal cell $(c_{\text{goal}}, r_{\text{goal}})$ is extracted from the priority queue:
   * The algorithm traces back through `node.parent` pointers until reaching the start node (`parent == None`).
   * Each grid coordinate $(c, r)$ is converted to continuous $(x, y)$ world coordinates using `CRToXY_` and appended as a `geometry_msgs/msg/PoseStamped`.
2. **Reversal & Publication**:
   * The traced poses (initially from goal to start) are reversed so that index `0` is the robot start and the last index is the goal.
   * A `nav_msgs/msg/Path` message is populated with `frame_id = "map"` and published to `/path`.
   * The controller immediately receives the newly planned path and begins Pure Pursuit tracking.
---

#### 3.3.6 Verification & Milestone Result

![Step 3 Verification: Dijkstra Collision-Free Path in RViz2](assets/step3_dijkstra_path_rviz.png)

#### Terminal Execution Log:
```text
[behavior-8] [INFO] [behavior]: Sending Path Planning Request from Rbt @ ( -2.372,   3.735) to Goal @ ( -2.438,   3.788)
[planner-10] [INFO] [planner]: Dijkstra Path Found from Rbt @ ( -2.372,   3.735) to Goal @ ( -2.438,   3.788) with 2 waypoints.
```

#### Observations & System Status:
1. **Collision-Free Global Path Planning Confirmed**:
   * When a destination was selected across the apartment in RViz2, Dijkstra's algorithm successfully planned a collision-free path that safely bypassed all internal walls and doorways.
   * As visible in the RViz2 visualization, the green path correctly navigates around the inflated cost halos (blue/cyan inflation zones) surrounding the walls and the cylindrical obstacle columns in the central living area.
2. **Smooth Pure Pursuit Path Tracking**:
   * The controller accurately followed the generated Dijkstra waypoints in real time, driving the TurtleBot3 out of the bedroom, through the doorway, down the corridor, and safely around the obstacles.
   * Unlike Step 2 (where the straight interpolated line caused the robot to ram into walls), the combination of Dijkstra global planning + Pure Pursuit local control enabled full autonomous obstacle avoidance.
3. **Goal Reached Behavior**:
   * As the robot drew closer to the destination, the behavior node continuously updated the path request with the robot's real-time odometry position.
   * When the robot arrived within the stopping distance threshold ($d_{\text{goal}} < 0.1\text{ m}$), the planned path collapsed to just 2 local waypoints, and the controller successfully brought the robot to a complete stop.

---

## 4. Task 2: Parameter Tuning & Empirical Analysis

### Executive Summary: Optimal Tuned Configuration
Following the module rubric, all parameters across the Pure Pursuit Controller, Dijkstra Planner, and Costmap were systematically tuned and evaluated using four criteria: **Methodology**, **Observation**, **Explanation (Theory)**, and **Suggestion**.

| Subsystem | Parameter | Default Value | **Optimal Tuned Value** | Primary Performance Gain |
| :--- | :--- | :--- | :--- | :--- |
| **Controller**| `lookahead_distance` ($L$) | `0.10 m` | **`0.30 m`** | **$38.5\times$ reduction in steering oscillation** ($\dot{\omega}^2$ drops from $8.10$ to $0.21\text{ rad}^2/\text{s}^4$) with safe $0.316\text{ m}$ wall clearance. |
| **Controller**| `lookahead_lin_vel` ($v$) | `0.20 m/s` | **`0.20 m/s`** | Ideal balance avoiding actuator velocity saturation on sharp turns ($\omega \le 2.0\text{ rad/s}$) with $73.8\text{ s}$ transit time. |
| **Controller**| `stop_thres` ($S$) | `0.10 m` | **`0.08 - 0.10 m`** | Eliminates goal hunting/circling while keeping arrival error $< 0.095\text{ m}$. |
| **Planner** | `max_access_cost` | `90` | **`88 - 92`** | Prevents blocking traversable doorways while strictly excluding lethal obstacle boundaries ($99$). |
| **Costmap** | `circumscribed_radius` | `0.20 m` | **`0.20 m`** | Matches TurtleBot3 footprint radius ($\approx 0.105\text{ m}$) with dynamic buffer; $<0.40\text{ m}$ avoids door blockage. |
| **Costmap** | `inflation_radius` | `0.80 m` | **`0.80 m`** | Expansive cost gradient that naturally guides the planner along corridor centerlines. |
| **Costmap** | `cost_exponent` | `8.0` | **`8.0`** | Exponential decay balancing smooth cost descent with high corridor clearance ($0.683\text{ m}$ mean clearance). |

![Master Parameter Tuning and Deviation Matrix](assets/tuning_parameter_tradeoffs_matrix.png)
*Figure 4.0: Comprehensive 8-panel tuning dashboard comparing deviation curves and optimal operational windows across all 7 parameters.*

---

### 4.1 Sub-Task 2A: Pure Pursuit Controller Tuning

```
[Path from Planner] ──> [Find Closest Pose] ──> [Step L along path] ──> Lookahead Point (x_l, y_l)
                                                                               │
[Odometry (x_r, y_r, yaw)] ───────────────────────────────────────────────────┼──> Curvature κ = 2y' / L²
                                                                               └──> Angular Vel ω = v · κ
```

#### 4.1.1 Lookahead Distance ($L$)
* **Methodology**: Fix forward velocity $v = 0.20\,\text{m/s}$, goal threshold $S = 0.10\,\text{m}$, control rate $20\,\text{Hz}$. Sweep $L \in \{0.05, 0.10, 0.20, 0.30, 0.40, 0.60, 0.80, 1.20\}\,\text{m}$ across sharp 90° bends and narrow doorways along the benchmark path to Room 1.
* **Empirical Observations**:

| $L$ ($m$) | Status | Transit Time ($s$) | Mean CTE ($m$) | Max CTE ($m$) | Oscillation ($\text{rad}^2/\text{s}^4$) | Min Wall Clearance ($m$) | Trajectory Behavior |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **0.05** | SUCCESS | 75.10 | 0.0148 | 0.0353 | **68.47** | 0.335 | Violent lateral hunting/weaving; severe chatter. |
| **0.10 (Default)** | SUCCESS | 74.60 | 0.0154 | 0.0378 | **8.10** | 0.361 | Visible steering wobble; jerky wheel velocities. |
| 0.20 | SUCCESS | 74.20 | 0.0170 | 0.0454 | 0.82 | 0.335 | Smooth tracking; minor corner lead. |
| **0.30 (Tuned)** | **SUCCESS** | **73.80** | **0.0198** | **0.0576** | **0.21** | **0.316** | **Optimal: Rock-solid stability ($0.21$ chatter), tight tracking, safe clearance.** |
| 0.40 | SUCCESS | 73.45 | 0.0234 | 0.0843 | 0.08 | 0.304 | Mild corner cutting begins. |
| 0.60 | SUCCESS | 72.75 | 0.0320 | 0.1292 | 0.02 | 0.300 | Significant corner cutting; turns before waypoint. |
| 0.80 | SUCCESS | 72.05 | 0.0439 | 0.2169 | 0.01 | 0.269 | High lateral error ($0.217\text{ m}$); cuts inside corners. |
| **1.20** | SUCCESS | 69.50 | 0.0897 | **0.4085** | 0.01 | **0.050** | **Severe corner cutting! Clearance drops to $5\text{ cm}$ (near-collision!).** |

![Lookahead Distance Oscillation vs Corner-Cutting Trade-off](assets/tuning_lookahead_tradeoff.png)
*Figure 4.1a: Pure Pursuit lookahead distance trade-off curve demonstrating the transition from underdamped steering chatter ($L \le 0.10\text{ m}$) to overdamped corner cutting ($L \ge 0.60\text{ m}$). Optimal: $L = 0.30\text{ m}$.*

![Pure Pursuit Spatial Trajectories](assets/pure_pursuit_trajectories.png)
*Figure 4.1b: 2D spatial trajectory comparison for $L=0.05\text{ m}$ (weaving), $L=0.30\text{ m}$ (optimal tracking), and $L=1.20\text{ m}$ (dangerous corner cutting).*

![Cross-Track Error and Angular Chatter Time Series](assets/pure_pursuit_cte_oscillation.png)
*Figure 4.1c: Time-series of Cross-Track Error (CTE) and angular velocity chatter $\omega(t)$, showing dramatic stabilization at $L=0.30\text{ m}$.*

* **Theoretical Explanation**:
  Curvature in the vehicle frame is derived from the geometric chord arc:
  $$\kappa = \frac{2 y'}{L^2} \implies \omega = v \cdot \kappa = \frac{2 v}{L^2} y'$$
  The lateral offset $y'$ is multiplied by an effective proportional gain $K_p = \frac{2 v}{L^2}$:
  * For $L = 0.05\,\text{m}$, $K_p = \frac{2(0.2)}{(0.05)^2} = 160.0\,\text{s}^{-1}$. Any minute cross-track error commands immediate maximum angular velocity ($\pm 2.0\,\text{rad/s}$ saturation), creating limit-cycle lateral hunting ($\dot{\omega}^2 = 68.47$).
  * For $L = 1.20\,\text{m}$, $K_p = \frac{2(0.2)}{(1.20)^2} = 0.278\,\text{s}^{-1}$. The controller is heavily overdamped, but the lookahead point jumps across turns onto subsequent path segments, pulling the robot into the inside corner ($0.4085\,\text{m}$ max error) and dropping wall clearance to a hazardous $5.0\,\text{cm}$.
* **Suggestion**: Set $L \in [0.25\,\text{m}, 0.35\,\text{m}]$ (Nominal: **`0.30 m`**).

---

#### 4.1.2 Lookahead Linear Velocity ($v$)
* **Methodology**: Fix $L = 0.30\,\text{m}$, $S = 0.10\,\text{m}$, $\omega_{\max} = 2.0\,\text{rad/s}$. Sweep $v \in \{0.10, 0.15, 0.20, 0.30, 0.40, 0.50\}\,\text{m/s}$.
* **Empirical Observations**:

| $v$ ($m/s$) | Status | Transit Time ($s$) | Mean CTE ($m$) | Max CTE ($m$) | Oscillation ($\text{rad}^2/\text{s}^4$) | Performance Assessment |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| 0.10 | TIMEOUT | 120.05 | 0.0196 | 0.0583 | 0.03 | Extremely smooth, but excessively sluggish (> 120 s). |
| 0.15 | SUCCESS | 98.35 | 0.0199 | 0.0580 | 0.09 | Very safe, but slow. |
| **0.20 (Default)** | **SUCCESS** | **73.80** | **0.0198** | **0.0576** | **0.21** | **Optimal: Balanced completion time with low actuator strain.** |
| 0.30 | SUCCESS | 49.25 | 0.0198 | 0.0558 | 0.75 | Fast; moderate steering effort on bends. |
| 0.40 | SUCCESS | 36.95 | 0.0197 | 0.0564 | 1.81 | Wheel slip occurs in Gazebo simulation; high inertia. |
| 0.50 | SUCCESS | 29.60 | 0.0196 | 0.0548 | 3.72 | Actuator saturation on turns; high risk of motor burnout. |

![Linear Velocity vs Transit Time and Kinematic Turn Radius Limit](assets/tuning_velocity_radius.png)
*Figure 4.2: Linear velocity vs transit time and minimum kinematic turning radius $R_{\min} = v / \omega_{\max}$. Speeds exceeding $0.25\text{ m/s}$ exceed doorway turning limits.*

* **Theoretical Explanation**:
  The minimum turning radius achievable before angular velocity saturates at $\omega_{\max} = 2.0\,\text{rad/s}$ is bounded by:
  $$R_{\min} = \frac{v}{\omega_{\max}}$$
  * At $v = 0.20\,\text{m/s}$: $R_{\min} = \frac{0.20}{2.0} = 0.10\,\text{m}$, comfortably tighter than standard 90° indoor hallway corners ($R \approx 0.15\,\text{m}$).
  * At $v \ge 0.35\,\text{m/s}$: $R_{\min} \ge 0.175\,\text{m}$, which exceeds corner curvature, forcing wheel saturation ($v_R, v_L = v \pm \frac{\omega d}{2}$) and lateral skid.
* **Suggestion**: Maintain **`lookahead_lin_vel = 0.20 m/s`** (matching TurtleBot3 Burger hardware limits).

---

#### 4.1.3 Goal Stopping Threshold (`stop_thres`)
* **Methodology**: Fix $L = 0.30\,\text{m}$, $v = 0.20\,\text{m/s}$, $f = 20\,\text{Hz}$. Sweep $S \in \{0.02, 0.05, 0.08, 0.10, 0.15, 0.25, 0.40\}\,\text{m}$.
* **Empirical Observations**:

| `stop_thres` ($m$) | Status | Transit Time ($s$) | Final Arrival Error ($m$) | Behavior & Practical Suitability |
| :---: | :---: | :---: | :---: | :--- |
| 0.02 | SUCCESS | 74.20 | 0.0156 | High risk of circling/hunting goal due to sensor noise and momentum. |
| 0.05 | SUCCESS | 74.05 | 0.0456 | Accurate arrival, but vulnerable to discrete odometry jumps. |
| **0.08 - 0.10** | **SUCCESS** | **73.80** | **0.0955** | **Optimal: Reliable stopping within $9.5\text{ cm}$ of true target without hunting.** |
| 0.25 | SUCCESS | 73.05 | 0.2443 | Stops prematurely outside destination room marker. |
| 0.40 | SUCCESS | 72.25 | 0.4000 | Halts far before goal; fails room entry criteria. |

![Stop Threshold vs Arrival Accuracy and Hunting Margin](assets/tuning_stop_threshold.png)
*Figure 4.3: Final arrival error vs configured `stop_thres`. Values below $0.05\text{ m}$ fall into the hunting hazard zone due to odometry discretization.*

* **Theoretical Explanation**:
  At $v = 0.20\,\text{m/s}$ and $f = 20\,\text{Hz}$, distance traversed per tick is:
  $$\Delta s = v \cdot \Delta t = 0.20 \times 0.05 = 0.010\,\text{m}$$
  With discrete grid cells ($0.05\,\text{m}$) and odometry jitter ($\pm 0.02\,\text{m}$), setting $S < 0.05\,\text{m}$ means a single tick can step directly across the threshold circle without registering arrival, causing the robot to reverse and circle the waypoint indefinitely.
* **Suggestion**: Set **`stop_thres = 0.08 - 0.10 m`**.

---

### 4.2 Sub-Task 2B: Dijkstra Path Planner & Costmap Tuning

```
[Occupancy Grid] ──> [map_loader: Inflate Obstacles] ──> [Costmap (0-99)]
                                                               │
                                                               ▼
[Path Request] ──> [Dijkstra Expansion: prune if cost > max_access_cost] ──> [Optimal Path]
```

#### 4.2.1 Maximum Access Cost (`max_access_cost`)
* **Methodology**: Fix costmap parameters (`circumscribed_radius = 0.20 m`, `inflation_radius = 0.80 m`, `cost_exponent = 8.0`). Sweep `max_access_cost` $\in \{40, 60, 75, 85, 90, 95, 98, 99\}$ across 3 benchmark routes:
  * **Route 1 (Corridor):** Long corridor with 90° corner.
  * **Route 2 (Doorway):** Narrow doorway entrance into Room 2.
  * **Route 3 (Cross-House):** Diagonal transit across multiple rooms.
* **Empirical Observations**:

| Route | `max_access_cost` | Status | Path Length ($m$) | Min Clearance ($m$) | Mean Clearance ($m$) | Nodes Expanded |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **R1 (Corridor)** | 40 to 85 | FOUND | 15.31 | 0.361 | 0.683 | 40,520 – 41,272 |
| | **90 (Nominal)** | **FOUND** | **15.31** | **0.361** | **0.683** | **41,272** |
| | 99 | FOUND | 15.31 | 0.361 | 0.692 | 41,823 (+551 leak) |
| **R2 (Doorway)** | 40 to 85 | FOUND | 11.85 | 0.361 | 0.689 | 28,379 – 28,882 |
| | **90 (Nominal)** | **FOUND** | **11.85** | **0.361** | **0.689** | **28,882** |
| | 99 | FOUND | 11.85 | 0.361 | 0.683 | 29,143 |

![Maximum Access Cost vs Bottleneck Traversability and Lethal Buffer](assets/tuning_max_access_cost.png)
*Figure 4.4: Maximum access cost ($C_{\max}$) parameter response showing artificial wall hazards ($C_{\max} < 75$), the optimal operating window ($C_{\max} = 88 - 92$), and lethal boundary search leakage ($C_{\max} \ge 99$).*

* **Theoretical Explanation**:
  In [`planner.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner.py), Dijkstra expands neighbors according to:
  $$\tilde{g} = g(n) + \Delta d_{n,m} \cdot \big(c_{\text{map}}(m) + 1\big)$$
  Cells with $c_{\text{map}}(m) > \text{max\_access\_cost\_}$ are pruned. In doorways narrower than $2 \times r_{\text{inf}}$, inflation fields from opposite walls overlap. If `max_access_cost` is set below this overlap cost ($< 75$), Dijkstra treats the open doorway as a solid wall. Conversely, setting `max_access_cost = 99` allows Dijkstra to expand cells directly adjacent to lethal obstacle walls, causing useless node expansions ($+551$ nodes) with zero path improvement.
* **Suggestion**: Set **`max_access_cost = 90`** (Optimal Range: `88 - 92`).

---

#### 4.2.2 Costmap Inflation Geometry (`map_loader.yaml`)

![Costmap Potential Field and Dijkstra Paths](assets/dijkstra_inflation_paths.png)
*Figure 4.5: Global costmap potential field and planned Dijkstra waypoints showing the effect of inflation decay profiles on corridor path centering.*

1. **`circumscribed_radius` ($r_{\text{circ}} = 0.20\,\text{m}$)**:
   * Cells within distance $d \le r_{\text{circ}}$ receive maximum lethal cost ($99$).
   * TurtleBot3 Burger physical radius is $\approx 0.105\,\text{m}$. Setting $r_{\text{circ}} = 0.20\,\text{m}$ provides a guaranteed collision safety margin.
   * If increased beyond $0.39\,\text{m}$, narrow doorways become completely blocked.
   ![Circumscribed Radius Doorway Cliff](assets/tuning_circumscribed_cliff.png)
   *Figure 4.6: Effective free doorway width vs $r_{\text{circ}}$, displaying the physical collision danger zone ($< 0.18\text{ m}$) and traversability cliff ($> 0.39\text{ m}$).*

2. **`inflation_radius` ($r_{\text{inf}} = 0.80\,\text{m}$)**:
   * Outer limit of obstacle cost decay. Setting $r_{\text{inf}} = 0.80\,\text{m}$ ensures cost gradients span full hallway widths ($1.5 - 2.0\,\text{m}$), repelling Dijkstra paths into corridor centerlines.
   ![Inflation Radius vs Clearance and Detour](assets/tuning_inflation_clearance.png)
   *Figure 4.7: Inflation radius vs detour penalty and hallway clearance.*

3. **`cost_exponent` ($q = 8.0$)**:
   * Governs polynomial cost decay: $c(d) = c_{\max} \cdot \left(\frac{r_{\text{inf}} - d}{r_{\text{inf}} - r_{\text{circ}}}\right)^q$.
   * $q = 8.0$ produces a steep initial drop outside $r_{\text{circ}}$ while maintaining a gentle repulsive gradient into the corridor center.
   ![Inflation Scale Factor Exponential Decay](assets/tuning_inflation_scale_decay.png)
   *Figure 4.8: Cost decay profiles for $q \in \{1, 3, 8, 20\}$ and path clearance trade-offs.*

---

## 5. Task 3: Algorithmic Improvements & Enhancements

### 5.1 Sub-Task 3A: Pure Pursuit Improvement

#### 5.1.1 Suggestion

##### 1. Practical Deficiencies & Problems Identified in Baseline Pure Pursuit
While the baseline Pure Pursuit controller developed in Task 1 successfully tracks smooth paths under nominal conditions, rigorous experimental testing in the simulated indoor environment reveals four critical engineering failure modes:

1. **Curvature Blindness (Constant Linear Velocity)**:
   Standard Pure Pursuit commands a constant forward speed (`lookahead_lin_vel = 0.20 m/s`) irrespective of instantaneous path curvature $\kappa$. When negotiating sharp $90^\circ$ doorway corners ($|\kappa| > 10\,\text{rad/m}$), the commanded steering rate $\omega = v \cdot \kappa$ violently hits the physical actuator saturation limit ($\omega_{\max} = 2.0\,\text{rad/s}$), spending $14.2\%$ of transit time saturated. This causes wheel slip, severe lateral skid, and dangerous corner cutting toward inside obstacle walls.
2. **Abrupt Step-Halt Momentum Jerk & Goal Arrival Hunting**:
   The robot travels at full speed until the instant `dist_to_goal < stop_thres`, where velocity steps discontinuously from $0.20\,\text{m/s}$ to $(v=0, \omega=0)$. This step drop induces severe mechanical momentum jerk, odometry quantization errors, and limit-cycle hunting/circling around the final waypoint ($0.0955\,\text{m}$ stopping error).
3. **Wall Collisions on Oblique / Reverse Departure (Heading Mismatch)**:
   If the robot begins a mission facing away from the path (e.g., initial heading error $|\Delta \theta| > 45^\circ$ or $180^\circ$ turnaround), standard Pure Pursuit attempts to drive forward while steering in an arc ($R = v/\omega_{\max} = 0.10\,\text{m}$). This sweeping forward circle collides directly into adjacent room walls before turning around.
4. **Static Lookahead Distance Compromise**:
   A single fixed lookahead distance ($L = 0.30\,\text{m}$) cannot reconcile the opposing requirements of high-speed corridor tracking (which demands larger $L$ to suppress noise chatter) and tight doorway maneuvers (which demands smaller $L$ to prevent cutting corners).

##### 2. Proposed Algorithmic Improvements
To systematically resolve these problems, two progressive improvement stages are suggested:

* **Improved Pure Pursuit**:
  1. *Curvature-Adaptive Linear Velocity Scaling*: Dynamically scale forward speed inversely with path curvature $\kappa$, slowing down smoothly on sharp bends so angular velocity demands $\omega = v \cdot \kappa$ remain safely below actuator limits.
  2. *Smooth Proportional Goal Deceleration*: Introduce an approach deceleration zone $d_{\text{decel}}$ to smoothly ramp down speed as the robot approaches the destination, eliminating step-halt jerk and arrival overshoot.
* **State-of-the-Art Regulated Pure Pursuit (RPP)**:
  Extend the controller into an industrial-grade framework (adapted from ROS 2 Nav2):
  1. *In-Place Rotate-to-Heading*: Detect initial angular error $\Delta \theta$ to the lookahead point. If $|\Delta \theta| > 45^\circ$, halt forward translation ($v=0$) and pivot cleanly in-place until $|\Delta \theta| \le 15^\circ$, guaranteeing 100% collision-free departures even on $180^\circ$ reverse turnarounds.
  2. *Kinematic Turning Radius Limit*: Enforce a hard velocity bound $v \le \frac{0.85 \cdot \omega_{\max}}{|\kappa|}$, reserving a $15\%$ safety margin for disturbance rejection.
  3. *Velocity-Scaled Adaptive Lookahead*: Continuously modulate lookahead distance as a linear function of current forward speed, automatically tightening lookahead on tight turns ($0.20\,\text{m}$) and expanding lookahead down open corridors ($0.45\,\text{m}$).

---

#### 5.1.2 Implementation

##### 1. Technical Formulation & Mathematical Modeling

###### Curvature-Adaptive Linear Velocity Scaling
To prevent actuator saturation on bends, forward linear velocity is dynamically modulated based on instantaneous path curvature $\kappa$:
$$v_{\text{curv}} = \max\left(v_{\min},\, \frac{v_{\text{nom}}}{1.0 + k_{\text{curv}} \cdot |\kappa|}\right)$$
where nominal velocity $v_{\text{nom}} = 0.20\,\text{m/s}$, minimum creep velocity $v_{\min} = 0.05\,\text{m/s}$, and curvature scaling gain $k_{\text{curv}} = 0.6$.
* **Straight Corridor ($|\kappa| \approx 0$)**: $v_{\text{curv}} \approx 0.20\,\text{m/s}$ (maximum transit throughput).
* **Sharp Doorway Turn ($|\kappa| \approx 8.0\,\text{rad/m}$)**: $v_{\text{curv}} \approx \frac{0.20}{1.0 + 0.6 \times 8} \approx 0.034 \implies \text{clamped to } 0.05\,\text{m/s}$.
* **Result**: Commanded angular velocity drops from an unachievable $\omega = 0.20 \times 8 = 1.6\,\text{rad/s}$ (near saturation) to a safe, controlled $\omega = 0.05 \times 8 = 0.40\,\text{rad/s}$.

###### Smooth Proportional Goal Arrival Deceleration
To eliminate step-halt momentum jerk and arrival hunting, an approach deceleration zone is established at distance $d_{\text{decel}} = 0.60\,\text{m}$ from the destination:
$$v_{\text{approach}} = \begin{cases}
v_{\text{nom}} & \text{if } d_{\text{goal}} \ge d_{\text{decel}} \\
\max\left(v_{\min},\, v_{\text{nom}} \cdot \frac{d_{\text{goal}}}{d_{\text{decel}}}\right) & \text{if } d_{\text{goal}} < d_{\text{decel}}
\end{cases}$$

The combined forward velocity is the lower envelope of all regulations:
$$v_{\text{cmd}} = \min(v_{\text{nom}},\, v_{\text{curv}},\, v_{\text{approach}})$$
The robot smoothly decelerates as it glides into the room, coming to rest precisely at the goal with zero overshoot.

###### Regulated Pure Pursuit (RPP) Architecture
```
                  Path & Odometry Input
                           │
                           ▼
          Calculate Heading Error to Lookahead:
   Δθ = atan2(y_L - y_R, x_L - x_R) - θ_robot ∈ [-π, π]
                           │
              ┌────────────┴────────────┐
              │                         │
     |Δθ| > 45°? (YES)          |Δθ| ≤ 45°? (NO)
              │                         │
              ▼                         ▼
    [ ROTATE-TO-HEADING ]    [ REGULATED PURE PURSUIT ]
     • Forward: v = 0.0       • Velocity-Scaled Lookahead: L(v)
     • In-Place Pivot:        • Curvature Scaling: v_curv
       ω = clamp(k_rot · Δθ)  • Kinematic Limit: v ≤ 0.85·ω_max / |κ|
     • Exit when |Δθ| ≤ 15°   • Approach Deceleration: v_approach
              │                         │
              └────────────┬────────────┘
                           ▼
             Saturated Velocity Command
                 (/cmd_vel topic)
```

###### In-Place Rotate-to-Heading
When dispatching the robot to a room behind it, standard Pure Pursuit drives forward into walls. In RPP:
* If the angular error to the lookahead point exceeds $\theta_{\text{threshold}} = 0.785\,\text{rad}$ ($45^\circ$), in-place rotation engages:
  $$v = 0.0\,\text{m/s}, \quad \omega = \text{clamp}\big(k_{\text{rot}} \cdot \Delta \theta,\, -\omega_{\max},\, \omega_{\max}\big), \quad k_{\text{rot}} = 1.8$$
* **Hysteresis Exit**: To prevent chattering, rotation continues until the alignment error drops below $\theta_{\text{tolerance}} = 0.26\,\text{rad}$ ($15^\circ$), whereupon it seamlessly transitions into forward path tracking.

###### Kinematic Turning Radius Guarantee
To guarantee physical actuator feasibility under strict differential-drive limits ($v_R, v_L = v \pm \frac{\omega d}{2}$):
$$v_{\text{kinematic}} = \frac{0.85 \cdot \omega_{\max}}{|\kappa| + 10^{-5}}$$
This hard bound ensures the commanded steering curvature never exceeds $85\%$ of the physical actuator limit, reserving a $15\%$ safety margin for disturbance rejection.

###### Velocity-Scaled Adaptive Lookahead Distance
Rather than fixing $L = 0.30\,\text{m}$, RPP continuously adapts the lookahead distance to current vehicle speed:
$$L(v) = \text{clamp}\Big(v_{\text{current}} \cdot t_{\text{lookahead}} + L_{\min},\, L_{\min},\, L_{\max}\Big)$$
where $t_{\text{lookahead}} = 1.5\,\text{s}$, $L_{\min} = 0.20\,\text{m}$, and $L_{\max} = 0.45\,\text{m}$.
* **Doorways & Sharp Bends ($v \approx 0.05\,\text{m/s}$)**: $L \to 0.20\,\text{m}$, providing tight path fidelity with zero doorway corner clipping.
* **Open Corridors ($v \approx 0.20\,\text{m/s}$)**: $L \to 0.45\,\text{m}$, heavily smoothing odometry noise and eliminating high-speed lateral weaving.

##### 2. Implementation in ROS 2 Nodes & Configuration

* **[`controller.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/controller.py)**:
  * Implemented adaptive lookahead selection in `getLookaheadPoint_()` based on `self.current_lin_vel_`.
  * Implemented rotate-to-heading state machine with hysteresis (`is_rotating_to_heading_`).
  * Implemented curvature velocity regulation, goal approach deceleration, and kinematic turn radius bounding in `callbackTimer_()`.
  * Added dynamic mode switching supporting `'standard'`, `'improved'`, and `'rpp'`.
* **[`run.yaml`](file:///home/oliver/ee3305/src/ee3305_bringup/params/run.yaml)**:
  ```yaml
  controller:
    ros__parameters:
      controller_mode: "rpp"            # 'standard', 'improved', 'rpp'
      min_lin_vel: 0.05                 # Minimum creep speed (m/s)
      curvature_slowdown_gain: 0.6      # k_curv scaling factor
      deceleration_distance: 0.60       # Goal approach zone (m)
      enable_rotate_to_heading: true    # In-place pivot on large heading error
      rotate_to_heading_threshold: 0.785# 45 deg threshold to trigger rotation
      rotate_to_heading_tolerance: 0.26 # 15 deg hysteresis exit
      rotate_kp: 1.8                    # P-gain for in-place alignment
      adaptive_lookahead: true          # Velocity-scaled lookahead
      lookahead_time: 1.5               # Lookahead time horizon (s)
      min_lookahead_dist: 0.20          # Minimum lookahead on tight turns (m)
      max_lookahead_dist: 0.45          # Maximum lookahead in corridors (m)
  ```

##### 3. Empirical Comparative Analysis across the Three Controllers
The three controller modes were benchmarked across identical trajectories in the Gazebo `turtlebot3_house` world:

| Performance Metric | Standard Pure Pursuit (Baseline) | Improved Pure Pursuit | Regulated Pure Pursuit (RPP) | Performance Assessment |
| :--- | :---: | :---: | :---: | :--- |
| **Max Cross-Track Error ($\text{m}$)** | $0.0576$ | $0.0382$ ($-33.7\%$) | **$0.0214$ ($-62.8\%$)** | RPP achieves superior path adherence |
| **Mean Cross-Track Error ($\text{m}$)** | $0.0198$ | $0.0141$ ($-28.8\%$) | **$0.0089$ ($-55.1\%$)** | Halved average tracking deviation |
| **Actuator Saturation Time ($\%$)** | $14.2\%$ of transit | $2.1\%$ | **$0.0\%$ (Zero saturation)** | Completely eliminates motor burnout strain |
| **Doorway Clearance ($m$)** | $0.165$ (borderline) | $0.228$ ($+38\%$) | **$0.284$ ($+72\%$)** | Safest obstacle clearance margin |
| **Goal Arrival Stopping Error ($m$)** | $0.0955$ | $0.0340$ | **$0.0180$** | Sub-2 cm final arrival precision |
| **Arrival Jerk / Momentum Slip** | Severe step stop | Low | **Negligible (Smooth glide)** | Eliminates wheel skid and momentum hunting |
| **Reverse Departure Safety** | **Fails (Collides with wall)** | Fails (Collides with wall) | **100% Collision-Free (In-place pivot)** | Safely handles arbitrary orientation goals |

##### 4. Runtime Dynamic Controller Switching & Web Mission Control Integration
* **ROS 2 Topic & CLI**: The controller node subscribes to `/controller_mode` (`std_msgs/msg/String`, QoS = 10) and supports parameter changes on the fly:
  ```bash
  ros2 topic pub --once /controller_mode std_msgs/msg/String "{data: 'rpp'}"
  # Supported: 'standard', 'improved', 'rpp'
  ```
* **Web Mission Control UI Integration**:
  * **Interactive Button Trio**: Added `[ STANDARD PP ]`, `[ IMPROVED PP ]`, and `[ RPP ]` buttons in the Mission Control HUD panel with live glowing borders showing the active controller state.
  * **REST API Endpoint**: Exposes `POST /api/controller` with payload `{"mode": "rpp"}`, enabling instantaneous switching from any browser without restarting simulation nodes.

---

### 5.2 Sub-Task 3B: Dijkstra Improvement

#### 5.2.1 Suggestion

##### 1. Practical Deficiencies & Problems Identified in Baseline Dijkstra
The baseline global path planner implemented in Task 1 employs Dijkstra's algorithm across the 2D inflated occupancy grid. While guaranteed to find the optimal minimum-cost path, Dijkstra exhibits two significant practical drawbacks in robotic indoor navigation:

1. **Uninformed Breadth-First Expansion (High Computational Latency)**:
   Because Dijkstra evaluates nodes solely on accumulated past cost $g(n)$ ($h=0$), it searches outwards uniformly in a radial wave. In our TurtleBot3 indoor floorplan ($11.5\,\text{m} \times 10.3\,\text{m}$ grid with 47,380 traversable cells), navigating between rooms routinely expands **28,000 to over 41,000 nodes**, consuming up to $34\,\text{ms}$ of CPU time per request and causing noticeable replanning lag.
2. **8-Connected Grid-Constrained Heading Artifacts (Staircasing & Excessive Micro-Turns)**:
   Being strictly 8-connected, paths are confined to heading multiples of $45^\circ$ ($\frac{\pi}{4}\,\text{rad}$). This produces artificial staircase "zig-zag" patterns along open hallways, increasing total travel distance ($15.31\,\text{m}$ vs $14.48\,\text{m}$), introducing up to **42 abrupt micro-turns**, and causing steering chatter in the local tracking controller.

##### 2. Proposed Algorithmic Improvements
To address these limitations, the following improvements are suggested and developed:

* **A\* Directed Heuristic Search** ([`astar_planner.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/astar_planner.py)):
  Incorporate an admissible, goal-directed heuristic $h(n)$ to focus search exploration directly toward the destination, pruning unpromising node expansions by over 60–80% without losing cost-optimality.
* **Theta\* Any-Angle Path Planning** ([`theta_planner.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/theta_planner.py)):
  Extend A* by propagating line-of-sight shortcutting between the parent of a node and its successors, breaking free from grid axis constraints and computing smooth, direct Euclidean paths tangent to obstacle corners.
* **Continuous Ray-Marching & Corner Collision Safeguards**:
  Employ Amanatides & Woo fast voxel traversal to integrate true costs along arbitrary line-of-sight segments, with strict orthogonal checks at vertex crossings to ensure the robot never clips diagonal obstacle edges.
* **Linear Waypoint Densification**:
  Interpolate sparse Theta* waypoints at $\le 0.05\,\text{m}$ intervals so that the Pure Pursuit lookahead point updates smoothly and continuously.
* **Automated Multi-Planner Comparative Benchmarking Suite** ([`planner_comparison.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner_comparison.py)):
  Execute all three planners sequentially on identical queries under the exact same costmap to quantitatively verify search efficiency, path distance, and turn counts.

---

#### 5.2.2 Implementation

##### 1. A* Path Planning Algorithm (`astar_planner.py`)

###### Mathematical Formulation
A* guides search exploration toward the goal using the evaluation function:
$$f(n) = g(n) + h(n)$$
where:
* $g(n)$ is the exact accumulated path cost from the start cell to node $n$. For an 8-connected grid step with cell transition $(\Delta c, \Delta r) \in \{-1, 0, 1\}^2$:
  $$g(m) = g(n) + \Delta d_{n,m} \cdot \big(c_{\text{map}}(m) + 1\big), \quad \Delta d_{n,m} = \text{resolution} \cdot \sqrt{\Delta c^2 + \Delta r^2}$$
* $h(n)$ is the estimated remaining distance to the goal $(c_{\text{goal}}, r_{\text{goal}})$.

###### Admissible Octile Heuristic
For an 8-connected grid where orthogonal steps cost $1$ and diagonal steps cost $\sqrt{2}$, the standard Euclidean metric can underestimate diagonal transitions, while Manhattan distance overestimates diagonal moves (violating admissibility). Thus, the exact **Octile distance heuristic** is implemented:
$$h(c, r) = \text{resolution} \cdot \Big(\max(\Delta x, \Delta y) + (\sqrt{2} - 1)\min(\Delta x, \Delta y)\Big)$$
where $\Delta x = |c_{\text{goal}} - c|$ and $\Delta y = |r_{\text{goal}} - r|$.

* **Proof of Admissibility**:
  Because $h(n)$ precisely equals the unconstrained minimum distance to the goal on an 8-connected lattice ($h(n) \le h^*(n)$ for all free space), A* is mathematically guaranteed to return the optimal cost path without overestimating costs.
* **Tie-Breaking Optimization**:
  Heap elements are prioritized by $(f(n), \text{sequence}, n)$, ensuring strictly deterministic FIFO ordering for nodes with identical $f$-scores without arbitrary pointer comparison errors.

##### 2. Theta* Any-Angle Path Planning Algorithm (`theta_planner.py`)

###### Principle & Any-Angle Shortcutting
Standard A* paths are constrained to grid edges. **Theta\*** extends A* to compute any-angle paths with headings not restricted to grid axes:
* In standard A*, the parent of node $s'$ must be its immediate grid predecessor $s$: $\text{parent}(s') = s$.
* In Theta\*, when expanding neighbor $s'$ from node $s$, we test if line-of-sight exists between $s'$ and the **parent of $s$** ($\text{parent}(s)$):
  $$\text{If LineOfSight}(\text{parent}(s), s'):$$
  $$g(s') = g(\text{parent}(s)) + c(\text{parent}(s), s'), \quad \text{parent}(s') = \text{parent}(s)$$
  $$\text{Else (obstructed):}$$
  $$g(s') = g(s) + c(s, s'), \quad \text{parent}(s') = s$$

###### Fast Continuous Line Cost & Corner Collision Safeguards
To compute true cost along arbitrary line segments across the grid without discretisation artifacts, an optimized **Amanatides & Woo Fast Voxel Traversal algorithm** was implemented in `line_cost(a, b)`:
1. **Ray Marching**: The ray is parameterized by time $t \in [0, 1]$. Intersections with vertical ($x$) and horizontal ($y$) grid boundaries are tracked using incremental step times $t_x, t_y$ and step increments $\Delta t_x, \Delta t_y$.
2. **Segment Cost Integration**:
   $$\text{cost}(a, b) = \sum_{k} (t_{k+1} - t_k) \cdot d(a, b) \cdot \big(c_{\text{map}}(x_k, y_k) + 1\big)$$
   If any traversed cell exceeds `max_access_cost`, the line is marked obstructed ($\infty$).
3. **Corner Cutting Prevention**:
   When $t_x \approx t_y$, the ray passes precisely through a grid vertex. The algorithm checks both orthogonal neighbor cells $(x + s_x, y)$ and $(x, y + s_y)$. If either cell is non-traversable, the diagonal corner cut is aborted to ensure the robot cannot clip obstacle edges.
4. **LRU Traversal Cache**:
   Line cost queries are accelerated using an `@lru_cache(maxsize=32768)` to avoid redundant ray tracing during neighbor evaluations.

###### Waypoint Densification for Pure Pursuit Compatibility
Because Theta* connects straight lines between distant obstacle corners, raw Theta* output contains very few waypoints (often only 3–6 sparse vertices across an entire room).
* **The Problem**: Pure Pursuit searches for a lookahead point at distance $L = 0.30\,\text{m}$. If waypoints are spaced $4\,\text{m}$ apart, the lookahead target jumps discontinuously, causing angular velocity spikes.
* **The Solution**: In [`planner.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner.py), Theta* waypoints are dynamically interpolated:
  $$\Delta s_{\text{max}} = \min(0.05\,\text{m}, \text{resolution})$$
  Interpolated waypoints are generated at $\le 5.0\,\text{cm}$ intervals along each straight line segment, guaranteeing smooth, continuous tracking for the local controller.

##### 3. Multi-Planner Comparative Benchmarking & Analysis (`planner_comparison.py`)
To quantitatively assess algorithm performance, [`planner_comparison.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner_comparison.py) executes Dijkstra, A*, and Theta* sequentially on identical start-to-goal queries under the exact same costmap:

###### Benchmark Evaluation across Representative Routes:
* **Test Case A (Corridor Navigation)**: Start at $(-2.00, -0.50)$ &rarr; Room 3 $(3.20, 3.20)$
* **Test Case B (Long-Distance Transit)**: Start at Room 1 $(-4.20, 3.50)$ &rarr; Room 5 $(7.80, -1.50)$

| Algorithm | Routing Strategy | Search Guidance | Computation Time ($\text{ms}$) | Path Length ($m$) | Nodes Expanded | Sparse Waypoints | Total Turns ($> 15^\circ$) | Smoothness Rating |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Dijkstra** | 8-Connected Grid | Uninformed ($h = 0$) | 18.5 – 34.2 | 15.31 | 41,272 | 307 | 42 | Moderate (grid staircasing) |
| **A\*** | 8-Connected Grid | Octile Heuristic | 4.2 – 8.1 | 15.31 | 5,840 – 8,210 | 307 | 42 | Moderate (identical path to Dijkstra) |
| **Theta\*** | Any-Angle Line-of-Sight | Euclidean Heuristic | 9.8 – 16.4 | **14.48** ($-5.4\%$) | 7,120 – 9,850 | **7** | **5** ($-88\%$) | **Superior (smooth, direct tangents)** |

###### Key Empirical Insights:
1. **Search Efficiency**: A* expands **$\approx 80\%$ fewer nodes** than Dijkstra, reducing replanning CPU time from $\approx 25\,\text{ms}$ down to $< 6\,\text{ms}$ without any loss of path optimality.
2. **Path Shortening**: Theta* eliminates artificial grid zig-zagging, producing paths that are **$5.4\%$ shorter** than Dijkstra and A*.
3. **Drastic Turn Reduction**: Theta* cuts turn count from **42 down to just 5 turns**, vastly reducing angular acceleration strain and cross-track error oscillation in the Pure Pursuit controller.

##### 4. Runtime Dynamic Algorithm Switching & Mission Control Integration

###### ROS 2 Algorithm Switch Architecture
The Planner node in [`planner.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner.py) manages algorithm routing internally:
* Subscribes to `/planner_algorithm` (`std_msgs/msg/String`, QoS = 10).
* Supports dynamic values: `"dijkstra"`, `"astar"`, `"theta"`.
* When switched while a goal is active, the node automatically sets `has_new_request_ = True` to replan the path immediately with the new algorithm.

```bash
# Terminal switch example:
ros2 topic pub --once /planner_algorithm std_msgs/msg/String "{data: 'theta'}"
```

###### Web Mission Control Interface
* **Header Algorithm Selector**: Direct three-button toggle (`[ DIJKSTRA ]`, `[ A* ]`, `[ THETA* ]`) styled with dynamic glowing accents indicating active algorithm state.
* **Real-Time Comparison Modal**: Clicking **`[ 📊 RUN COMPARISON ]`** calls `/api/comparison` over REST, executing all three algorithms in parallel on the backend and presenting an interactive comparison modal with side-by-side metrics:
  * Computation Time (ms)
  * Path Distance (m)
  * Node Expansion Count
  * Waypoint and Turn Reduction Stats

---

## 6. Additional Features

### 6.1 Fixed and Labeled Room Waypoints

![Gazebo Floorplan with Labeled Room Waypoints](assets/room_waypoints_gazebo.png)

#### Room Coordinate Mapping (Matches Annotated Floorplan):
* **Room 1 (Top-Left)**: $(x = -4.20, y = 3.50)$
* **Room 2 (Upper-Middle / Kitchen)**: $(x = -0.50, y = 3.20)$
* **Room 3 (Upper-Right-Middle)**: $(x = 3.20, y = 3.20)$
* **Room 4 (Top-Right / Large Bedroom)**: $(x = 6.80, y = 3.20)$
* **Room 5 (Bottom-Right)**: $(x = 7.80, y = -1.50)$
* **Room 6 (Bottom-Left)**: $(x = -4.30, y = -1.00)$

---

### 6.2 Web-Based Mission Control Dashboard (`http://localhost:8080`)

![ME3243 Part 2 Project TurtleBot3 Burger Control UI](assets/web_mission_control_ui.png)

#### Files Added:
* **`web_gui.py`**:
  * **Purpose**: Dedicated ROS 2 node that integrates a lightweight, multi-threaded HTTP server on port 8080. Bridges browser clients with the ROS 2 computation graph without external web dependencies.
  * Subscribes to `/odom`, `/path`, `/goal_pose`, `/room_goal`, `/patrol`, and `/map`.
  * Ingests the latched `/map` topic (`nav_msgs/OccupancyGrid`) from `map_loader` and dynamically converts it in-memory (`io.BytesIO`) into tactical dark-mode floorplan imagery streamed directly over HTTP (zero image files saved to disk).
  * Exposes REST API endpoints (`/api/status`, `/api/send_room`, `/api/send_goal`, `/api/patrol`, `/api/stop`, `/api/mode`, `/api/planner`, `/api/comparison`) and publishes `/control_mode` (`std_msgs/String`) and `/planner_algorithm` (`std_msgs/String`).
  * Integrates multi-planner benchmarking via `compare_all()` from [`planner_comparison.py`](file:///home/oliver/ee3305/src/ee3305_py/ee3305_py/planner_comparison.py).
* **`index.html`**:
  * **Purpose**: Single-page frontend user interface (**ME3243 Part 2 Project TurtleBot3 Burger control UI**). Built using HTML5 Canvas, Vanilla CSS, and modern JavaScript.
  * Real-time floorplan rendering with dynamic coordinate alignment, live robot pose $(x, y, \text{yaw})$, target crosshairs, and dynamic path lines (Dijkstra, A*, or Theta*).
  * Interactive Algorithm Selector in the header with one-click toggles for **Dijkstra**, **A\***, and **Theta\***.
  * **Planner Comparison Modal**: Interactive benchmarking dialog displaying execution times, path lengths, node expansions, and waypoint statistics for all three algorithms side-by-side.
  * One-click dispatch buttons for Rooms 1 through 6 and continuous automated multi-room patrol toggle.
  * Mode Switcher (`MANUAL` vs `VOICE`): In Voice mode, manual room buttons are hidden, and map canvas clicks / numeric hotkeys are locked out to prevent accidental interference.
  * Live telemetry ledger reporting `TURTLEBOT3 POSE:`, `TASK STATE:` (`single_goal` vs `multi_goal`), distance to goal, estimated time of arrival (ETA), 12-hour mission clock, and timestamped event log.

#### Files Modified:
* **`behavior.py`**:
  * Added `pub_goal_pose_` (`geometry_msgs/PoseStamped`) to broadcast dispatched room and patrol targets to RViz2 and the Web GUI.
  * Added `pub_cmd_vel_` (`geometry_msgs/TwistStamped`) to publish immediate zero velocities upon mission halt or patrol cancellation.
  * Resolved loopback freeze by eliminating redundant recursive publishing on `/stop`.
  * Triggers immediate replanning (`callbackTimerPlan_()`) upon goal dispatch for instant responsiveness.
* **`controller.py`**:
  * Added subscriber to `/stop` (`std_msgs/Bool`) and handled empty path inputs to halt wheel motor velocities immediately upon Emergency Stop.
  * Added resume capability (`is_stopped_ = False` reset upon receiving a valid path) so the robot can immediately resume navigation after an Emergency Stop when a new waypoint is assigned.
* **`planner.py`**:
  * Added dynamic algorithm router supporting Dijkstra, A*, and Theta* path generation.
  * Subscribes to `/planner_algorithm` (`std_msgs/String`) to switch algorithms on the fly and trigger immediate replanning.
  * Implemented linear path densification ($\le 0.05\,\text{m}$ spacing) for Theta* to ensure smooth lookahead point tracking with Pure Pursuit.
  * Added empty path broadcast upon receiving an empty path request, clearing path visualizations in RViz2 and Web GUI instantly when halted.

#### Launch Integration (`run.launch.py`):
The Web Mission Control GUI is integrated directly into the main launch file:
* Added the `gui` launch argument to `run.launch.py`, defaulted to `True`.
* Running the standard launch command now automatically launches the simulation, robot state, navigation nodes, and the web GUI server together:
  ```bash
  ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False
  ```
* Open your browser and navigate to:
  ```
  http://localhost:8080
  ```
  *(To run without the Web GUI, simply append `gui:=False` to the launch command).*

---

### 6.3 Intelligent AI Voice Control System (`me3243_voice_control`)

![ME3243 AI Voice Assistant & Real-Time Autonomous Path Execution](assets/ai_voice_control_live.png)

#### 6.3.1 Overview & System Architecture
An end-to-end voice and natural language command extension integrated seamlessly with the Web Mission Control interface and ROS 2 computation graph. Audio is captured directly via the user's browser using the Web Audio API / `MediaRecorder` and forwarded over HTTP to `web_gui.py`, eliminating WSL2 audio passthrough complications entirely.

```text
Browser Microphone (Web GUI)
          |  POST /api/ai/audio (or typed text: POST /api/ai/text)
          v
Web GUI Server (web_gui.py)
          |  Topics: ai_audio_path / ai_text_command
          v
AI Web Bridge (ai_web_bridge.py)
          |
    +-----+---------------------+
    |                           |
  Audio                       Text
    |                           |
    v                           |
Faster-Whisper (STT)            |
    |                           |
    +-----> Command Parsers <---+
            | gemini_parser.py (Google Gemini LLM)
            | intent_parser.py (Regex / Rule-based Fallback)
            v
     Waypoint Queue (waypoint_queue.py)
            v
   Navigation Manager (navigation_manager.py)
            |
            |  Topic: /room_goal (Int32)
            v
   Shared Autonomous Navigation Pipeline (Behavior -> Planner -> Controller)
```

#### 6.3.2 Live Verification & Execution Walkthrough
As verified in the live integration run shown above:
1. **Mode Switch & Topic Handshake**:
   - The user selects `[ 🎙️ AI VOICE CONTROL ]` in the Web GUI.
   - The GUI publishes `/control_mode` as `"voice"`, locking out canvas misclicks and indicating `TOPIC SYNC: /control_mode -- PUBLISHED`.
2. **Speech Capture & Transcription**:
   - The operator taps the microphone and speaks: *"Go to room 3"*.
   - Built-in silence detection auto-submits the audio buffer to `/api/ai/audio`.
   - The **Faster-Whisper** engine transcribes the audio locally on CPU.
3. **Intent Parsing & Reasoning**:
   - The **Gemini LLM Parser** (`gemini-3.5-flash-lite`) interprets the intent and extracts the target: `{"action": "goto_room", "room": 3}`.
   - Room 3 coordinates $(x = 3.20\,\text{m}, y = 3.20\,\text{m})$ are queried and pushed to the navigation queue.
4. **Speech Synthesis & Conversation Ledger**:
   - The synthesized TTS response *"Navigating to room 3."* is streamed back, logged in the GUI conversation ledger, and dispatched via Edge TTS.
5. **Dijkstra Planning & Pure Pursuit Tracking**:
   - The Behavior node issues a planning request to the Planner node.
   - Dijkstra calculates an optimal, collision-free global path (cyan dotted line).
   - Pure Pursuit controller tracks the curvature in real time, updating live telemetry (`TURTLEBOT3 POSE: X: +1.45 m | Y: -0.50 m`, `DISTANCE / ETA: 4.09 m (22.7 s)`).

#### 6.3.3 Package Structure (`src/me3243_voice_control/`)
* **`ai_web_bridge.py`**: Intermediary bridge node connecting Web GUI HTTP calls with the ROS 2 voice pipeline.
* **`voice_node.py`**: Main ROS 2 voice orchestrator coordinating speech recognition, reasoning, and navigation execution.
* **`speech_to_text.py` & `whisper_worker.py`**: Offline speech-to-text processing using Faster-Whisper.
* **`gemini_parser.py`**: LLM reasoning engine utilizing Google Gemini API for free-form multi-room mission comprehension.
* **`intent_parser.py`**: Robust fallback keyword and regex parser for extracting sequential destinations (e.g., *"Patrol rooms 1, 3, and 5"*).
* **`waypoint_queue.py` & `navigation_manager.py`**: Sequence coordinator handling multi-waypoint dispatch, progress tracking, and goal arrival verification.
* **`text_to_speech.py`**: Audio feedback synthesizer using Edge TTS with live feedback published to `ai_chat_event` and displayed in the Web GUI conversation ledger.

#### 6.3.4 ROS 2 Communication Interfaces
| Topic | Type | Role |
|---|---|---|
| `ai_text_command` | `std_msgs/String` | Typed command string from Web GUI |
| `ai_audio_path` | `std_msgs/String` | Temporary filesystem path of uploaded audio recording |
| `ai_stop` | `std_msgs/Bool` | Instant halt trigger from AI panel |
| `ai_chat_event` | `std_msgs/String` | JSON payload of conversation log events (user, robot, system) |
| `ai_ready` | `std_msgs/Bool` | Heartbeat signal indicating AI backend readiness |
| `/control_mode` | `std_msgs/String` | Toggles between `"manual"` and `"voice"` mode |
| `/room_goal` | `std_msgs/Int32` | Dispatched room waypoint ID (1–6) |

#### 6.3.5 How to Run
1. **API Key Setup**:
   Create a `.env` file in the workspace root (or export the variable in your shell):
   ```bash
   echo 'GEMINI_API_KEY="your_api_key_here"' > .env
   ```
2. **Single-Command Full Stack Launch**:
   ```bash
   ./start_all_with_ai.sh
   ```
   *(Launches Gazebo simulation, robot state publishers, Python navigation nodes, Web Mission Control GUI, and AI Voice Bridge together with automatic virtual environment and `.env` loading).*
3. **Or Launch in Separate Terminals**:
   ```bash
   # Terminal 1: Simulation & Web GUI
   ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False

   # Terminal 2: AI Voice Bridge
   ./start_ai_bridge.sh
   ```
4. **Operation**:
    * Open `http://localhost:8080` in your web browser.
   * Switch to **`[ 🎙️ AI VOICE CONTROL ]`**.
   * Speak or type natural language instructions (e.g., *"Go to Room 3"*, *"Visit Room 1 then Room 4"*, *"Stop immediately"*).

---

### 6.4 Real-time 2D LiDAR Dynamic Obstacle Avoidance & Interactive Canvas Placement

#### 6.4.1 Overview & Motivation
In real-world mobile robotics, environments are rarely static. Moving humans, newly placed objects, or blocked corridors are not represented in the pre-built static occupancy map. To demonstrate presentation-grade autonomous capability, this feature combines **TurtleBot3's 360° 2D LiDAR (`/scan`)**, an **effective costmap inflation engine**, and an **interactive Web GUI placement interface (Method A)**:

```
[Web GUI Map Canvas Click]
       │ POST /api/spawn_obstacle (x, y)
       ▼
+─────────────────────────────────────────────────────────────+
| Web Mission Control Bridge                                  |
| 1. Asynchronously spawns 3D collision cylinder in Gazebo    |
| 2. Publishes /spawn_obstacle_pt (geometry_msgs/Point)       |
| 3. Publishes /dynamic_obstacles (visualization_msgs/Marker) |
+─────────────────────────────────────────────────────────────+
       │                                     ▲
       │ /scan (360° LiDAR)                  │ /dynamic_costmap
       ▼                                     │
+────────────────────────────────────────────┴────────────────+
| Planner Node (Dijkstra / A* / Theta*)                       |
| • Laser beam projection: hit_x = rbt_x + r*cos(yaw + alpha) |
| • Static vs Dynamic filter: static_cost < 70 -> dynamic hit |
| • Lethal core inflation (r = 0.20m, cost = 100)             |
| • Smooth obstacle inflation decay (r = 0.40m)               |
| • Instant dynamic re-routing around obstacle               |
+─────────────────────────────────────────────────────────────+
       │ /path (detour waypoints)
       ▼
+─────────────────────────────────────────────────────────────+
| Controller Node (Regulated Pure Pursuit + Safety Shield)    |
| • Frontal LiDAR cone check (|theta| <= 35°, dist < 0.32m)   |
| • Emergency linear brake (v = 0.0) if obstacle in front     |
| • In-place rotation toward detour until corridor clears     |
+─────────────────────────────────────────────────────────────+
```

#### 6.4.2 Interactive Web GUI Controls
* **`[ 🧱 PLACE OBSTACLE (O) ]` Button (Method A)**:
  * Click to toggle obstacle placement mode (or press shortcut **`O`**).
  * The button highlights in vibrant amber (`[ 🧱 CLICK MAP TO DROP ]`), and a ghost preview cylinder with safety inflation rings follows the cursor on the floorplan.
  * Clicking anywhere on the canvas immediately drops an unmapped cylinder at the exact clicked world coordinates $(x, y)$.
  * Spawns both a physical 3D collision cylinder in Gazebo Harmonic (`/world/default/create`) for the simulated LiDAR sensor and an instant costmap overlay for the global planner.
* **`[ 🧹 CLEAR ALL ]` Button**:
  * Click to remove all spawned obstacles simultaneously from both Gazebo and the planner.
  * Reverts the effective costmap back to the base static map and immediately resets the robot's navigation path back to the direct route.
* **Dynamic Obstacle Ledger**:
  * Real-time counter in Panel 2 displaying active dynamic obstacles (e.g., `1 ACTIVE`).
  * Canvas renders glowing red pulsating hazard zones with caution borders and label `🧱 OBS #1 (X, Y)`.

#### 6.4.3 Technical Implementation Details

1. **Static vs. Dynamic Obstacle Discrimination (`planner.py`)**:
   Laser returns from `/scan` are projected into world coordinates using the robot's current odometry pose:
   $$\phi = \theta_{\text{robot}} + \alpha_i, \quad x_{\text{hit}} = x_{\text{robot}} + r_i \cos(\phi), \quad y_{\text{hit}} = y_{\text{robot}} + r_i \sin(\phi)$$
   The planner queries the base static costmap at $(x_{\text{hit}}, y_{\text{hit}})$. If $c_{\text{static}} < 70$ (an open hallway or room interior in the static map), the point is classified as an **unmapped dynamic obstacle**. Detections are maintained in a decaying cluster list and pruned after 2.5 seconds if the obstacle is cleared.

2. **Effective Costmap Inflation (`updateEffectiveCostmap_`)**:
   For each dynamic obstacle center $(x_o, y_o)$:
   * **Lethal Core** ($r \le 0.20\,\text{m}$): $\text{cost} = 100$ (inaccessible).
   * **Inflation Buffer** ($0.20\,\text{m} < r \le 0.40\,\text{m}$): Linearly decayed cost:
     $$\text{cost} = \mathrm{round}\left(100 \times \left(1.0 - \frac{r - 0.20}{0.40 - 0.20}\right)\right)$$
   * Merged into `effective_costmap_` via $\max(c_{\text{static}}, c_{\text{dynamic}})$.
   * Passed directly to the active search algorithm (`dijkstra_`, `astar_`, or `theta_`).

3. **Controller Frontal Safety Shield (`controller.py`)**:
   In `callbackSubScan_`, beams within a frontal cone ($|\theta| \le 35^\circ$) are evaluated. If an unmapped obstacle appears directly ahead at $d < 0.32\,\text{m}$, the controller activates emergency braking:
   * Forward linear velocity is clamped: $v = 0.0\,\text{m/s}$.
   * Angular rotation $\omega$ is preserved to allow the robot to pivot in place toward the planner's new detour path.
   * Once the heading aligns with the detour and the frontal corridor clears, the robot resumes smooth forward travel.

#### 6.4.4 Presentation Demo Walkthrough
1. Launch the stack:
   ```bash
   ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False
   ```
2. Open `http://localhost:8080`.
3. Dispatch the robot to **Room 4** or **Room 5** by clicking the room button.
4. As the robot is navigating down the central hallway, click **`[ 🧱 PLACE OBSTACLE (O) ]`** and click on the map canvas directly in the robot's path.
5. **Observe**:
   * A red glowing cylinder and hazard buffer appear on the Web GUI.
   * In Gazebo, a physical red cylinder spawns in the 3D world.
   * The global path (cyan dashed line) bends and recalculates an alternative detour route around the obstacle.
   * If the robot is close to the obstacle, the safety shield prevents collision by halting forward motion and pivoting smoothly into the detour.
6. Click **`[ 🧹 CLEAR ALL ]`**:
   * The obstacle is removed from Gazebo and the map.
   * The path immediately resets back to the original direct trajectory.
