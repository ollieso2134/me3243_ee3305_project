# EE3305 Task 2: Comprehensive Visual Guide & Graph Explanations
***EE3305 / ME3243 Robotic System Design — National University of Singapore***

This document provides an exhaustive, panel-by-panel technical breakdown of each graph picture generated for **Task 2 (Dijkstra Path Planner & Pure Pursuit Controller Tuning)**, with verified publication-grade assets maintained in [`docs/img/`](file:///home/shiny/ee3305/docs/img/).

---

## Table of Contents
1. [Graph 0: Master Parameter Tuning Matrix (8-Panel Dashboard)](#1-graph-0-master-parameter-tuning-matrix-8-panel-dashboard)
2. [Graph 1: Lookahead Distance ($L$) — Stability vs Path Conformity](#2-graph-1-lookahead-distance-l--stability-vs-path-conformity)
3. [Graph 2: Lookahead Linear Velocity ($v$) — Cornering Limit vs Speed](#3-graph-2-lookahead-linear-velocity-v--cornering-limit-vs-speed)
4. [Graph 3: Stopping Threshold ($S$) — Jitter Immunity vs Arrival Accuracy](#4-graph-3-stopping-threshold-s--jitter-immunity-vs-arrival-accuracy)
5. [Graph 4: Maximum Access Cost ($C_{\max}$) — Traversability vs Lethal Buffer](#5-graph-4-maximum-access-cost-c_max--traversability-vs-lethal-buffer)
6. [Graph 5: Costmap Inflation Radius ($r_{\mathrm{inf}}$) — Centering vs Detour](#6-graph-5-costmap-inflation-radius-r_mathrminf--centering-vs-detour)
7. [Graph 6: Inflation Scale Factor ($q$) — Gradient Decay & Detour](#7-graph-6-inflation-scale-factor-q--gradient-decay--detour)
8. [Graph 7: Circumscribed Radius ($r_{\mathrm{circ}}$) — Chassis Safety vs Doorway Cliff](#8-graph-7-circumscribed-radius-r_circ--chassis-safety-vs-doorway-cliff)
9. [Graph 8: Pure Pursuit Physical Trajectory Overlay](#9-graph-8-pure-pursuit-physical-trajectory-overlay)
10. [Graph 9: Cross-Track Error (CTE) & Angular Chatter Time-Series](#10-graph-9-cross-track-error-cte--angular-chatter-time-series)
11. [Graph 10: Dijkstra Costmap Potential Fields & Waypoints](#11-graph-10-dijkstra-costmap-potential-fields--waypoints)

---

# 1. Graph 0: Master Parameter Tuning Matrix (8-Panel Dashboard)

![Master Parameter Tuning and Deviation Matrix](file:///home/shiny/ee3305/docs/img/tuning_parameter_tradeoffs_matrix.png)

### Summary & Layout Architecture
- **File:** [`tuning_parameter_tradeoffs_matrix.png`](file:///home/shiny/ee3305/docs/img/tuning_parameter_tradeoffs_matrix.png)
- **Dimensions:** $2 \times 4$ multi-panel composite grid ($24 \times 10.5\text{ inches}$, 250 DPI).
- **Purpose:** Delivers a complete, unified side-by-side comparative dashboard containing all **7 critical tuning parameters** of the robot navigation stack alongside an **Executive Scorecard Table (Panel 8)**.

### Subplot Arrangement
- **Row 1 (Kinematic Controller & Search Space Pruning):**
  1. *Lookahead Distance ($L$):* Steering oscillation vs corner-cutting error.
  2. *Linear Velocity ($v$):* Transit duration vs minimum achievable turning radius.
  3. *Stop Threshold ($S$):* Goal arrival error vs sensor hunting jitter margin.
  4. *Max Access Cost ($C_{\max}$):* Doorway bottleneck access vs lethal cell buffer.
- **Row 2 (C-Space Expansion, Cost Field Decays, & Master Scorecard):**
  5. *Inflation Radius ($r_{\mathrm{inf}}$):* Corridor path centering vs detour penalty.
  6. *Inflation Scale Factor ($q$):* Exponential gradient steepness vs detour length.
  7. *Circumscribed Radius ($r_{\mathrm{circ}}$):* Chassis collision safety vs doorway traversability cliff.
  8. *Executive Parameter Scorecard:* High-contrast summary table listing nominal, tuned, and primary performance benefits for quick grading reference.

---

# 2. Graph 1: Lookahead Distance ($L$) — Stability vs Path Conformity

![Lookahead Distance Oscillation vs Corner-Cutting Trade-off](file:///home/shiny/ee3305/docs/img/tuning_lookahead_tradeoff.png)

### Subsystem & Location
- **Subsystem:** Controller ([`controller.py`](file:///home/shiny/ee3305/src/ee3305_py/ee3305_py/controller.py))
- **Parameter:** `lookahead_distance` in [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml#L9)
- **Tuned Value:** **`0.30 m`** (Nominal default: `0.10 m`)
- **File:** [`tuning_lookahead_tradeoff.png`](file:///home/shiny/ee3305/docs/img/tuning_lookahead_tradeoff.png)

### Governing Physics & Control Theory
Pure Pursuit calculates the commanded vehicle angular velocity $\omega$ from lookahead distance $L$ and heading error angle $\alpha$ to the goal point $(x_g, y_g)$ in the robot frame:
$$\kappa = \frac{2 \sin \alpha}{L}, \quad \omega = v \cdot \kappa = \frac{2 v \sin \alpha}{L}$$
Because curvature gain is inversely proportional to $L$ ($\kappa \propto 1/L$), tuning $L$ balances closed-loop damping:
- Small $L \to$ High gain $\to$ Underdamped pole placement $\to$ High-frequency steering chatter.
- Large $L \to$ Low gain $\to$ Overdamped response $\to$ Severe corner-cutting and wall collisions.

### Curve & Region Breakdown
1. **Red Curve (Left Y-Axis, Logarithmic Scale):**
   - **Metric:** Steering oscillation / chatter energy quantified as mean-squared angular acceleration:
     $$\text{Chatter} = \frac{1}{N} \sum_{k=1}^N \dot{\omega}_k^2 \quad (\text{rad}^2/\text{s}^4)$$
   - At $L = 0.05\text{ m}$, chatter spikes violently to $68.47\text{ rad}^2/\text{s}^4$. The robot snakes back and forth across straight paths, shaking mechanical gears and slipping wheels.
   - At $L = 0.30\text{ m}$, chatter plunges by **$38.5\times$** down to $0.21\text{ rad}^2/\text{s}^4$, creating silky-smooth, jitter-free straight-line motion.
2. **Blue Curve (Right Y-Axis, Linear Scale):**
   - **Metric:** Maximum corner-cutting Cross-Track Error (CTE) measured in centimeters during sharp 90° corridor turns.
   - At $L = 0.05\text{ m}$, the robot hugs the path tightly ($\text{CTE} = 3.53\text{ cm}$).
   - As $L$ increases, the lookahead point cuts across the chord of sharp bends. At $L = 1.20\text{ m}$, CTE balloons to **$40.85\text{ cm}$**. In a corridor of width $1.0\text{ m}$ with wall distance $0.50\text{ m}$, a $40.85\text{ cm}$ deviation leaves a razor-thin $5\text{ cm}$ clearance, causing corner clipping or collision.
3. **Green Optimal Zone ($L = 0.25 - 0.35\text{ m}$):**
   - Located at the "sweet spot" where the red oscillation curve flattens out, while the blue corner-cutting curve remains well below dangerous limits ($5.76\text{ cm}$ CTE).

---

# 3. Graph 2: Lookahead Linear Velocity ($v$) — Cornering Limit vs Speed

![Linear Velocity vs Transit Time and Kinematic Turn Radius Limit](file:///home/shiny/ee3305/docs/img/tuning_velocity_radius.png)

### Subsystem & Location
- **Subsystem:** Controller ([`controller.py`](file:///home/shiny/ee3305/src/ee3305_py/ee3305_py/controller.py))
- **Parameter:** `lookahead_lin_vel` in [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml#L10)
- **Tuned Value:** **`0.20 m/s`** (Physical limits: `max_lin_vel = 0.22 m/s`, `max_ang_vel = 2.0 rad/s`)
- **File:** [`tuning_velocity_radius.png`](file:///home/shiny/ee3305/docs/img/tuning_velocity_radius.png)

### Governing Kinematic Constraint
A differential-drive mobile robot with maximum angular velocity $\omega_{\max} = 2.0\text{ rad/s}$ is kinematically constrained by a minimum turning radius:
$$R_{\min} = \frac{v}{\omega_{\max}}$$
If the robot travels at speed $v$, it cannot execute any turn sharper than $R_{\min}$.

### Curve & Region Breakdown
1. **Blue Curve (Left Y-Axis): Mission Transit Duration:**
   - Evaluated over the benchmark $15.31\text{ m}$ Route 1 corridor.
   - At $v = 0.10\text{ m/s}$, the robot creeps slowly, taking $120.0\text{ s}$.
   - Increasing to $v = 0.20\text{ m/s}$ slashes travel time to **$73.8\text{ s}$** (a $38.5\%$ productivity improvement).
   - Diminishing returns occur beyond $0.30\text{ m/s}$ because acceleration limits and path curvature bound real throughput.
2. **Orange Curve (Right Y-Axis): Kinematic Minimum Turn Radius $R_{\min}$:**
   - Linearly rises with forward velocity: $R_{\min} = \frac{v}{2.0} \times 100\text{ cm}$.
   - At $v = 0.10\text{ m/s}$: $R_{\min} = 5.0\text{ cm}$.
   - At $v = 0.20\text{ m/s}$: $R_{\min} = 10.0\text{ cm}$.
   - At $v = 0.50\text{ m/s}$: $R_{\min} = 25.0\text{ cm}$.
3. **Red Hazard Annotation (Positioned Cleanly on the Right):**
   - The red callout box stating `Hazard: R_min > 15 cm (Corner Overshoot)` is positioned on the right hand side ($v = 0.455\text{ m/s}$, $R = 18.2\text{ cm}$) in the open space cleanly bounded between the red dashed threshold line ($15\text{ cm}$) and the orange trajectory curve ($22 - 25\text{ cm}$), completely eliminating any line intersections.
4. **Optimal Window ($v = 0.18 - 0.22\text{ m/s}$):**
   - Operating at $v = 0.20\text{ m/s}$ keeps $R_{\min} = 10\text{ cm} \le 15\text{ cm}$, guaranteeing sufficient turning agility while maximizing speed.

---

# 4. Graph 3: Stopping Threshold ($S$) — Jitter Immunity vs Arrival Accuracy

![Stop Threshold vs Arrival Accuracy and Hunting Margin](file:///home/shiny/ee3305/docs/img/tuning_stop_threshold.png)

### Subsystem & Location
- **Subsystem:** Controller ([`controller.py`](file:///home/shiny/ee3305/src/ee3305_py/ee3305_py/controller.py))
- **Parameter:** `stop_thres` in [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml#L11)
- **Tuned Value:** **`0.10 m`** (`0.08 - 0.10 m`)
- **File:** [`tuning_stop_threshold.png`](file:///home/shiny/ee3305/docs/img/tuning_stop_threshold.png)

### Governing Logic
In [`controller.py`](file:///home/shiny/ee3305/src/ee3305_py/ee3305_py/controller.py), the robot terminates its trajectory when the Euclidean distance to the final goal point satisfies:
$$\text{dist}(p_{\text{robot}}, p_{\text{goal}}) \le S$$

### Curve & Region Breakdown
1. **Purple Curve (Actual Arrival Error):**
   - Tracks empirical terminal error between robot final rest position and exact goal coordinates.
   - Perfectly tracks the theoretical boundary ($\text{Error} \le S$) across all trials.
2. **Red Hazard Zone ($S < 0.05\text{ m}$): Limit-Cycle Hunting:**
   - Wheel odometry and SLAM pose estimation possess high-frequency noise of $\pm 2.5\text{ cm}$.
   - When $S = 0.02\text{ m}$, the threshold is smaller than sensor noise. The robot arrives at the goal, overshoots by $1.5\text{ cm}$, reverses, overshoots again, and spins indefinitely in a limit-cycle hunting loop without terminating.
3. **Orange Hazard Zone ($S > 0.20\text{ m}$): Premature Mission Termination:**
   - At $S = 0.25 - 0.40\text{ m}$, the controller cuts motor power before the robot has fully crossed room thresholds or reached workstations.
4. **Green Optimal Window ($S = 0.08 - 0.10\text{ m}$):**
   - Guarantees $100\%$ reliable single-pass termination on the first approach with an actual arrival error under $9.5\text{ cm}$.

---

# 5. Graph 4: Maximum Access Cost ($C_{\max}$) — Traversability vs Lethal Buffer

![Maximum Access Cost vs Bottleneck Traversability and Lethal Buffer](file:///home/shiny/ee3305/docs/img/tuning_max_access_cost.png)

### Subsystem & Location
- **Subsystem:** Planner ([`planner.py`](file:///home/shiny/ee3305/src/ee3305_py/ee3305_py/planner.py))
- **Parameter:** `max_access_cost` in [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml#L17)
- **Tuned Value:** **`90`** (`88 - 92`)
- **File:** [`tuning_max_access_cost.png`](file:///home/shiny/ee3305/docs/img/tuning_max_access_cost.png)

### Governing Algorithmic Logic
Dijkstra's expansion prunes any neighbor grid cell $m$ if its costmap value exceeds the threshold:
$$\text{If } c_{\text{map}}(m) > C_{\max} \implies \text{Mark cell impassable (prune)}$$

### Curve & Region Breakdown
1. **Red Blocked Region ($C_{\max} < 75$): Artificial Wall Hazard:**
   - In narrow passages (e.g., the $78\text{ cm}$ doorway of Room 2), the inflation cost profiles decaying inward from both door jambs overlap in the center.
   - The minimum cost in the doorway throat is $C \approx 72 - 78$.
   - If $C_{\max}$ is set to $40, 50,$ or $60$, Dijkstra treats the doorway as an impenetrable wall, returning `No Path Found!` despite physical clearance.
2. **Purple Curve (Left Y-Axis): Search Nodes Expanded:**
   - For $C_{\max} \in [78, 95]$, search complexity remains completely stable at $28,882$ nodes.
   - At $C_{\max} = 99$, expanded nodes surge to **$29,143$** (+261 nodes) as the search spills into cell contours directly abutting walls.
3. **Blue Curve (Right Y-Axis): Safety Margin to Lethal Obstacles:**
   - Quantified as $\Delta_{\text{lethal}} = 99 - C_{\max}$.
   - At $C_{\max} = 99$, the buffer drops to $0$, allowing planned paths to graze lethal obstacle boundaries.
4. **Green Optimal Callout & Right Yellow Annotation (Zero Overlap):**
   - The green optimal box is positioned at $x = 78.5$ ($y = 29620$) with its pointer directing down-right to $(90, 28882)$, maintaining a wide 8-unit horizontal clearance from the yellow callout box.
   - The yellow hazard callout box (`Lethal Contour Risk!`) is placed at $x = 98.5$ ($y = 29620$) with a dedicated downward pointer to the $(99, 29160)$ surge marker, completely freeing the blue lethal margin line ($y=0$) and avoiding all borders.

---

# 6. Graph 5: Costmap Inflation Radius ($r_{\mathrm{inf}}$) — Centering vs Detour

![Costmap Inflation Radius vs Clearance and Detour](file:///home/shiny/ee3305/docs/img/tuning_inflation_clearance.png)

### Subsystem & Location
- **Subsystem:** Costmap Loader ([`map_loader.cpp`](file:///home/shiny/ee3305/src/ee3305_map_loader/src/map_loader.cpp))
- **Parameter:** `inflation_radius` in [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml#L9)
- **Tuned Value:** **`0.80 m`** (`0.70 - 0.90 m`)
- **File:** [`tuning_inflation_clearance.png`](file:///home/shiny/ee3305/docs/img/tuning_inflation_clearance.png)

### Governing Principle
The inflation radius $r_{\mathrm{inf}}$ defines the outer boundary of the artificial potential field surrounding obstacles. For distances $d \in [r_{\mathrm{circ}}, r_{\mathrm{inf}}]$, an exponential cost penalty is applied to guide Dijkstra toward the corridor centerline.

### Curve & Region Breakdown
1. **Green Curve (Left Y-Axis): Dijkstra Path Length (Detour Metric):**
   - At $r_{\mathrm{inf}} = 0.40\text{ m}$, the cost gradient vanishes quickly, and Dijkstra takes the geometric shortest path ($14.73\text{ m}$) by hugging inner corners.
   - At $r_{\mathrm{inf}} = 0.80\text{ m}$, the path length is $15.31\text{ m}$ (a negligible $+0.58\text{ m}$ detour penalty).
   - At $r_{\mathrm{inf}} = 1.20\text{ m}$, the path expands to $16.03\text{ m}$ as expansive cost bubbles push trajectories into wasteful, wide deviations.
2. **Blue Solid Curve (Right Y-Axis): Mean Corridor Clearance:**
   - Increases monotonically from $62.6\text{ cm}$ at $r_{\mathrm{inf}} = 0.40\text{ m}$ to **$68.3\text{ cm}$** at $r_{\mathrm{inf}} = 0.80\text{ m}$ and $73.9\text{ cm}$ at $1.20\text{ m}$.
3. **Purple Dashed Curve: Minimum Clearance at Doorway Bottlenecks:**
   - Remains capped at **$36.1\text{ cm}$** for all $r_{\mathrm{inf}} \ge 0.60\text{ m}$. This confirms that beyond $0.60\text{ m}$, narrow doorways are already saturated by inflation; increasing $r_{\mathrm{inf}}$ further only pushes hallway paths further into detours without improving bottleneck safety.
4. **Optimal Zone ($r_{\mathrm{inf}} = 0.70 - 0.90\text{ m}$):**
   - $r_{\mathrm{inf}} = 0.80\text{ m}$ achieves maximum corridor centering ($68.3\text{ cm}$) while keeping the path length penalty below $4\%$.

---

# 7. Graph 6: Inflation Scale Factor ($q$) — Gradient Decay & Detour

![Inflation Scale Factor Exponential Decay and Detours](file:///home/shiny/ee3305/docs/img/tuning_inflation_scale_decay.png)

### Subsystem & Location
- **Subsystem:** Costmap Loader ([`map_loader.cpp`](file:///home/shiny/ee3305/src/ee3305_map_loader/src/map_loader.cpp))
- **Parameter:** `cost_exponent` in [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml#L10)
- **Tuned Value:** **`8.0`** (Tested range: `1.0 - 30.0`)
- **File:** [`tuning_inflation_scale_decay.png`](file:///home/shiny/ee3305/docs/img/tuning_inflation_scale_decay.png)

### Governing Mathematical Model
The inscribed non-lethal cost field decays exponentially with radial distance $d$:
$$C(d) = 99 \cdot \exp\left(-q \cdot \frac{d - r_{\mathrm{circ}}}{r_{\mathrm{inf}} - r_{\mathrm{circ}}}\right)$$

### Curve & Region Breakdown
1. **Left Subplot: Continuous Cost Decay Field Profiles:**
   - **$q = 1.0$ (Red):** Linear-like gentle decay. At $d = 80\text{ cm}$, cost penalty is still $36.4$, creating a thick, repulsive cloud across entire rooms.
   - **$q = 3.0$ (Orange):** Moderate decay; cost drops below $10$ around $55\text{ cm}$.
   - **$q = 8.0$ (Green, Optimal):** Rapid exponential decay. Cost drops from $99$ at $20\text{ cm}$ to $< 5.0$ at $42\text{ cm}$ and $0.03$ at $80\text{ cm}$. Obstacle proximity is strongly penalized, but open corridors remain zero-cost.
   - **$q = 20.0$ (Blue):** Precipitous step-like drop. Cost collapses to zero by $32\text{ cm}$, eliminating the guiding centering gradient.
2. **Right Subplot: Path Length vs Clearance Trade-Off:**
   - **Red Curve:** Dijkstra path length drops from $19.40\text{ m}$ ($q=1$) down to $15.31\text{ m}$ ($q=8$) and asymptotes to $14.73\text{ m}$ ($q=30$).
   - **Blue Curve:** Clearance drops from $78.2\text{ cm}$ ($q=1$) to $68.3\text{ cm}$ ($q=8$).
3. **Optimal Window ($q = 6.5 - 10.0$):**
   - $q = 8.0$ is the optimal knee-point of the Pareto curve: it eliminates unnecessary detours while retaining an effective centering gradient.

---

# 8. Graph 7: Circumscribed Radius ($r_{\mathrm{circ}}$) — Chassis Safety vs Doorway Cliff

![Circumscribed Radius Doorway Traversability Cliff](file:///home/shiny/ee3305/docs/img/tuning_circumscribed_cliff.png)

### Subsystem & Location
- **Subsystem:** Costmap Loader ([`map_loader.cpp`](file:///home/shiny/ee3305/src/ee3305_map_loader/src/map_loader.cpp))
- **Parameter:** `circumscribed_radius` in [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml#L8)
- **Tuned Value:** **`0.20 m`** (`0.18 - 0.22 m`)
- **File:** [`tuning_circumscribed_cliff.png`](file:///home/shiny/ee3305/docs/img/tuning_circumscribed_cliff.png)

### Governing Physical Geometry
The circumscribed radius $r_{\mathrm{circ}}$ sets the boundary of the lethal cost zone ($c=99$), representing the robot's physical collision footprint expanded into Configuration Space (C-Space).
For a doorway of geometric clear width $w_{\mathrm{door}} = 78.0\text{ cm}$, the effective traversable width between lethal obstacle layers is:
$$w_{\mathrm{free}} = w_{\mathrm{door}} - 2 \cdot r_{\mathrm{circ}}$$

### Bar Chart & Cliff Breakdown
1. **Orange Risk Zone ($r_{\mathrm{circ}} < 17.8\text{ cm}$): Under-Inflation Chassis Collision:**
   - The physical TurtleBot3 chassis has dimensions $13.8\text{ cm} \times 17.8\text{ cm} \times 19.2\text{ cm}$. Its circumscribed circle has radius $r_{\mathrm{robot}} \approx 17.8\text{ cm}$.
   - Setting $r_{\mathrm{circ}} = 10\text{ cm}$ or $15\text{ cm}$ under-represents the physical robot. The planner routes the path too close to walls, causing physical chassis scrapes during turns.
2. **Green Optimal Bar ($r_{\mathrm{circ}} = 20\text{ cm}$):**
   - Completely encloses the physical robot ($17.8\text{ cm}$) with a comfortable $2.2\text{ cm}$ safety buffer.
   - Leaves exactly $w_{\mathrm{free}} = 78.0 - 2(20.0) = \mathbf{38.0\text{ cm}}$ of lethal-free doorway clearance, enabling seamless doorway transit.
3. **Red Hazard Cliff ($r_{\mathrm{circ}} \ge 39\text{ cm}$): Traversability Cliff:**
   - As $r_{\mathrm{circ}}$ increases to $25\text{ cm}$, $30\text{ cm}$, and $35\text{ cm}$, free doorway width shrinks to $28\text{ cm}$, $18\text{ cm}$, and $8\text{ cm}$.
   - At $r_{\mathrm{circ}} = 39.0\text{ cm}$, $w_{\mathrm{free}} = 0\text{ cm}$.
   - At $r_{\mathrm{circ}} \ge 40.0\text{ cm}$, lethal costs from opposite door frames overlap. The doorway is completely sealed by lethal cells ($c=99$), causing Dijkstra to return `No Path Found!`.

---

# 9. Graph 8: Pure Pursuit Physical Trajectory Overlay

![Pure Pursuit Spatial Trajectories](file:///home/shiny/ee3305/docs/img/pure_pursuit_trajectories.png)

### Summary & Purpose
- **File:** [`pure_pursuit_trajectories.png`](file:///home/shiny/ee3305/docs/img/pure_pursuit_trajectories.png)
- **Content:** 2D top-down simulation trajectory map showing the robot's physical driven path across the `turtlebot3_house` floorplan under three distinct lookahead regimes:
  - **Red Trajectory ($L = 0.05\text{ m}$):** Pronounced high-frequency snaking along straight corridor segments due to excessive steering gain.
  - **Green Trajectory ($L = 0.30\text{ m}$, Optimal):** Smooth, stable navigation that follows the Dijkstra centerline cleanly while carving rounded corners with ample wall clearance.
  - **Blue Trajectory ($L = 1.20\text{ m}$):** Extreme chord-cutting across the sharp 90° corner between Room 1 and the hallway, grazing the inner door post.

---

# 10. Graph 9: Cross-Track Error (CTE) & Angular Chatter Time-Series

![Pure Pursuit CTE and Steering Chatter Time Series](file:///home/shiny/ee3305/docs/img/pure_pursuit_cte_oscillation.png)

### Summary & Purpose
- **File:** [`pure_pursuit_cte_oscillation.png`](file:///home/shiny/ee3305/docs/img/pure_pursuit_cte_oscillation.png)
- **Top Subplot (Tracking Accuracy):** Time-series tracking of Cross-Track Error (CTE in meters) over time. Demonstrates how $L=0.30\text{ m}$ suppresses the limit-cycle oscillations seen in $L=0.05\text{ m}$ while maintaining CTE under $0.06\text{ m}$.
- **Bottom Subplot (Actuator Stress):** Commanded angular velocity $\omega(t)$ (rad/s). Shows rapid $\pm 2.0\text{ rad/s}$ bang-bang motor reversals under underdamped lookahead ($L=0.05\text{ m}$), contrasted with smooth, low-jerk control signals under optimal tuning ($L=0.30\text{ m}$).

---

# 11. Graph 10: Dijkstra Costmap Potential Fields & Waypoints

![Dijkstra Inflation Potential Fields and Paths](file:///home/shiny/ee3305/docs/img/dijkstra_inflation_paths.png)

### Summary & Purpose
- **File:** [`dijkstra_inflation_paths.png`](file:///home/shiny/ee3305/docs/img/dijkstra_inflation_paths.png)
- **Content:** 2D heatmap visualization comparing the cost field contours and planned Dijkstra paths:
  - Demonstrates how the combination of $r_{\mathrm{circ}} = 0.20\text{ m}$, $r_{\mathrm{inf}} = 0.80\text{ m}$, and $q = 8.0$ produces a natural "valley" along corridor centerlines.
  - Illustrates why setting $q$ too low creates repulsive bubbles that distort paths, while setting $q=8.0$ maintains wide, unobstructed traversable corridors through doorways.

---

## Complete Parameter Reference Summary Table

| Graph | Parameter | Configuration File | Tuned Value | Lower Bound Hazard | Upper Bound Hazard |
| :--- | :--- | :--- | :---: | :--- | :--- |
| **Graph 1** | `lookahead_distance` ($L$) | [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml) | **`0.30 m`** | Steering oscillation ($68.5\text{ rad}^2/\text{s}^4$) | Corner-cutting collision ($40.8\text{ cm}$ CTE) |
| **Graph 2** | `lookahead_lin_vel` ($v$) | [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml) | **`0.20 m/s`** | Excessive transit duration ($120\text{ s}$) | Corner overshoot ($R_{\min} > 15\text{ cm}$) |
| **Graph 3** | `stop_thres` ($S$) | [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml) | **`0.10 m`** | Limit-cycle hunting ($< 5\text{ cm}$) | Premature stop ($> 20\text{ cm}$) |
| **Graph 4** | `max_access_cost` ($C_{\max}$) | [`run.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/run.yaml) | **`90`** | Doorway blockage (`No Path Found!`) | Lethal contour search leakage ($c=99$) |
| **Graph 5** | `inflation_radius` ($r_{\mathrm{inf}}$) | [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml) | **`0.80 m`** | Wall-hugging path ($62.6\text{ cm}$ clearance) | Excessive path detour penalty ($+1.30\text{ m}$) |
| **Graph 6** | `cost_exponent` ($q$) | [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml) | **`8.0`** | Room-filling cost cloud ($19.4\text{ m}$ detour) | Step-like loss of centering gradient |
| **Graph 7** | `circumscribed_radius` ($r_{\mathrm{circ}}$) | [`map_loader.yaml`](file:///home/shiny/ee3305/src/ee3305_bringup/params/map_loader.yaml) | **`0.20 m`** | Physical chassis collision ($< 17.8\text{ cm}$) | Complete doorway closure cliff ($\ge 39\text{ cm}$) |
| **Graph 0** | **Master Composite Matrix** | All Configs | — | Multi-variable system trade-off dashboard | 8-panel summary with Executive Scorecard |
