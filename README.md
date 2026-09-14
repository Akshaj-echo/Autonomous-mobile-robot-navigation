# Autonomous Mobile Robot Navigation System

A Python-based robotics project focused on developing an **Autonomous Mobile Robot (AMR) Navigation System**.

The project progresses from controlled grid-based path planning toward autonomous navigation in increasingly realistic and uncertain environments.

Current work focuses on:

* Path planning
* Robot motion constraints
* Collision detection
* Dynamic obstacles
* Automatic replanning
* Simulated sensing
* Navigation control
* Algorithm benchmarking
* Automated verification

The long-term goal is to develop a robot capable of **localizing itself, building a map, planning and replanning routes, and autonomously navigating previously unknown environments**, eventually transferring the navigation system to physical hardware.

---

## Development Progression

```text
V1 → V2 → V3 → V4 → Sensors + Physics → Localization
   → Mapping → SLAM → Unknown-Environment Navigation → Physical Robot
```

Each version introduces a new capability toward the final autonomous navigation system.

---

# V4 — Dynamic Autonomous Navigation

## COMPLETE

V4 is the current completed milestone.

V3 demonstrated autonomous path planning through a known environment. V4 moves beyond static pathfinding by introducing a robot that must **continue navigating while its environment changes**.

## Core Capabilities

* A* and Dijkstra path planning
* Common planner interface
* Configurable A* heuristic
* Weighted terrain costs
* Robot position and heading
* Linear velocity and acceleration limits
* Angular velocity and acceleration limits
* Physical collision footprint
* Static obstacle collision detection
* Swept collision detection
* Moving obstacles
* Multiple dynamic obstacles
* Obstacle appearance and disappearance
* Path invalidation
* Automatic replanning
* Unreachable-goal handling
* Simulation timestep
* Navigation and planning metrics
* Simulated sensor layer
* World-state representation
* Automated testing

---

## V4 Architecture

```text
Environment
     ↓
Sensor / World State
     ↓
Obstacle Representation
     ↓
Planner
 ┌───┴────┐
 A*     Dijkstra
     ↓
Global Path
     ↓
Path Validator
     ↓
Replan Trigger
     ↓
Controller
     ↓
Robot
```

The system separates **environment, sensing, planning, control, and robot motion** so that increasingly realistic implementations can be introduced without rebuilding the entire architecture.

---

## Planning

V4 uses a common planner interface supporting:

### A*

Uses accumulated path cost plus a configurable heuristic to guide the search toward the goal.

### Dijkstra

Provides a heuristic-free minimum-cost planning baseline.

Both planners account for the robot's collision footprint during planning.

---

## Dynamic Navigation

The environment can change while the robot is moving.

For example:

```text
Robot follows path
       ↓
Obstacle appears / moves
       ↓
Current path becomes invalid
       ↓
Path validation detects change
       ↓
Planner runs again
       ↓
Robot follows new path
```

The system also handles obstacles disappearing and multiple dynamic obstacles changing the environment.

---

## Robot Motion Model

The robot is no longer treated as a point that can instantly move between cells.

It maintains:

* Position
* Heading
* Velocity
* Angular velocity
* Maximum velocity
* Maximum acceleration
* Collision radius
* Maximum angular velocity
* Maximum angular acceleration

Both linear and rotational movement are constrained.

This provides a foundation for eventually replacing simplified grid movement with increasingly realistic robot motion.

---

## Collision Handling

V4 checks the robot's physical footprint against obstacles rather than treating the robot as dimensionless.

Collision handling includes:

* Static obstacle checking
* Robot footprint checking
* Robot-vs-dynamic-obstacle checking
* Swept collision detection
* Moving-obstacle swept collision detection

Swept checking prevents the robot from effectively passing through an obstacle between simulation steps.

---

## Sensors and World State

V4 introduces the architectural layers required for future perception systems:

```text
Sensor
  ↓
World State
  ↓
Obstacle Representation
  ↓
Planner
```

The current sensor is intentionally simple. It is **not** a full perception, localization, mapping, or SLAM system.

Its purpose at this stage is to establish the separation between:

**what the environment actually contains**

and

**what the robot knows about it.**

---

## Simulation

Navigation operates using an explicit simulation timestep `dt`.

Simulation time is used for:

* Robot movement
* Dynamic obstacle movement
* Controller updates
* Sensor updates
* Navigation timing

The system records:

* Planning time
* Replanning time
* Path length
* Path cost
* Nodes explored
* Number of replans
* Collision count
* Simulation time
* Navigation success/failure

---

## V4 Benchmark

A planner benchmark was run across **20 randomized environments**.

Dynamic obstacles were removed from this benchmark to isolate **planner performance** from dynamic-navigation behaviour.

| Metric              |   Dijkstra |         A* |
| ------------------- | ---------: | ---------: |
| Success             |      20/20 |      20/20 |
| Avg. planning time  | 0.417518 s | 0.306995 s |
| Avg. nodes explored |     467.85 |     343.00 |
| Avg. path length    |      40.35 |      40.15 |

**A* results:**

* **1.36× average speedup**
* **26.69% fewer nodes explored**

Both planners produced the same average path cost across the benchmark.

---

## Automated Verification

V4 includes targeted tests for:

* Dynamic navigation
* Moving obstacles
* Automatic replanning
* Unreachable goals
* Dijkstra navigation
* Configurable heuristics
* Simulation time
* Obstacle appearance/disappearance
* Multiple dynamic obstacles
* Robot footprint
* Swept collision
* Dynamic swept collision
* Robot turning
* World-state architecture
* Obstacle representation

The completed V4 test suite passes, including the randomized dynamic-navigation test.

---

# Previous Versions

## V3 — Autonomous Path Planning

V3 transformed the simulator into a significantly more capable path-planning system.

Introduced:

* Irregular environments
* Weighted terrain
* BFS
* Dijkstra
* A*
* Autonomous navigation
* Algorithm comparison
* Path-cost analysis
* Explored-node measurement

**V3 question:**

> What is a good route through a known environment?

---

## V2 — Dynamic Environment + OOP

V2 introduced:

* Random environment dimensions
* Difficulty levels
* Random obstacles and goals
* Solvability checking
* Manual controls
* `Warehouse` and `Robot` classes

This established the software structure used for later versions.

---

## V1 — Foundation

V1 established the original grid-based robot environment with:

* Robot movement
* Obstacles
* Goal detection
* Collision handling
* BFS solvability checking
* Basic terminal visualization

---

# Roadmap

V4 is intentionally treated as a **frozen milestone**.

The next stages move toward realistic autonomous robotics rather than adding unrelated features to the current simulator.

### 1. Realistic Robot Simulation

More realistic robot geometry, motion, physics, and environmental interaction.

### 2. Sensors + Perception

Progress toward realistic sensor data, obstacle detection, uncertainty, and environmental perception.

### 3. Localization

Develop robot pose estimation using concepts such as odometry, IMU data, coordinate frames, and sensor fusion.

### 4. Mapping

Build representations such as occupancy grids from sensor observations.

### 5. SLAM

Combine localization and mapping so the robot can estimate its position while constructing its environment representation.

### 6. Unknown-Environment Navigation

Move from a known complete map toward navigation using information discovered by the robot.

### 7. Physical Robot

Transfer the navigation architecture to real hardware with physical sensors, actuators, and onboard computation.

---

# Project Philosophy

The project follows a progression from controlled problems toward increasingly realistic autonomous systems.

Each stage is intended to answer a more difficult robotics question.

The project does not treat the warehouse simulator as the destination.

It is the controlled laboratory in which the foundations of autonomous navigation are being developed.

---

# Current Status

```text
V1  ── COMPLETE
 ↓
V2  ── COMPLETE
 ↓
V3  ── COMPLETE
 ↓
V4  ── COMPLETE
 ↓
Realistic Robot Simulation
 ↓
Sensors + Perception
 ↓
Localization
 ↓
Mapping
 ↓
SLAM
 ↓
Unknown-Environment Navigation
 ↓
Physical Autonomous Robot
```

**Current milestone: V4 — Dynamic Autonomous Navigation**

The project has progressed from basic grid movement to a modular navigation system capable of **planning, executing, monitoring, and replanning in changing environments**.

The next major challenge is moving from **known simulated environments** toward robots that must **perceive, localize, map, and navigate through environments they do not already know**.

---

# Final Vision

The project began with a simple question:

> **What is the best path from A to B?**

V3 expanded that into:

> **How can an autonomous robot find and follow a good path through a complex environment?**

V4 expanded it again:

> **How can a robot continue navigating when its environment changes while it is moving?**

The eventual objective is a much larger problem:

> **How can a robot enter an unfamiliar environment, perceive its surroundings, determine where it is, build a representation of the environment, plan a safe route, and autonomously navigate to its destination?**

The current system is one stage in that progression.

**V4 establishes the dynamic navigation foundation.**

The next stages will develop sensing, localization, mapping, SLAM, unknown-environment navigation, and eventually physical autonomous robotics.

---

# Author

**V. Akshaj Ram Charan**
