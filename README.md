# Autonomous Mobile Robot Navigation System Version - 4

A Python-based robotics project developing an **Autonomous Mobile Robot (AMR) Navigation System**.

The project progresses from controlled grid-based navigation toward a realistic autonomous robot capable of sensing its environment, localizing itself, building maps, planning routes, avoiding obstacles, and eventually operating on physical hardware.

The warehouse environment is a **development and testing foundation**, not the final objective.

---

## Long-Term Objective

The final system is intended to enable an autonomous mobile robot to:

* Perceive its surroundings through sensors
* Estimate its position and orientation
* Build and update an environment representation
* Plan safe routes
* Navigate previously unknown environments
* Detect changing obstacles
* Replan when necessary
* Execute motion under physical constraints
* Operate on physical robotic hardware

The development path is:

```text
Controlled Navigation
        ↓
Path Planning
        ↓
Dynamic Navigation
        ↓
Realistic Robot Simulation
        ↓
Sensors + Physics + Robotics Software
        ↓
Localization + Mapping
        ↓
SLAM
        ↓
Unknown-Environment Navigation
        ↓
Physical Robot
```

---

# Project Progression

| Version | Capability                                                     | Status       |
| ------- | -------------------------------------------------------------- | ------------ |
| V1      | Basic grid-based robot movement                                | Complete     |
| V2      | Configurable environment + OOP                                 | Complete     |
| V3      | Path planning with BFS, Dijkstra and A*                        | Complete     |
| **V4**  | **Dynamic autonomous navigation + replanning**                 | **Complete** |
| V5      | Realistic robotics navigation stack                            | Next         |
| V6+     | Localization, mapping, SLAM and unknown-environment navigation | Planned      |
| Later   | Physical autonomous robot                                      | Long-term    |

---

# V4 — Dynamic Autonomous Navigation

V4 is the current completed milestone.

V4 changes the problem from:

> **"What is a good route through a known environment?"**

to:

> **"Can a robot continue navigating while its environment changes?"**

The navigation system now operates as a continuous simulation rather than simply calculating a path once.

## V4 Capabilities

### Path Planning

* A* search
* Dijkstra's algorithm
* Common planner interface
* Configurable A* heuristic
* Weighted terrain costs
* Path reconstruction
* Path-cost calculation
* Nodes-explored measurement

### Dynamic Navigation

* Moving obstacles
* Multiple dynamic obstacles
* Obstacles appearing during navigation
* Obstacles disappearing during navigation
* Path invalidation
* Automatic replanning
* Unreachable-goal handling
* Dynamic obstacle avoidance

### Robot Motion

The robot now has a physical-style motion state including:

* Position
* Heading
* Linear velocity
* Angular velocity
* Maximum velocity
* Maximum acceleration
* Maximum angular velocity
* Maximum angular acceleration
* Collision radius
* Simulation timestep

Turning uses shortest-angle rotation while respecting angular motion limits.

### Collision Handling

V4 includes:

* Robot footprint collision
* Static obstacle collision
* Dynamic obstacle collision
* Swept collision detection
* Dynamic swept collision detection
* Continuous movement safety checks

This prevents the robot from simply "jumping" between grid cells and ignoring collisions between simulation steps.

### Sensor / World-State Foundations

V4 introduces the initial separation between:

```text
Sensor
   ↓
World State
   ↓
Obstacle Representation
   ↓
Planner
```

The architecture establishes the boundary required for future perception-driven navigation.

V4 does **not** claim LiDAR, localization, mapping, or SLAM. Those are future capabilities.

---

# V4 Architecture

```text
                ENVIRONMENT
                     ↓
              SENSOR / STATE
                     ↓
           OBSTACLE REPRESENTATION
                     ↓
                  PLANNER
                ↙        ↘
              A*       Dijkstra
                ↘        ↙
                 GLOBAL PATH
                      ↓
                PATH VALIDATOR
                      ↓
                REPLAN TRIGGER
                      ↓
                  CONTROLLER
                      ↓
                    ROBOT
```

The visualization layer is separate from the navigation logic and is used for debugging and observing simulation behavior.

---

# Navigation Loop

During autonomous navigation, the system repeatedly performs:

```text
Observe
   ↓
Update world state
   ↓
Check path validity
   ↓
Replan if required
   ↓
Calculate motion
   ↓
Check collision safety
   ↓
Move robot
   ↓
Repeat
```

This allows the robot to react to environmental changes while it is already moving.

---

# Planner Benchmark

V4 includes a dedicated planner benchmark using randomized static environments.

The latest benchmark used **20 environments**, with dynamic obstacles removed so that planner performance could be evaluated independently.

| Metric                 |   Dijkstra |         A* |
| ---------------------- | ---------: | ---------: |
| Success rate           |      20/20 |      20/20 |
| Average planning time  | 0.417518 s | 0.306995 s |
| Average nodes explored |     467.85 |     343.00 |
| Average path length    |      40.35 |      40.15 |
| Average path cost      |      58.55 |      58.55 |

Measured results:

* A* was approximately **1.36× faster**
* A* explored approximately **26.69% fewer nodes**
* Both planners achieved the required successful paths and equal average path cost

The benchmark is intended as an engineering measurement rather than a theoretical comparison alone.

---

# Automated Verification

V4 contains automated tests covering:

* Dynamic navigation
* Full navigation
* Replanning
* Moving obstacles
* Multiple dynamic obstacles
* Obstacle appearance/disappearance
* Unreachable goals
* Dijkstra navigation
* Configurable heuristics
* Simulation time
* Robot footprint
* Swept collision
* Dynamic swept collision
* Robot turning
* Controller behavior
* World-state handling
* Obstacle representation

The V4 test suite verifies both individual components and integrated navigation behavior.

---

# Earlier Versions

## V1 — Basic Robot

Established:

* Grid environment
* Robot movement
* Obstacles
* Goal
* Boundary handling
* Collision detection
* Basic solvability checking

## V2 — Configurable Environment + OOP

Introduced:

* Dynamic dimensions
* Random robot and goal
* Difficulty levels
* Random obstacle generation
* Replay functionality
* `Warehouse` and `Robot` classes
* Improved environment generation

## V3 — Path Planning

Introduced:

* Irregular environments
* Weighted terrain
* BFS
* Dijkstra
* A*
* Path reconstruction
* Path-cost calculation
* Algorithm comparison
* Autonomous path execution

V3 established the path-planning foundation used by V4.

---

# V5 — Realistic Robotics Navigation Stack

V5 is the next major capability jump.

The objective is to move from an algorithmic navigation simulator toward a realistic robotics software and physics architecture.

Target architecture:

```text
Environment
     ↓
Physics
     ↓
Robot
     ↓
Sensors
     ↓
Sensor Data
     ↓
Perception
     ↓
Robot World Representation
     ↓
Planner
     ↓
Controller
     ↓
Robot Motion
```

V5 will focus on:

* Differential-drive robot modelling
* Robot kinematics
* Wheel motion
* Physics-based simulation
* LiDAR sensing
* Wheel odometry
* Coordinate frames
* Robot pose
* Robotics software architecture
* Sensor-derived obstacle representation
* Navigation using sensed information rather than ground truth

The critical V5 acceptance test is:

> **If the ground-truth obstacle representation is hidden from the navigation stack, can the robot still navigate using sensor-derived information?**

V5 is intentionally not the full SLAM or unknown-environment milestone.

---

# Future Direction

After V5, the project will progressively develop:

```text
V5
Realistic Robotics Navigation
        ↓
Localization
        ↓
Mapping
        ↓
SLAM
        ↓
Unknown-Environment Navigation
        ↓
Dynamic Unknown-Environment Navigation
        ↓
Physical Robot
```

The exact version boundaries may evolve as the architecture develops. The important progression is from **known simulated environments** toward **sensor-driven autonomous operation**.

---

# Project Philosophy

The project is developed through increasing capability rather than increasing feature count.

```text
Can I make a robot move?
        ↓
Can I model an environment?
        ↓
Can I plan a route?
        ↓
Can I navigate while the environment changes?
        ↓
Can a realistic robot perceive its environment?
        ↓
Can it determine where it is?
        ↓
Can it build a map?
        ↓
Can it navigate an unknown environment?
        ↓
Can the same architecture operate on physical hardware?
```

Each milestone should add a capability that transfers toward the eventual autonomous mobile robot.

---

# Current Status

**V1 — Complete**

**V2 — Complete**

**V3 — Complete**

**V4 — Complete and merged into `main`**

**V5 — Preparation / next development milestone**

The current repository therefore represents a completed dynamic-navigation foundation rather than a final autonomous robotics system.

---

# Author

**V. Akshaj Ram Charan**
