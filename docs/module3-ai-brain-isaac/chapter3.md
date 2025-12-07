# Chapter 11: Navigation (Nav2) + Biped Concepts

## Learning Goals
- Understand the components and workflow of the ROS 2 Navigation Stack (Nav2).
- Learn to configure Nav2 for autonomous navigation in simulated environments.
- Comprehend fundamental concepts of bipedal locomotion for humanoid robots.
- Explore challenges and approaches for humanoid navigation and balance control.
- Gain insight into integrating perception data with Nav2 for robust navigation.

## Prerequisites
Familiarity with ROS 2 core concepts (Module 1), Isaac Sim (Chapter 9), and Isaac ROS perception (Chapter 10).

## Key Concepts

### Introduction to ROS 2 Navigation Stack (Nav2)
Nav2 is the next-generation navigation framework for ROS 2. It enables robots to autonomously navigate complex environments by integrating various modules for mapping, localization, path planning, and control. Nav2 is highly modular and configurable, supporting a wide range of robot types and navigation strategies.

### Nav2 Architecture Overview
Nav2 is composed of several independent ROS 2 nodes that work together:
-   **Map Server**: Provides a static map of the environment.
-   **AMCL (Adaptive Monte Carlo Localization)**: A probabilistic localization algorithm that estimates the robot's pose within a known map.
-   **WayPoint Follower**: Executes a sequence of predefined waypoints.
-   **Planner Servers (Global and Local)**:
    -   **Global Planner**: Computes a collision-free path from the robot's current location to a distant goal (e.g., using A* or Dijkstra's).
    -   **Local Planner (Controller)**: Generates velocity commands to follow the global path and avoid local obstacles (e.g., using DWA, TEB).
-   **Behavior Tree**: Orchestrates navigation behaviors (e.g., waiting, recovering from errors, following a path).
-   **Costmap Filters**: Dynamically modify costmaps based on sensor data (e.g., removing static obstacles).

### Standard Nav2 Workflow
1.  **Map Creation**: A map of the environment is typically created beforehand using SLAM (Simultaneous Localization and Mapping) or provided as a static file (e.g., `.pgm`, `.yaml`).
2.  **Localization**: The robot uses AMCL (or another localization method) to determine its position on the map.
3.  **Goal Setting**: A navigation goal (target pose) is sent to Nav2.
4.  **Global Planning**: The global planner computes a path from the robot's current pose to the goal, avoiding known obstacles on the static map.
5.  **Local Planning/Control**: The local planner continuously generates velocity commands, following the global path while dynamically avoiding new, unforeseen obstacles detected by sensors.
6.  **Execution**: Velocity commands are sent to the robot's base controller.
7.  **Feedback**: The robot provides feedback on its current pose and any detected obstacles, closing the loop.

### Bipedal Locomotion Concepts
Bipedal locomotion, the act of walking on two legs, is inherently complex and challenging for robots, especially humanoids. Key concepts include:
-   **Center of Mass (CoM)**: The average position of all the mass in the robot. Maintaining the CoM within the **Support Polygon** is critical for stability.
-   **Support Polygon**: The convex hull formed by the contact points of the robot's feet with the ground. As long as the CoM projection is within this polygon, the robot is statically stable.
-   **Zero Moment Point (ZMP)**: A concept used in dynamic stability, representing the point on the ground where the robot can apply no net moment. Keeping the ZMP within the support polygon ensures dynamic balance during walking.
-   **Gait Generation**: Algorithms that determine the sequence of joint movements and foot placements to achieve stable and efficient walking (e.g., ZMP-based gait, capture point methods).
-   **Balance Control**: Real-time adjustments to joint torques or positions based on IMU feedback to maintain equilibrium, especially during disturbances.

### Humanoid Navigation Challenges
-   **High Degrees of Freedom**: Humanoids have many joints, making kinematic control and balance complex.
-   **Dynamic Stability**: Unlike wheeled robots, humanoids are inherently unstable and require continuous balance control, especially during walking or manipulation.
-   **Footstep Planning**: Deciding where to place feet to navigate obstacles and uneven terrain while maintaining stability.
-   **Manipulation while Navigating**: Performing tasks with arms while walking significantly complicates balance.
-   **Integration with Environment**: Navigating human-centric environments requires understanding complex spatial layouts and interacting with objects/humans.

### Integrating Perception with Nav2
Advanced perception (e.g., from Isaac ROS) plays a vital role in enhancing Nav2's capabilities:
-   **Dynamic Obstacle Avoidance**: Real-time object detection and tracking feed into Nav2's local costmap, allowing the robot to avoid moving obstacles.
-   **Semantic Mapping**: Object recognition can create maps with semantic information (e.g., identifying "chairs," "tables"), enabling more intelligent navigation behaviors (e.g., "go to the kitchen table").
-   **AprilTag Localization**: Using AprilTags as robust landmarks for more accurate global localization within Nav2.
-   **Uneven Terrain Navigation**: Depth cameras and 3D point clouds inform Nav2 about traversable surfaces, crucial for humanoids on challenging terrain.

## Diagrams
-   **Diagram 1: Nav2 stack overview**
    *   *Description*: A block diagram illustrating the main components of the Nav2 stack (Map Server, AMCL, Planner Servers, Controller Server, Behavior Tree) and their interactions. Show data flow via ROS 2 topics/services/actions between these nodes, leading to velocity commands for the robot base.
-   **Diagram 2: Bipedal locomotion concepts**
    *   *Description*: A simple diagram of a humanoid robot standing or taking a step. Illustrate and label the Center of Mass (CoM), Support Polygon, and Zero Moment Point (ZMP) during different phases of locomotion (e.g., double support, single support). Explain their relationship to stability.

## Examples

### Basic Nav2 Configuration for a Wheeled Robot (Conceptual)
While humanoids are complex, understanding Nav2 starts with simpler robots. This is a conceptual example for a mobile robot.

1.  **Robot Description**: A URDF/SDF for a wheeled robot with a LiDAR sensor.
2.  **Static Map**: A `.pgm` image and `.yaml` file describing an office environment.
3.  **ROS 2 Launch File (Conceptual)**:

```xml
<!-- minimal_navigation.launch.xml (Conceptual) -->
<launch>
  <arg name="map" default="path/to/my_map.yaml"/>
  <arg name="robot_model" default="path/to/my_robot.urdf"/>

  <node pkg="tf2_ros" exec="static_transform_publisher" name="map_to_odom_tf" args="0 0 0 0 0 0 map odom"/>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" command="$(find-pkg-share xacro)/xacro --inorder $(var robot_model)" />
  </node>

  <include file="$(find nav2_bringup)/launch/bringup_launch.py">
    <arg name="map" value="$(var map)"/>
    <arg name="params_file" value="path/to/nav2_params.yaml"/>
    <!-- ... other Nav2 parameters ... -->
  </include>

  <!-- Simulated LiDAR publisher (from Isaac Sim or Gazebo) -->
  <node pkg="ros_gz_bridge" exec="ros_gz_bridge" name="lidar_bridge" type="sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan">
    <remap from="/lidar" to="/scan"/>
  </node>
</launch>
```

4.  **Nav2 Parameters (`nav2_params.yaml`)**:
    *   Configuring `amcl` (e.g., `min_particles`, `max_particles`, `transform_tolerance`).
    *   Setting up `global_planner` (e.g., `planner_id`, `use_astar`).
    *   Defining `local_controller` (e.g., `min_vel_x`, `max_vel_x`, `acc_lim_x`).

### Bipedal Gaits (Conceptual)
Research into bipedal gaits often involves specialized libraries or custom controllers, distinct from Nav2's typical wheeled robot focus.

1.  **State Machine**: A control algorithm could use a state machine to transition between different phases of walking (e.g., double support, single support, swing phase).
2.  **Inverse Kinematics**: Calculate required joint angles for desired end-effector (foot) positions.
3.  **ZMP Control**: Adjust joint trajectories to keep the ZMP within the support polygon for dynamic stability.

## Hands-on Exercises

### Exercise 1: Basic Nav2 configuration for a simulated robot
1.  **Setup Isaac Sim/Gazebo**: Launch a simulated wheeled robot (e.g., TurtleBot3 or a simple differential drive robot) in Isaac Sim or Gazebo. Ensure its odometry and LiDAR data are published to ROS 2 topics (`/odom`, `/scan`).
2.  **Create a Static Map**: If you don't have one, manually create a simple `map.pgm` (grayscale image) and `map.yaml` file describing a small, enclosed environment. Place it in a `maps` directory in your ROS 2 workspace.
3.  **Configure Nav2**: Create a `nav2_params.yaml` file for your robot, setting up basic parameters for AMCL, global planner (NavFn or SmacPlanner), and local controller (DWAPlanner or TEBPlanner).
4.  **Launch Nav2**: Use a launch file (similar to the example above, but adapted for your robot and map) to start the Nav2 stack.
5.  **Send a Goal**: In RViz, use the "2D Nav Goal" tool to send a navigation goal to your robot. Observe if the robot plans a path and attempts to navigate to the goal.

### Exercise 2: Exploring Bipedal Balance Concepts
1.  **Research**: Watch videos of advanced humanoid robots (e.g., Boston Dynamics Atlas) performing complex locomotion. Pay attention to how they maintain balance.
2.  **Simulate a Simple Biped (Conceptual)**: If possible, find a simple bipedal robot model in Isaac Sim or Gazebo. Try to manually control its joint angles (e.g., through a GUI or simple ROS 2 commands) to make it take a single stable step.
3.  **Analyze ZMP/CoM**: Using visualization tools in the simulator, try to track the CoM and ZMP during the robot's movement. How does it relate to the robot's stability?

## Assignments

1.  **Humanoid Navigation Strategy**: Propose a high-level navigation strategy for a humanoid robot in a dynamic indoor environment (e.g., an office building). Consider:
    *   How would the robot localize itself?
    *   What kind of map would it use (static, dynamic, semantic)?
    *   How would it plan paths, considering its bipedal nature and need for balance?
    *   How would it avoid both static and moving obstacles?
    *   Integrate concepts from Isaac ROS perception (Chapter 10) to enhance navigation. Provide a detailed explanation and a `mermaid` diagram of the proposed system.

2.  **Balance Control Algorithm Concept**: Describe the conceptual design of a basic balance control algorithm for a humanoid robot during walking. Focus on:
    *   Key sensor inputs (e.g., IMU, force sensors in feet).
    *   The role of CoM and ZMP in maintaining stability.
    *   How joint torques or positions would be adjusted in real-time to prevent falling. You don't need to provide full code, but outline the logic and feedback loops involved.

## Summary
Chapter 11 provided a comprehensive overview of the ROS 2 Navigation Stack (Nav2), detailing its architectural components and workflow for autonomous robot navigation in simulated environments. We then delved into the complex world of bipedal locomotion, introducing key concepts like the Center of Mass (CoM), Support Polygon, and Zero Moment Point (ZMP), which are fundamental to maintaining balance in humanoid robots. The chapter also highlighted the significant challenges in humanoid navigation and explored how advanced perception from Isaac ROS can enhance Nav2's capabilities. Through conceptual examples and exercises, we gained insight into configuring Nav2 and the intricate mechanisms required for bipedal stability, setting the stage for truly autonomous humanoid robots.
