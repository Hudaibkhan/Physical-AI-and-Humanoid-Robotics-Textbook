# Chapter 5: URDF & Humanoid Robot Structure

## Learning Goals
- Understand the purpose and structure of URDF (Unified Robot Description Format).
- Learn to define robot kinematics (links and joints) using URDF.
- Comprehend how to add visual and collision properties to URDF models.
- Gain insight into the typical anatomical structure and kinematic chains of humanoid robots.
- Explore the challenges and considerations in modeling humanoid robots for simulation and real-world deployment.

## Prerequisites
Basic understanding of ROS 2 concepts and XML syntax is helpful. Familiarity with 3D geometry concepts (e.g., transformations, rotations) is beneficial.

## Key Concepts

### Unified Robot Description Format (URDF)
URDF (Unified Robot Description Format) is an XML format used in ROS to describe all aspects of a robot. It specifies the robot's kinematic and dynamic properties, visual appearance, and collision models. A URDF file is essentially a blueprint that ROS 2 tools and simulators (like Gazebo or RViz) use to understand and interact with the robot.

### Links and Joints: The Kinematic Chain
The core of a URDF model consists of two main elements:
-   **Links**: Represent the rigid bodies of the robot (e.g., torso, upper arm, forearm, hand, thigh, shin, foot). Each link has an inertial properties, visual properties, and collision properties.
-   **Joints**: Connect two links and define their relative motion. Joints specify the type of motion allowed (e.g., revolute for rotation, prismatic for translation, fixed for no motion) and their limits (e.g., min/max angle for revolute joints).

A series of interconnected links and joints forms the robot's **kinematic chain**, which describes how the robot's parts move relative to each other.

### Visual and Collision Models
-   **Visual Model**: Describes how the robot looks. This typically involves referencing 3D mesh files (e.g., `.stl`, `.dae`) and defining their color or texture. The visual model is used for rendering the robot in simulators and visualization tools.
-   **Collision Model**: Describes the robot's physical shape for collision detection. This is crucial for simulations to prevent the robot from self-colliding or passing through environmental obstacles. Often, simplified geometries (spheres, boxes, cylinders) are used for collision models to reduce computational complexity, even if the visual model is highly detailed.

### Humanoid Robot Structure
Humanoid robots are designed to mimic the human body. Their structure typically includes:
-   **Torso**: The main body, often containing core electronics and power.
-   **Head**: Houses cameras, microphones, and sometimes displays.
-   **Arms**: Multi-jointed limbs for manipulation, usually ending in hands or grippers.
-   **Legs**: Multi-jointed limbs for locomotion, enabling bipedal walking.
-   **Kinematic Chains**: Humanoids have complex kinematic chains to achieve human-like dexterity and mobility. This involves many degrees of freedom (DOF) and intricate joint configurations.

### Challenges in Humanoid Modeling
-   **High Degrees of Freedom**: Humanoids have many joints, making kinematic and dynamic control complex.
-   **Balance and Stability**: Maintaining balance during walking or manipulation is a significant challenge, often requiring sophisticated control algorithms and sensory feedback (e.g., from IMUs).
-   **Self-Collision Avoidance**: With many articulating parts, preventing the robot from colliding with itself is critical and requires careful collision model design.
-   **Realistic Simulation**: Accurately simulating humanoid behavior (especially walking) requires precise physical models and robust simulation environments.

## Diagrams
-   **Diagram 1: Basic URDF structure**
    *   *Description*: An XML snippet showing the root `<robot>` tag, a `<link>` definition with inertial, visual, and collision properties, and a `<joint>` definition connecting two links with a specific type (e.g., `revolute`). Highlight the relationships between elements.
-   **Diagram 2: Humanoid robot kinematic chain**
    *   *Description*: A simplified stick-figure diagram of a humanoid robot, illustrating the major links (torso, head, upper arm, forearm, hand, thigh, shin, foot) and the joints connecting them (e.g., neck, shoulder, elbow, hip, knee, ankle). Label key degrees of freedom.

## Examples

### Simple URDF Example (Robot Arm Segment)
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>
</robot>
```
This URDF describes a `base_link` (a box) connected to `link1` (a cylinder) by a `revolute` joint, allowing rotation around the Z-axis. This forms a single segment of a robotic arm.

## Hands-on Exercises

### Exercise 1: Creating a simple URDF model
1.  **Setup**: Ensure you have a ROS 2 workspace. Create a new package (e.g., `my_urdf_tutorial`) and a `urdf` subdirectory within it.
2.  **Draft URDF**: Create a file named `simple_robot.urdf` in the `urdf` directory. Start with the `simple_arm` example above, but modify it to represent a two-link robot arm with two revolute joints.
3.  **Visualize in RViz**: Install `ros-humble-joint-state-publisher-gui` and `ros-humble-rviz2`.
    -   Open PowerShell and navigate to your workspace.
    -   Source ROS 2 setup: `.\install\setup.ps1`
    -   Launch `joint_state_publisher_gui` with your URDF: `ros2 launch urdf_tutorial display.launch.py model:=src/my_urdf_tutorial/urdf/simple_robot.urdf`
    -   Use the GUI sliders to manipulate the joints and observe your robot in RViz.

## Assignments
1.  **Humanoid Leg Segment URDF**: Design a URDF snippet for a single leg of a simplified humanoid robot. It should include:
    -   A `hip_link` (box)
    -   A `thigh_link` (cylinder) connected by a `hip_joint` (revolute, allowing pitch/roll)
    -   A `shin_link` (cylinder) connected to the `thigh_link` by a `knee_joint` (revolute, allowing pitch)
    -   A `foot_link` (box) connected to the `shin_link` by an `ankle_joint` (revolute, allowing pitch)
    Specify appropriate `origin` and `axis` for each joint. You do not need to add inertial, visual, or collision properties for this assignment.
2.  **URDF vs. SDF Comparison**: Research and briefly explain the key differences between URDF (Unified Robot Description Format) and SDF (Simulation Description Format). When would you choose one over the other for a robotics project, especially in the context of ROS 2 and Gazebo?

## Summary
Chapter 5 introduced URDF as the standard XML format for describing robot kinematics, dynamics, and visual/collision properties within ROS 2. We explored the fundamental components of links and joints, which form the robot's kinematic chain, and understood the distinction between visual and collision models. The chapter also delved into the typical structural elements of humanoid robots and the inherent challenges in modeling such complex systems. Through practical examples and exercises, we gained the foundational knowledge required to create and visualize simple URDF models, laying the groundwork for more advanced robot simulations and development.