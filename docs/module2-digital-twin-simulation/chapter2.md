# Chapter 7: Physics, Sensors & World Building

## Learning Goals
- Understand fundamental physics concepts relevant to robotics simulation.
- Learn about common types of sensors used in robotics and their simulation.
- Comprehend how to configure physics properties for links and joints in Gazebo.
- Explore techniques for creating and populating dynamic simulation worlds.
- Grasp the process of integrating sensors into a robot model and accessing their data.

## Prerequisites
Familiarity with Gazebo simulation fundamentals (Chapter 6) and basic understanding of robot kinematics.

## Key Concepts

### Robot Physics in Simulation
Accurate physics simulation is crucial for realistic robot behavior. Gazebo's physics engine calculates interactions based on:
-   **Gravity**: Applied to all links with mass. Can be configured per world.
-   **Mass and Inertia**: Defined for each link. Determines how a link responds to forces and torques.
-   **Friction**: Controls resistive forces between colliding surfaces. Can be static or dynamic.
-   **Restitution (Elasticity)**: Determines how much energy is conserved during a collision (bounciness).
-   **Damping**: Reduces oscillations and brings objects to rest over time.

### Common Sensors in Robotics
Simulators like Gazebo provide models for various sensors, allowing realistic data generation:
-   **Cameras**: Simulate visual input (RGB, depth, infrared). Output image data as ROS 2 image messages.
-   **LiDAR (Light Detection and Ranging)**: Simulate laser range finders, providing distance measurements to objects. Used for mapping and navigation.
-   **IMU (Inertial Measurement Unit)**: Provides orientation, angular velocity, and linear acceleration data. Crucial for robot pose estimation and balance control.
-   **Contact Sensors**: Detect physical contact between objects or robot parts. Useful for collision detection and tactile feedback.
-   **Joint State Sensors**: Report the position, velocity, and effort of robot joints. Essential for robot control.

### Configuring Physics Properties
In SDF, physics properties are defined within the `<inertial>`, `<collision>`, and `<surface>` tags of a link:
-   `<inertial>`: Contains `<mass>` and `<inertia>` elements.
-   `<collision>`: Can have `<surface>` properties like `<friction>` (e.g., `ode.mu` for dynamic friction, `ode.mu2` for static friction) and `<bounce>` (restitution).

For joints, properties like `<damping>` and `<friction>` can be set to model mechanical resistance or losses.

### World Building Techniques
Creating complex simulation worlds involves:
-   **Adding Static Models**: Importing pre-made models of buildings, furniture, trees, etc., from the Gazebo Model Database or custom sources.
-   **Creating Dynamic Objects**: Defining objects that can be moved or manipulated by the robot (e.g., blocks, balls).
-   **Environmental Factors**: Configuring lighting (e.g., `<light>` tag, including sun or spotlights), fog, and wind to mimic real-world conditions.
-   **Terrain**: Using heightmaps or meshes to create uneven ground or landscapes.

### Integrating Sensors into Robot Models
Sensors are typically defined within a robot model's URDF or SDF. In SDF:
-   `<sensor>` tag: Placed inside a `<link>` element.
-   `type`: Specifies the sensor type (e.g., `camera`, `gpu_lidar`, `imu`).
-   `<always_on>`, `<update_rate>`: Control sensor activity and data frequency.
-   `camera`, `lidar`, `imu` specific properties: Configure resolution, field of view, noise models, etc.
-   `<plugin>`: Often used to connect Gazebo sensors to ROS 2, publishing sensor data to ROS 2 topics.

## Diagrams
-   **Diagram 1: Robot physics in simulation**
    *   *Description*: A diagram illustrating how forces (gravity, friction, contact), mass, and inertia interact within a robot model in a physics simulation. Show arrows representing forces and labels for physical properties affecting robot movement.
-   **Diagram 2: Common sensors in robotics**
    *   *Description*: An infographic showing various robot sensors (Camera, LiDAR, IMU, Tactile, Sonar) with a small icon for each and a brief description of what they measure and their typical applications.

## Examples

### Gazebo World with Custom Physics and a Sensor
Consider a simple world with a custom friction ground plane and a model with an IMU sensor.

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_sensor_world">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -30 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>   <!-- high friction -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.8 0.8 0.8 1</specular></material>
        </visual>
      </link>
    </model>

    <model name="imu_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>0.166</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.166</iyy><iyz>0</iyz><izz>0.166</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material><ambient>0.0 1.0 0.0 1</ambient><diffuse>0.0 1.0 0.0 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <sensor name="imu_sensor" type="imu">
          <always_on>1</always_on>
          <update_rate>100</update_rate>
          <imu>
            <angular_velocity><x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>...</angular_velocity>
            <!-- ... other IMU noise properties ... -->
          </imu>
          <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
            <ros>        <namespace>/imu_box</namespace>        <publish_tf>0</publish_tf>      </ros>
            <topicName>imu</topicName>
            <frameName>box_link</frameName>
          </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
```
This world sets a high-friction ground plane and includes `imu_box`, a green cube equipped with an IMU sensor that publishes data to a ROS 2 topic via `libgazebo_ros_imu_sensor.so` plugin.

## Hands-on Exercises

### Exercise 1: Configuring physics properties in Gazebo
1.  **Create a World**: Start with `empty.world` (from Chapter 6) or create a new one. Add a simple box model with default physics properties.
2.  **Observe Behavior**: Launch Gazebo with this world. Note how the box falls and rests on the ground. Try applying a force to it (in Gazebo GUI) and observe its movement.
3.  **Modify Friction**: Edit your world file. Increase the `<mu>` and `<mu2>` (friction coefficients) for the `ground_plane` or the `box_link`'s collision surface to a higher value (e.g., `10.0`). Relaunch Gazebo. How does the box's sliding behavior change?
4.  **Add Restitution**: Introduce `<bounce>` (restitution) properties to the box's collision surface (e.g., `<restitution_coefficient>0.8</restitution_coefficient>`). Relaunch and observe if the box now bounces after impact.
5.  **Integrate an IMU Sensor**: Add the IMU sensor definition from the `imu_box` example above into your box model's link. Ensure you have the `libgazebo_ros_imu_sensor.so` plugin configured correctly.
6.  **Verify Sensor Data**: In a new PowerShell terminal, after launching Gazebo, source your ROS 2 environment and check for the IMU topic: `ros2 topic list`. Then `ros2 topic echo /imu_box/imu` to see the sensor data streaming.

## Assignments
1.  **Dynamic World with Obstacles**: Design a Gazebo `.world` file (SDF) that simulates a simple test track for a wheeled robot. Include:
    -   An uneven terrain (e.g., using a heightmap or stacked boxes as ramps).
    -   At least three static obstacles of different shapes and materials (e.g., a high-friction patch, a low-friction icy patch, a small wall).
    -   Configure the physics properties (friction, restitution) for these obstacles to challenge a robot's navigation. Provide the complete SDF file.
2.  **LiDAR Sensor Integration**: Taking the `imu_box` example as a template, describe how you would modify a URDF/SDF model to include a 2D LiDAR sensor. What key parameters would you configure (e.g., horizontal range, resolution, update rate)? What Gazebo plugin would you typically use to interface this LiDAR with ROS 2, and what ROS 2 topic would it likely publish to?

## Summary
Chapter 7 delved into the intricacies of physics, sensors, and world building within robotics simulation. We gained an understanding of critical physics properties like mass, friction, and restitution, and learned how to configure them in Gazebo. The chapter introduced common robot sensors such as cameras, LiDAR, and IMUs, explaining their simulated counterparts and integration. Through hands-on exercises, we practiced modifying physics settings and verifying sensor data, laying essential groundwork for creating realistic and challenging simulation environments for physical AI development.