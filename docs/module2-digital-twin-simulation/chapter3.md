# Chapter 8: Unity Visualization for Robotics

## Learning Goals
- Understand the benefits of using Unity for robotics visualization and simulation.
- Learn to set up a Unity project for ROS 2 integration using existing packages (e.g., `Unity-ROS-TCP-Connector`, `ROS-Unity-Integration`).
- Comprehend how to import and visualize URDF robot models within Unity.
- Explore techniques for streaming ROS 2 data (e.g., sensor feedback, robot poses) to Unity for real-time visualization.
- Gain insight into controlling Unity-simulated robots from ROS 2.

## Prerequisites
Familiarity with Unity Editor basics, ROS 2 core concepts, and URDF robot descriptions. Basic C# scripting knowledge is beneficial.

## Key Concepts

### Why Unity for Robotics Visualization?
While Gazebo excels in physics-accurate simulation, Unity offers unparalleled capabilities for high-fidelity 3D rendering, custom user interfaces, and advanced visual effects.
-   **High-Fidelity Graphics**: Create visually stunning and realistic robot environments.
-   **Customizable UIs**: Develop intuitive dashboards and control panels within the simulation.
-   **Cross-Platform Deployment**: Easily deploy visualizations to various platforms (desktop, web, VR/AR).
-   **Extensive Asset Store**: Access a vast library of 3D models, textures, and tools.
-   **Game Engine Features**: Leverage Unity's built-in tools for animation, camera control, and scene management.

### Unity-ROS 2 Integration Architecture
Integrating Unity with ROS 2 typically involves a client-server architecture.
-   **ROS 2 Side**: Publishers send robot state, sensor data, or other information. Subscribers receive commands or status updates.
-   **Unity Side**: A C# application runs within the Unity Editor or as a standalone build. It uses a network connector (e.g., `Unity-ROS-TCP-Connector`) to establish communication with the ROS 2 system. This connector often acts as a bridge, translating ROS 2 messages into Unity-compatible data structures and vice-versa.
-   **ROS-Unity-Integration**: This package from Unity Technologies provides a robust framework for managing ROS 2 communication within Unity, including tools for importing URDF, publishing and subscribing to topics, and calling services.

### Importing URDF Robot Models into Unity
The `ROS-Unity-Integration` package often includes tools or workflows for importing URDF files directly into Unity.
-   **URDF Importer**: Converts the URDF's kinematic structure (links and joints), visual meshes, and collision geometries into Unity GameObjects and components.
-   **Coordinate Systems**: Careful attention is required to ensure consistent coordinate system transformations between ROS 2 (Z-up, right-handed) and Unity (Y-up, left-handed).
-   **Physics Components**: Unity's physics engine can be used for basic interaction, though it's often more about visualization than high-accuracy simulation when integrating with ROS 2.

### Real-time Data Streaming and Visualization
Once integrated, Unity can subscribe to ROS 2 topics to receive and visualize various data:
-   **Robot Pose**: Update the position and orientation of robot parts (GameObjects) in Unity based on ROS 2 `tf` or `odom` messages.
-   **Sensor Data**:
    -   **Camera Images**: Display ROS 2 `sensor_msgs/Image` streams on Unity textures.
    -   **LiDAR Scans**: Render point clouds or laser beams based on `sensor_msgs/LaserScan` data.
    -   **IMU Data**: Visualize robot orientation changes or apply forces/torques in Unity based on `sensor_msgs/Imu` data.
-   **Joint States**: Animate robot joints in Unity based on `sensor_msgs/JointState` messages.

### Controlling Unity-Simulated Robots from ROS 2
Unity can also act as a simulated environment where the robot's actuators are controlled by ROS 2 commands.
-   **Actuator Commands**: Unity subscribes to ROS 2 topics (e.g., `cmd_vel` for mobile robots, joint position/velocity commands for manipulators).
-   **Unity Physics/Animation**: C# scripts within Unity translate these commands into forces, velocities, or target positions for the robot's GameObjects.
-   **Feedback**: Unity can then publish simulated sensor data back to ROS 2, closing the control loop.

## Diagrams
- Diagram 1: Unity-ROS 2 integration architecture
    *   *Description*: A block diagram illustrating the communication flow between a ROS 2 system (nodes, topics) and a Unity application (C# scripts, network connector, imported robot model). Show topics like `/robot/joint_states` and `/camera/image_raw` flowing from ROS 2 to Unity, and `/cmd_vel` from Unity to ROS 2 for control.

## Examples

### Basic Unity Scene with a ROS 2 Connected Robot
This is a conceptual example, as a full Unity project setup is extensive.

1.  **Unity Project Setup**:
    *   Create a new 3D project in Unity Hub.
    *   Install `ROS-Unity-Integration` package via Unity's Package Manager (using Git URL or `file:` path).
    *   Ensure .NET compatibility (Unity's `Scripting Backend` set to `Mono` or `IL2CPP` with `NET Framework` as `API Compatibility Level` if `ROS-TCP-Connector` requires it).

2.  **Import URDF**:
    *   Use the `ROS-Unity-Integration` menu (e.g., `ROS -> Import URDF`) to bring in a robot's URDF file. This generates a hierarchy of GameObjects representing the robot.

3.  **ROS 2 Publisher (Python)**:
    *   A simple ROS 2 node publishing joint states (similar to Chapter 4's publisher).

    ```python
    # ros2_joint_publisher.py
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import time

    class JointPublisher(Node):
        def __init__(self):
            super().__init__('joint_publisher')
            self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.angle = 0.0

        def timer_callback(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1'] # Replace with actual joint names from your URDF
            self.angle += 0.01
            msg.position = [self.angle % (2 * 3.14159)] # Simulate a rotating joint
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing joint state: {msg.position}')

    def main(args=None):
        rclpy.init(args=args)
        joint_publisher = JointPublisher()
        rclpy.spin(joint_publisher)
        joint_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

4.  **Unity Subscriber (C#)**:
    *   Attach a C# script to your imported robot's root GameObject in Unity. This script would subscribe to the `joint_states` topic and update the corresponding Unity joints.

    ```csharp
    // JointStateSubscriber.cs (Conceptual C# script in Unity)
    using UnityEngine;
    using RosMessageTypes.Sensor; // Assumes generated ROS 2 message types are available
    using ROSGeometry; // For coordinate system conversions
    using Unity.Robotics.ROSTCPConnector; // Assuming Unity-ROS-TCP-Connector

    public class JointStateSubscriber : MonoBehaviour
    {
        ROSConnection ros;
        public string topicName = "joint_states";
        public ArticulationBody[] joints; // Assign your robot's joints in the Inspector

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<JointStateMsg>(topicName, ReceiveJointState);
        }

        void ReceiveJointState(JointStateMsg jointState)
        {
            // This is simplified. You would map jointState.name to your 'joints' array
            // and apply positions.
            for (int i = 0; i < jointState.name.Length; i++)
            {
                string jointName = jointState.name[i];
                float position = (float)jointState.position[i];

                // Example: Find a joint by name and set its target position
                // In a real scenario, you'd use a dictionary for efficient lookup or
                // ensure the 'joints' array is ordered correctly.
                foreach (ArticulationBody joint in joints)
                {
                    if (joint.name == jointName)
                    {
                        var drive = joint.xDrive;
                        drive.target = position * Mathf.Rad2Deg; // Convert radians to degrees for Unity
                        joint.xDrive = drive;
                        break;
                    }
                }
            }
        }
    }
    ```

## Hands-on Exercises

### Exercise 1: Setting up Unity for ROS 2 Communication
1.  **Install Unity**: Download and install Unity Hub, then install a recent LTS version of Unity Editor (e.g., 2022.3 LTS).
2.  **Create New Project**: Create a new 3D (URP or HDRP) project in Unity.
3.  **Import ROS-Unity-Integration**:
    *   Open `Window -> Package Manager`.
    *   Click the `+` icon, then `Add package from git URL...`.
    *   Enter `https://github.com/Unity-Technologies/ROS-Unity-Integration.git?path=/com.unity.robotics.ros-tcp-connector` (or the latest stable URL).
    *   Import other necessary packages (e.g., `com.unity.robotics.urdf-importer`).
4.  **Verify Connection**: Follow the documentation in the imported `ROS-TCP-Connector` package to run a simple echo example, ensuring Unity can communicate with a running ROS 2 `ros_tcp_endpoint`.

### Exercise 2: Visualizing a URDF Robot in Unity
1.  **Prepare URDF**: Take one of your URDF models (e.g., from Chapter 5 assignment) and ensure it has correct mesh paths (relative to the URDF).
2.  **Import URDF into Unity**:
    *   In Unity, go to `Robotics -> URDF Importer -> Import URDF`.
    *   Select your URDF file. Configure import settings (e.g., collision generation).
    *   Observe the imported robot model in your Unity scene. Adjust materials and lighting for better visualization.
3.  **Animate Joints (Manual)**: Select a joint GameObject in Unity and try manually changing its rotation/position in the Inspector to understand how the kinematic chain moves.

## Assignments

1.  **ROS 2-Unity Joint Control**:
    *   Expand on Exercise 2. Create a ROS 2 Python publisher node that publishes `JointState` messages for *two* joints of your imported URDF robot in Unity.
    *   Modify the conceptual `JointStateSubscriber.cs` script in Unity to correctly parse these messages and animate the corresponding `ArticulationBody` joints in real-time.
    *   Demonstrate that by changing values in your Python node, the robot's joints move smoothly in Unity. Provide both the Python ROS 2 code and the C# Unity script.

2.  **Unity Sensor Visualization**:
    *   Describe how you would integrate a simulated ROS 2 camera (publishing `sensor_msgs/Image`) or a LiDAR (publishing `sensor_msgs/LaserScan`) into Unity for visualization.
    *   Outline the steps in Unity (e.g., creating a `RawImage` for camera feed, generating `LineRenderer` or point cloud for LiDAR).
    *   Explain any necessary coordinate system transformations or data conversions.

## Summary
Chapter 8 explored the powerful capabilities of Unity for high-fidelity robotics visualization and simulation, complementing the physics-focused Gazebo environment. We learned about the architectural patterns for integrating Unity with ROS 2, leveraging packages like `ROS-Unity-Integration` for seamless communication. The process of importing URDF models, streaming real-time ROS 2 data (like joint states and sensor feeds) for visualization, and even controlling Unity-simulated robots from ROS 2, were key topics. The hands-on exercises and assignments provided practical experience in setting up Unity projects, importing robot models, and establishing fundamental ROS 2 communication, equipping you with the tools to create rich, interactive digital twins for physical AI development.