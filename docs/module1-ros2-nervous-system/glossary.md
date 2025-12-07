# Module 1 Glossary

This glossary provides concise definitions for key terms used throughout Module 1: The Robotic Nervous System (ROS 2).

- **ROS 2 (Robot Operating System 2)**: An open-source middleware suite for robot application development. It provides services for hardware abstraction, device drivers, inter-process communication, and more.

- **Node**: A process in ROS 2 that performs computation (e.g., a sensor driver, a controller, an algorithm). Nodes are modular and can communicate with each other.

- **Topic**: A named bus over which nodes exchange messages. Topics use a publish/subscribe model for asynchronous, many-to-many communication.

- **Service**: A synchronous communication mechanism in ROS 2, similar to a remote procedure call (RPC). A client sends a request, and a server processes it and returns a response.

- **Action**: A long-running, asynchronous communication mechanism in ROS 2 used for goal-oriented tasks (e.g., moving a robot arm to a target). It provides feedback and allows preemption.

- **rclpy**: The Python client library for ROS 2, enabling developers to write ROS 2 nodes and interact with the ROS 2 ecosystem using Python.

- **URDF (Unified Robot Description Format)**: An XML file format used in ROS to describe all elements of a robot. It specifies the robot's kinematics (links and joints), visuals, and collision properties.

- **Link**: A rigid body element in a URDF model (e.g., a robot's forearm, hand, or base). Links have mass, inertia, and visual properties.

- **Joint**: A connection between two links in a URDF model, defining their relative motion. Joints can be revolute, prismatic, fixed, etc.

- **colcon**: A command-line tool used to build, test, and install ROS 2 packages within a workspace.

- **Workspace**: A directory structure in ROS 2 that contains source code for packages, build artifacts, and installation directories.

- **Package**: The primary unit of organization in ROS 2, containing nodes, libraries, configuration files, and other resources related to a specific piece of functionality.
