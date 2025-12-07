# Weekly Roadmap

This page outlines the weekly roadmap for the Physical AI & Humanoid Robotics Textbook, providing a structured learning path through all modules and key concepts.

![Learning Path Overview](/img/learning-path-overview.png)

*Figure 1: Overview of the 7-week learning path through the Physical AI & Humanoid Robotics curriculum.*

## Week 1: ROS 2 Basics

### Learning Objectives
- Understand the fundamental concepts of ROS 2 and its architecture
- Install and configure ROS 2 Humble Hawksbill
- Learn about nodes, packages, and the ROS 2 ecosystem
- Set up your development environment for robotics programming

### Topics Covered
- Introduction to ROS 2 concepts and architecture
- Installing ROS 2 on your system
- Understanding the difference between ROS 1 and ROS 2
- Setting up your first ROS 2 workspace
- Basic ROS 2 commands and tools

### Activities
- Install ROS 2 Humble Hawksbill on your development machine
- Create your first ROS 2 workspace and package
- Run basic ROS 2 commands (ros2 run, ros2 topic, ros2 service)
- Explore the ROS 2 documentation and tutorials

### Diagram
![ROS 2 Architecture](/img/ros2-architecture.png)

*Figure 2: ROS 2 architecture showing nodes, topics, services, and the DDS communication layer.*

### Resources
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- ROS 2 tutorials for beginners
- Package management with colcon

### Estimated Time
8-10 hours of study and hands-on practice

## Week 2: ROS 2 Nodes, Topics, URDF

### Learning Objectives
- Create and run ROS 2 nodes in Python and C++
- Implement communication between nodes using topics
- Define robot structure using URDF (Unified Robot Description Format)
- Understand message passing and service architecture

### Topics Covered
- Creating ROS 2 nodes in Python and C++
- Publisher and subscriber patterns
- Topics vs services vs actions
- URDF for robot description
- TF (Transform) frames and robot state publisher

### Activities
- Create a simple publisher and subscriber node
- Implement a service client and server
- Build a URDF model of a simple robot
- Visualize your robot in RViz

### Diagram
![ROS 2 Node Communication](/img/ros2-node-communication.png)

*Figure 3: ROS 2 node communication patterns showing publishers, subscribers, and message passing.*

### Resources
- ROS 2 node creation tutorials
- URDF tutorials and examples
- TF and robot state publisher documentation

### Estimated Time
10-12 hours of study and hands-on practice

## Week 3: Gazebo/Simulation

### Learning Objectives
- Set up and use Gazebo for physics simulation
- Create custom simulation environments
- Implement sensor simulation for cameras, LiDAR, and IMU
- Integrate simulation with ROS 2 for seamless development

### Topics Covered
- Gazebo simulation environment setup
- Physics properties and world building
- Sensor simulation and plugins
- ROS 2 integration with Gazebo
- Creating custom models and environments

### Activities
- Launch your first Gazebo simulation with ROS 2
- Create a custom world for robot testing
- Add sensors to your robot model in simulation
- Implement sensor data processing in ROS 2 nodes

### Diagram
![Gazebo Simulation Environment](/img/gazebo-simulation.png)

*Figure 4: Gazebo simulation environment with physics properties and sensor integration.*

### Resources
- Gazebo Garden documentation
- ros_gz_bridge tutorials
- Sensor plugin documentation

### Estimated Time
10-12 hours of study and hands-on practice

## Week 4: Digital Twin

### Learning Objectives
- Understand digital twin concepts in robotics
- Use Unity for high-fidelity visualization
- Create accurate virtual representations of physical robots
- Implement bidirectional communication between real and virtual systems

### Topics Covered
- Digital twin architecture and benefits
- Unity integration with ROS 2
- High-fidelity visualization techniques
- Bidirectional data flow between physical and virtual systems
- Synthetic data generation

### Activities
- Set up Unity with ROS-TCP-Endpoint
- Create a digital twin of your robot in Unity
- Implement real-time synchronization between simulation and Unity
- Generate synthetic training data using your digital twin

### Diagram
![Digital Twin Architecture](/img/digital-twin-architecture.png)

*Figure 5: Digital twin architecture showing the relationship between physical robots, digital models, and simulation environments.*

### Resources
- Unity Robotics documentation
- ROS-TCP-Endpoint tutorials
- Digital twin best practices

### Estimated Time
10-12 hours of study and hands-on practice

## Week 5: Isaac AI Brain

### Learning Objectives
- Use NVIDIA Isaac Sim for advanced robotics simulation
- Implement Isaac ROS perception pipelines
- Apply Visual Simultaneous Localization and Mapping (VSLAM)
- Leverage GPU acceleration for AI workloads

### Topics Covered
- NVIDIA Isaac Sim setup and architecture
- Isaac ROS perception stack
- VSLAM techniques and implementation
- GPU acceleration for robotics AI
- Navigation solutions using Nav2

### Activities
- Install and configure NVIDIA Isaac Sim
- Implement perception pipelines using Isaac ROS
- Run VSLAM algorithms in Isaac Sim
- Configure Nav2 for robot navigation

### Diagram
![Isaac AI Architecture](/img/isaac-ai-architecture.png)

*Figure 6: NVIDIA Isaac architecture showing simulation, perception, and navigation components.*

### Resources
- NVIDIA Isaac Sim documentation
- Isaac ROS tutorials
- Nav2 navigation stack documentation

### Estimated Time
12-15 hours of study and hands-on practice

## Week 6: VLA Robotics

### Learning Objectives
- Implement Vision-Language-Action (VLA) system architectures
- Integrate Large Language Models (LLMs) with robotic systems
- Process voice commands using speech recognition tools
- Translate natural language commands into robotic actions

### Topics Covered
- Vision-Language-Action systems architecture
- Integration of LLMs with robotics
- Speech recognition and processing (Whisper)
- Natural language to action mapping
- Multimodal interfaces for robotics

### Activities
- Set up a VLA system with vision and language components
- Integrate an LLM with your robot's control system
- Implement voice command processing
- Create a language-driven robot control interface

### Diagram
![VLA System Architecture](/img/vla-system-architecture.png)

*Figure 7: Vision-Language-Action system architecture showing the flow from vision input, language understanding, to action execution.*

### Resources
- Hugging Face Transformers documentation
- OpenAI API documentation (or open-source LLM alternatives)
- Speech recognition libraries documentation

### Estimated Time
12-15 hours of study and hands-on practice

## Week 7: Capstone Project

### Learning Objectives
- Integrate concepts from all previous modules
- Implement a complete physical AI system
- Demonstrate proficiency in ROS 2, simulation, AI, and VLA systems
- Present and document your capstone project

### Topics Covered
- Integration of all textbook concepts
- Project planning and execution
- Testing and validation of complex systems
- Documentation and presentation of results

### Activities
- Design and implement your capstone project
- Integrate simulation, AI, and control components
- Test your system in both simulated and (if available) real environments
- Document and present your project results

### Diagram
![Capstone Project Integration](/img/capstone-integration.png)

*Figure 8: Capstone project showing integration of all modules: ROS 2, Simulation, AI, and VLA systems.*

### Resources
- All resources from previous weeks
- Project planning and documentation templates
- Best practices for robotics system integration

### Estimated Time
15-20 hours of study and hands-on practice

## Additional Notes

- Each week builds upon the previous week's knowledge and skills
- Hands-on practice is essential for mastering robotics concepts
- Students are encouraged to experiment and explore beyond the required activities
- Additional resources and advanced topics are available in the Additional Materials section