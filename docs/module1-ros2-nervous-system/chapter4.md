# Chapter 4: ROS 2 Nodes, Topics, Services, Actions

## Learning Goals
- Deepen understanding of ROS 2 nodes as fundamental computational units.
- Master the use of ROS 2 topics for asynchronous data streaming.
- Implement and utilize ROS 2 services for synchronous request-response interactions.
- Understand the concept and implementation of ROS 2 actions for long-running, pre-emptable tasks.
- Learn to create and manage ROS 2 parameters.

## Prerequisites
Familiarity with ROS 2 core concepts from Chapter 3, Python programming, and command-line operations.

## Key Concepts

### ROS 2 Nodes: The Building Blocks
In ROS 2, a **node** is an executable process that performs a specific task. Each node is designed to be modular and reusable, embodying the "do one thing well" philosophy. For example, one node might be responsible for reading data from a camera, another for controlling motors, and yet another for performing navigation computations. Nodes communicate with each other using the ROS 2 communication primitives.

### ROS 2 Topics: Asynchronous Data Streaming
**Topics** are the primary means for nodes to exchange asynchronous, one-way messages. A node that sends messages on a topic is called a **publisher**, and a node that receives messages is called a **subscriber**.
-   **Message Types**: Each topic has a defined message type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`), which dictates the structure of the data being transmitted.
-   **Decoupling**: Publishers and subscribers are decoupled; they don't need to know about each other's existence. The ROS 2 middleware handles the routing of messages.
-   **Use Cases**: Continuous data streams like sensor readings (e.g., LiDAR scans, IMU data), robot odometry, or video feeds.

### ROS 2 Services: Synchronous Request-Response
**Services** provide a synchronous request-reply mechanism. A node acting as a **client** sends a request to a **server** node and waits for a response. The server node processes the request and sends back a single response.
-   **Service Types**: Similar to topics, services have defined service types (e.g., `std_srvs/srv/SetBool`), which specify the structure of both the request and the response.
-   **Blocking Call**: The client typically blocks until it receives a response or a timeout occurs.
-   **Use Cases**: Instantaneous, single-shot operations such as setting a robot's state, triggering an action, or querying specific information (e.g., changing a robot's speed, getting the current battery level).

### ROS 2 Actions: Goal-Oriented, Pre-emptable Tasks
**Actions** are designed for long-running, goal-oriented tasks that may require intermediate feedback and can be pre-empted (cancelled). They build upon topics and services to provide a more sophisticated communication pattern.
-   **Action Structure**: An action consists of a **goal** (what to achieve), **feedback** (progress updates), and a **result** (final outcome).
-   **Components**: An action client sends a goal to an action server. The action server executes the task, periodically publishes feedback, and eventually sends a result. The client can also send a request to cancel the goal.
-   **Use Cases**: Navigation tasks (e.g., "go to point A"), complex manipulation sequences (e.g., "pick up object B"), or any task that takes time and benefits from progress updates and the ability to be stopped.

### ROS 2 Parameters
**Parameters** in ROS 2 allow nodes to expose configurable values. These can be used to dynamically adjust a node's behavior without recompiling the code. Parameters can be set at node launch time (via launch files) or dynamically modified during runtime using command-line tools or other nodes.
-   **Parameter Server**: Conceptually, there's a "parameter server" where parameters are stored and can be accessed by nodes. ROS 2 provides standard interfaces for parameter management.
-   **Use Cases**: Configuring sensor thresholds, motor PID gains, navigation constants, or any setting that might need to be tweaked for different environments or robot configurations.

## Diagrams
-   **Diagram 1: ROS 2 communication patterns (nodes, topics, services, actions)**
    *   *Description*: A diagram showing how multiple ROS 2 nodes interact. Illustrate one-way topic communication from a publisher to multiple subscribers. Show a client-server exchange for a service. Detail the action communication flow, including the goal, continuous feedback, and final result between an action client and an action server.

## Examples

### Python `rclpy` Node Example
Here's a basic Python example of a ROS 2 node that publishes a string message to a topic.

```python
# my_publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this:
1.  Save the code as `my_publisher_node.py` in a ROS 2 package (e.g., `src/my_robot_package/my_robot_package/my_publisher_node.py`).
2.  Build your workspace (`colcon build`).
3.  Source your `install/setup.bash` (or `setup.ps1` for PowerShell).
4.  Run the node: `ros2 run my_robot_package my_publisher_node`
5.  In another terminal, echo the topic: `ros2 topic echo /my_topic`

## Hands-on Exercises

### Exercise 1: Implementing basic ROS 2 communication
1.  **Create a ROS 2 Package**: Using PowerShell, create a new ROS 2 Python package (e.g., `my_ros2_examples`) in your workspace's `src` directory: `ros2 pkg create --build-type ament_python my_ros2_examples`
2.  **Implement a Publisher Node**: Adapt the `MyPublisher` example above. Create a Python script `simple_publisher.py` in your new package that publishes a custom message (e.g., an integer count) to a topic every second. Ensure it logs messages correctly.
3.  **Implement a Subscriber Node**: Create another Python script `simple_subscriber.py` in the same package that subscribes to the topic from your publisher node. It should receive and print the messages.
4.  **Build and Run**: Build your ROS 2 workspace (`colcon build`) and then run both your publisher and subscriber nodes in separate PowerShell terminals. Verify that messages are being sent and received correctly.

## Assignments
1.  **Service Implementation**: Extend your `my_ros2_examples` package. Design and implement a ROS 2 service using `rclpy` that takes two integers as a request and returns their sum as a response. Create a simple client node that calls this service with example values and prints the result. Provide the Python code for both the service server and client.
2.  **Action Concept**: Imagine a robot arm needs to perform a complex sequence: pick up object A, move it to point B, and place it down. Describe how you would design this as a ROS 2 action. Outline the goal, feedback, and result structure. Explain why an action is more suitable than a topic or service for this task.

## Summary
Chapter 4 provided a detailed exploration of ROS 2's fundamental communication primitives: nodes, topics, services, and actions. We learned how nodes serve as modular computational units, topics enable asynchronous data streaming, services facilitate synchronous request-response interactions, and actions manage long-running, pre-emptable tasks. The chapter also touched upon ROS 2 parameters for dynamic configuration and provided practical Python examples and hands-on exercises to solidify understanding of implementing these communication patterns, which are crucial for building sophisticated and reactive robotic applications.