---
title: "03. ROS 2 Architecture"
sidebar_label: "03. ROS 2 Architecture"
---


## Chapter 03: ROS 2 Architecture: Nodes, Topics, Services, Actions, & Beyond

### Introduction: The Nervous System of a Robot

Just as a human nervous system coordinates thoughts, movements, and sensations, the Robot Operating System (ROS 2) provides the digital nervous system for our humanoid robot. It's the essential framework that allows disparate software components—from low-level motor controllers to high-level AI planners—to communicate, cooperate, and form a coherent, intelligent agent.

In this chapter, we embark on a deep dive into the fundamental architecture of ROS 2. We will dissect its core communication mechanisms, explore how it builds upon the lessons of its predecessor (ROS 1), and understand the powerful concepts that enable complex, distributed robotic systems. By the end, you will have a comprehensive grasp of ROS 2's design philosophy and its critical building blocks, which will serve as the foundation for all subsequent chapters on simulation, control, and artificial intelligence.

### 3.1 Why ROS 2? An Evolution Driven by Real-World Demands

The original Robot Operating System (ROS 1), born in 2007, was revolutionary. It democratized robotics research and development, providing a standardized environment that abstracted away hardware complexities and fostered a vibrant open-source community. However, as robotics moved from academic labs to industrial deployment, and as the demands for real-time performance, security, and multi-robot coordination grew, ROS 1's foundational architecture revealed its limitations.

ROS 2 was engineered from the ground up to address these shortcomings, representing not merely an update, but a fundamental paradigm shift [^1]. The most significant architectural change, and the bedrock of ROS 2's power, is its adoption of the **Data Distribution Service (DDS)** as its core communication middleware.

#### DDS: The Decentralized Backbone

DDS is an international open standard (IEEE 1606, developed by the Object Management Group - OMG) designed for real-time, mission-critical, and data-intensive distributed systems [^2]. By integrating DDS, ROS 2 inherits a robust set of features crucial for modern robotics:

*   **Decentralized Discovery:** Unlike ROS 1, which relied on a central `roscore` (the "master") for node discovery and registration (a single point of failure), ROS 2 uses DDS's peer-to-peer discovery mechanisms. Nodes automatically find each other on the network without a central server, leading to more resilient, scalable, and fault-tolerant systems. This is vital for complex humanoid robots that might operate with multiple distributed processing units or in large multi-robot fleets.
*   **Quality of Service (QoS) Policies:** DDS provides granular control over communication characteristics. This is perhaps the most critical improvement for real-time and safety-critical applications. Developers can specify QoS settings for reliability (guaranteed delivery vs. best-effort), durability (whether messages persist for late-joining subscribers), history (how many messages to keep), and liveliness (detecting if a publisher is still active). This allows fine-tuning communication to meet the strict demands of, for example, a bipedal locomotion controller where missed messages could cause a fall.
*   **Security:** DDS includes robust security features, such as authentication, access control, and encryption, built into its core. This is a massive leap from ROS 1, which had no native security model. For humanoid robots interacting with humans or operating in sensitive environments, security is non-negotiable. We'll delve into SROS 2 later.
*   **Real-Time Capabilities:** DDS is explicitly designed for real-time systems. Its efficient message passing and configurable QoS policies make ROS 2 far more suitable for hard real-time control loops, essential for dynamic balance and fast manipulation.
*   **Interoperability:** As an open standard, DDS allows different DDS implementations (e.g., Fast DDS, Cyclone DDS, RTI Connext DDS) to communicate with each other. This vendor-agnostic approach increases flexibility and robustness.

<Mermaid chart={`
graph TD;
    subgraph ROS 1 (Master-Client)
        A[roscore] --> B[Node A];
        A --> C[Node B];
        B -- via roscore --> C;
    end
    subgraph ROS 2 (Peer-to-Peer)
        D[Node X] -- DDS --> E[Node Y];
        E -- DDS --> F[Node Z];
        F -- DDS --> D;
    end
    style A fill:#ffcccc,stroke:#333,stroke-width:2px
    style D fill:#ccffcc,stroke:#333,stroke-width:2px
    style E fill:#ccffcc,stroke:#333,stroke-width:2px
    style F fill:#ccffcc,stroke:#333,stroke-width:2px
`} />
*Figure 3.1: Architectural Comparison: ROS 1's Centralized Master vs. ROS 2's Decentralized DDS.*

### 3.2 The Core Concepts: Nodes – The Atomic Unit of Computation

At the heart of any ROS 2 system are **nodes** [^3]. A node is essentially an executable program—a single, self-contained computational unit designed to perform a specific task. In the context of a humanoid robot, each complex function is typically encapsulated within its own node.

**Examples of Nodes in a Humanoid System:**
*   `imu_sensor_node`: Reads data from the robot's Inertial Measurement Unit (IMU).
*   `left_leg_controller_node`: Implements the low-level PID control for the left leg's motors.
*   `head_camera_driver_node`: Publishes images and camera information from the head-mounted camera.
*   `object_detector_node`: Processes camera images to identify objects in the environment.
*   `path_planner_node`: Calculates collision-free paths for the robot's locomotion.
*   `whole_body_controller_node`: Executes the complex whole-body control algorithms for balance and movement.

This modularity is a foundational principle of ROS 2. It enables:
*   **Reusability:** Nodes can be developed and tested independently, then reused in different robotic systems.
*   **Isolation:** A crash in one node typically does not bring down the entire system.
*   **Distribution:** Nodes can run on different machines (e.g., a Jetson for perception, a more powerful workstation for planning) across a network.
*   **Simplicity:** Each node focuses on a single responsibility, making debugging and maintenance easier.

Let's illustrate with a minimal Python node using `rclpy`, the Python client library for ROS 2.

**Code Example 3.1: `minimal_node.py`**
```python
import rclpy                 # Core ROS 2 Python client library
from rclpy.node import Node  # Base class for all ROS 2 nodes

class MinimalNode(Node):
    """
    A minimal ROS 2 node that simply prints a message upon initialization.
    """
    def __init__(self):
        # Call the base Node class constructor and give our node a unique name.
        super().__init__('minimal_publisher_node') 
        self.get_logger().info('Hello from my minimal ROS 2 node! I am online.')
        # A timer could be added here to perform periodic tasks, or subscriptions
        # to react to incoming data.

def main(args=None):
    # Initialize the rclpy client library. This must be called before
    # any ROS 2 nodes can be created or used.
    rclpy.init(args=args)

    # Create an instance of our MinimalNode
    node = MinimalNode()

    # Enter a loop that processes callbacks. This keeps the node alive
    # and allows it to receive messages, handle services, etc.
    # For a node that only prints a message, this might seem unnecessary,
    # but it's crucial for any node that interacts with the ROS 2 graph.
    rclpy.spin(node)

    # Cleanly destroy the node. This releases all resources.
    node.destroy_node()
    
    # Shut down the rclpy client library.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Figure 3.2: Basic structure of a ROS 2 Python node using `rclpy`.*

To run this node (after sourcing your ROS 2 environment), you would typically use `python minimal_node.py` (if run as a standalone script) or `ros2 run <package_name> minimal_node` (once packaged).

### 3.3 Communication Patterns: The Language of Robots

Nodes don't operate in isolation. Their intelligence emerges from their ability to communicate. ROS 2 provides a rich set of communication patterns, each suited for different interaction styles.

#### 3.3.1 Topics: Asynchronous Publish/Subscribe

**Topics** are the most fundamental and widely used communication mechanism in ROS 2 [^4]. They implement an anonymous, asynchronous, many-to-many publish-subscribe model.

*   **Publisher:** A node that sends messages to a named topic. It doesn't know who is listening or even if anyone is listening.
*   **Subscriber:** A node that receives messages from a named topic. It doesn't know who is publishing.
*   **Decoupling:** Publishers and subscribers are completely decoupled in time and space. They can start and stop independently, without affecting each other.

**Key Characteristics:**
*   **Asynchronous:** Messages are sent and received independently. There's no guarantee of immediate response.
*   **One-to-Many:** A single publisher can send messages to multiple subscribers, and multiple publishers can send messages to a single topic (though this should be used carefully to avoid conflicts).
*   **Anonymous:** Publishers and subscribers don't need to know about each other's existence. DDS handles the discovery and routing.
*   **Strongly Typed:** All messages sent on a topic must conform to a predefined message type (e.g., `std_msgs/String`, `sensor_msgs/Image`, `geometry_msgs/Twist`). This ensures data integrity.

**Use Cases in Humanoid Robotics:**
*   Broadcasting continuous sensor data (e.g., camera images, LiDAR scans, IMU readings).
*   Publishing robot state (e.g., joint positions, odometry).
*   Sending continuous velocity commands to a base controller.

<Mermaid chart={`
graph TD
    P1[Node: Camera Driver] -- sensor_msgs/Image --> T1((/camera/image_raw));
    P2[Node: LiDAR Driver] -- sensor_msgs/LaserScan --> T2((/scan));
    T1 -- Image Stream --> S1[Node: Object Detector];
    T1 -- Image Stream --> S2[Node: Visual Odometry];
    T2 -- Laser Scan --> S3[Node: Navigation Stack];
    P3[Node: Whole-Body Controller] -- geometry_msgs/Twist --> T3((/cmd_vel));
    T3 -- Velocity Commands --> S4[Node: Base Controller];

    style P1 fill:#aaffaa
    style P2 fill:#aaffaa
    style P3 fill:#aaffaa
    style S1 fill:#aaaaff
    style S2 fill:#aaaaff
    style S3 fill:#aaaaff
    style S4 fill:#aaaaff
`} />
*Figure 3.3: Example of Topic Communication in a Humanoid Robot.*

**Code Example 3.2: `simple_publisher.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg # Renamed to avoid conflict with Python's built-in String
import time

class SimplePublisher(Node):
    """
    A ROS 2 node that publishes a String message to the 'chatter' topic every 0.5 seconds.
    """
    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher:
        # 1. Message type (StringMsg)
        # 2. Topic name ('chatter')
        # 3. QoS History Depth (10) - See QoS section below for more details
        self.publisher_ = self.create_publisher(StringMsg, 'chatter', 10)
        
        # Create a timer that calls self.timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0 # Counter for message data

    def timer_callback(self):
        msg = StringMsg()
        msg.data = f'Hello ROS 2 World: {self.i}' # Construct the message
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log what we published
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node) # Keep the node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Figure 3.4: Python code for a ROS 2 publisher node.*

**Code Example 3.3: `simple_subscriber.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

class SimpleSubscriber(Node):
    """
    A ROS 2 node that subscribes to the 'chatter' topic and prints received messages.
    """
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscriber:
        # 1. Message type (StringMsg)
        # 2. Topic name ('chatter')
        # 3. Callback function (self.listener_callback)
        # 4. QoS History Depth (10)
        self.subscription = self.create_subscription(
            StringMsg,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info("Subscribing to 'chatter' topic.")

    def listener_callback(self, msg):
        """
        Callback function executed when a message is received on the 'chatter' topic.
        """
        self.get_logger().info(f'I heard: "{msg.data}" (via custom QoS)')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Figure 3.5: Python code for a ROS 2 subscriber node.*

To run these: Open two separate terminals, source your ROS 2 environment in both. In the first, run `python simple_publisher.py`. In the second, run `python simple_subscriber.py`. You should see the subscriber printing the messages published by the publisher.

#### 3.3.2 Quality of Service (QoS) Policies

DDS's powerful QoS policies allow you to define how data is transmitted and received. These settings are crucial for reliability, latency, and resource management. When creating publishers and subscribers, you can specify a `QoSProfile`.

<Mermaid chart={`
graph TD;
    subgraph "Key QoS Policies"
        A[Reliability] --> A1[Best Effort<br/>(fast, may lose data)];
        A --> A2[Reliable<br/>(guaranteed delivery)];
        B[Durability] --> B1[Volatile<br/>(only live data)];
        B --> B2[Transient Local<br/>(last published data persists)];
        C[History] --> C1[Keep Last<br/>(buffer only N messages)];
        C --> C2[Keep All<br/>(buffer all messages)];
        D[Liveliness] --> D1[Automatic];
        D --> D2[Manual by Topic];
    end
    style A fill:#aaffaa
    style B fill:#aaaaff
    style C fill:#aaffaa
    style D fill:#aaaaff
`} />
*Figure 3.6: Overview of key ROS 2 Quality of Service (QoS) policies.*

**Example QoS Use Cases:**
*   **LiDAR Scan:** `BEST_EFFORT` reliability, `KEEP_LAST` history (depth 1) for the newest scan, as older scans are quickly outdated.
*   **Robot Critical State:** `RELIABLE` reliability, `TRANSIENT_LOCAL` durability (to get the last known state), `KEEP_LAST` history (depth 1).
*   **Controller Commands:** `RELIABLE` reliability, `KEEP_LAST` history.

**Code Example 3.4: `qos_subscriber.py`**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String as StringMsg

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        # Define a custom QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, # Ensure guaranteed delivery
            history=QoSHistoryPolicy.KEEP_LAST,        # Only keep the last message
            depth=1                                    # Buffer size of 1
        )
        self.subscription = self.create_subscription(
            StringMsg,
            'chatter',
            self.listener_callback,
            qos_profile) # Apply the custom QoS profile
        self.get_logger().info("Subscribing to 'chatter' with custom QoS.")

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}" (via custom QoS)')

def main(args=None):
    rclpy.init(args=args)
    node = QoSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Figure 3.7: Example of applying a custom QoS profile to a subscriber.*

#### 3.3.3 Services: Synchronous Request/Response

When you need an immediate, single response to a request, **Services** are the pattern to use [^5]. They operate on a client-server model.

*   **Service Server:** A node that offers a specific service and processes requests.
*   **Service Client:** A node that sends a request to a service server and waits for a response.
*   **Synchronous:** The client typically blocks until it receives a response or a timeout occurs.

**Key Characteristics:**
*   **One-to-One:** A client sends a request to a single server.
*   **Blocking:** The client waits for a response, making it suitable for RPC (Remote Procedure Call) patterns.
*   **Strongly Typed:** Requests and responses adhere to predefined service message types.

**Use Cases in Humanoid Robotics:**
*   Querying information (e.g., "What is the battery level?").
*   Triggering a one-shot action (e.g., "Reset the motor encoders").
*   Calculating a complex plan (e.g., "Compute an inverse kinematics solution").

<Mermaid chart={`
sequenceDiagram
    participant Client as Node A: Vision Client
    participant Server as Node B: IK Solver Server
    Client->>Server: Request: Calculate IK for (x,y,z)
    Note right of Server: Process Inverse Kinematics
    Server-->>Client: Response: Joint Angles (q1, q2, q3)
`} />
*Figure 3.8: Sequence diagram for Service Communication.*

ROS 2 provides a command-line tool to interact with services. If you had a service named `/add_two_ints` that expected two integers and returned their sum, you could call it from the terminal:
`ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 10}'`

#### 3.3.4 Actions: Goal-Oriented Tasks with Feedback

**Actions** are built on top of topics and services and are designed for long-running, goal-oriented tasks that require continuous feedback and can be preempted or canceled [^6]. They also follow a client-server model but are asynchronous.

*   **Action Server:** Executes a goal, provides periodic feedback, and sends a final result.
*   **Action Client:** Sends a goal, receives continuous feedback, and gets a final result. It can also cancel a goal.
*   **Asynchronous:** The client does not block after sending the goal, allowing it to perform other tasks while the action is in progress.

**Key Characteristics:**
*   **Long-Running:** Ideal for tasks that take a significant amount of time.
*   **Feedback:** The client receives updates on the progress of the goal.
*   **Preemptable:** The client can request to stop or cancel the ongoing goal.
*   **Strongly Typed:** Goals, feedback, and results adhere to predefined action message types.

**Use Cases in Humanoid Robotics:**
*   **Navigation:** "Go to the kitchen." (Feedback: current position; Result: reached goal/failed; Preempt: emergency stop).
*   **Manipulation:** "Pick up the red cup." (Feedback: gripper closing progress; Result: picked up/missed; Preempt: detected collision).
*   **Complex Motion:** "Perform a backflip." (Feedback: current phase of flip; Result: successful/failed; Preempt: detected instability).

<Mermaid chart={`
sequenceDiagram
    participant Client as Action Client
    participant Server as Action Server
    Client->>Server: Goal: Move to (X,Y)
    Server->>Client: Acknowledge Goal
    loop while Goal not achieved
        Server-->>Client: Feedback: Current position (x,y)
    end
    Server-->>Client: Result: Goal Reached
    Client->>Server: OPTIONAL: Cancel Goal
`} />
*Figure 3.9: Sequence diagram for Action Communication with feedback and preemption.*

### 3.4 Parameters: Dynamic Configuration

**Parameters** allow nodes to expose configurable values that can be changed dynamically at runtime, without recompiling or restarting the node. Each node manages its own set of parameters [^7].

**Use Cases:**
*   Adjusting the gain of a PID controller.
*   Changing the publishing frequency of a sensor node.
*   Setting a threshold for an object detection algorithm.
*   Configuring the robot's operating mode (e.g., "safe mode," "performance mode").

**Code Example 3.5: `configurable_node.py`**
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor # For adding descriptions to parameters
import time

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # 1. Declare a parameter with a default value and an optional description
        param_descriptor = ParameterDescriptor(description='Greeting message to prepend.')
        self.declare_parameter('greeting_prefix', 'Hello', param_descriptor)
        
        param_descriptor_rate = ParameterDescriptor(description='Publishing rate in Hz.')
        self.declare_parameter('publish_rate_hz', 1.0, param_descriptor_rate) # Default to 1 Hz

        # Create a timer based on the parameter
        self.update_timer_from_param()
        
        self.get_logger().info('Configurable node started. Try changing "greeting_prefix" or "publish_rate_hz"!')

    def timer_callback(self):
        # 2. Get the current value of the parameter
        greeting_prefix = self.get_parameter('greeting_prefix').get_parameter_value().string_value
        self.get_logger().info(f'{greeting_prefix}, the time is {time.time()}')

    def update_timer_from_param(self):
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        if hasattr(self, 'timer'):
            self.timer.destroy() # Destroy existing timer if any
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(f'Publishing rate set to {publish_rate} Hz.')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Figure 3.10: Python code for a ROS 2 node with dynamic parameters.*

To interact with parameters from the command line:
*   **List parameters:** `ros2 param list`
*   **Get a parameter:** `ros2 param get /configurable_node greeting_prefix`
*   **Set a parameter:** `ros2 param set /configurable_node greeting_prefix "Greetings Earthling"`
*   **Dump all parameters:** `ros2 param dump /configurable_node`

### 3.5 The ROS 2 Build System: `ament` and `colcon`

For larger projects with multiple nodes and dependencies, individual Python scripts are not sufficient. ROS 2 uses a build system composed of `ament` and `colcon` to manage packages, dependencies, and compilation.

*   **`ament`:** The underlying build framework. It defines how ROS 2 packages are structured and how they declare their dependencies. It supports various build types, including `ament_python` for Python packages and `ament_cmake` for C++ packages.
*   **`colcon`:** The command-line tool that orchestrates the build process across multiple `ament` packages. It can build, test, and install all packages in a workspace.

#### ROS 2 Workspace Structure

A ROS 2 workspace is a directory containing one or more source directories (usually named `src`). Each source directory holds your ROS 2 packages.

<Mermaid chart={`
graph TD;
    A[my_ros2_ws] --> B[src/];
    A --> C[build/];
    A --> D[install/];
    A --> E[log/];
    B --> F[my_first_package/];
    B --> G[my_sensor_package/];
    F --> H[package.xml];
    F --> I[setup.py or CMakeLists.txt];
    F --> J[nodes/];
    F --> K[launch/];
    G --> L[package.xml];
`} />
*Figure 3.11: Typical ROS 2 Workspace Structure.*

*   **`my_ros2_ws/`:** The root of your workspace.
*   **`src/`:** Contains the source code for your ROS 2 packages.
*   **`build/`:** Stores intermediate build files.
*   **`install/`:** Holds the installed files (executables, libraries, Python modules, launch files).
*   **`log/`:** Contains logs from `colcon` builds.
*   **`package.xml`:** Every ROS 2 package must have this manifest file. It declares the package's name, version, description, maintainers, license, and (crucially) its dependencies.
*   **`setup.py` (for Python) / `CMakeLists.txt` (for C++):** Defines how the package is built.

#### Building and Sourcing Your Workspace

1.  **Navigate to Workspace Root:** `cd ~/my_ros2_ws`
2.  **Build:** `colcon build` (or `colcon build --packages-select <package_name>` for specific packages).
3.  **Source the Setup File:** After a successful build, you *must* source the setup file in the `install` directory to make your new packages, executables, and libraries available in your current shell session.
    `source install/setup.bash` (for bash/zsh)
    `source install/setup.ps1` (for PowerShell)

### 3.6 Essential ROS 2 Tooling and Utilities

ROS 2 provides a rich set of command-line tools for introspection, debugging, and managing your robotic system. These are indispensable for development.

<Mermaid chart={`
graph TD;
    A[ros2 run] --> A1[Execute Node];
    B[ros2 topic] --> B1[List Topics];
    B --> B2[Echo Messages];
    B --> B3[Publish Messages];
    C[ros2 node] --> C1[List Nodes];
    C --> C2[Info on Node];
    D[ros2 service] --> D1[List Services];
    D --> D2[Call Service];
    E[ros2 param] --> E1[List Parameters];
    E --> E2[Get/Set Parameter];
F[ros2 launch] --> F1[Start Multiple Nodes];
    F --> F2[Manage Complex Systems];
    G[rviz2] --> G1[3D Visualization];
    H[rqt] --> H1[GUI Tools];
    style A fill:#aaffaa
    style B fill:#aaaaff
    style C fill:#aaffaa
    style D fill:#aaaaff
    style E fill:#aaffaa
    style F fill:#aaaaff
    style G fill:#aaffaa
    style H fill:#aaaaff
`} />
*Figure 3.12: Essential ROS 2 Command-Line Tools.*

*   **`ros2 run <package_name> <executable_name>`:** Executes a node from a built package.
*   **`ros2 topic list`:** Lists all active topics.
*   **`ros2 topic echo <topic_name>`:** Displays messages being published on a topic.
*   **`ros2 topic pub <topic_name> <message_type> <args>`:** Publishes a single message to a topic.
*   **`ros2 node list`:** Lists all active nodes.
*   **`ros2 node info <node_name>`:** Shows information about a specific node (publishers, subscribers, services, actions, parameters).
*   **`ros2 service list`:** Lists all active services.
*   **`ros2 service call <service_name> <service_type> <args>`:** Calls a service.
*   **`ros2 param list /node_name`:** Lists parameters exposed by a node.
*   **`ros2 param get /node_name <param_name>`:** Gets the value of a specific parameter.
*   **`ros2 param set /node_name <param_name> <value>`:** Sets the value of a specific parameter.
*   **`ros2 launch <package_name> <launch_file_name>`:** Starts a collection of nodes and other processes defined in a launch file. This is crucial for complex systems.
*   **`rviz2`:** The powerful 3D visualization tool for ROS 2. It can display sensor data (point clouds, images), robot models, navigation paths, and much more.
*   **`rqt`:** A suite of GUI tools for ROS 2, providing a modular framework for graphical debugging and interaction. Popular plugins include `rqt_graph` (visualizing the ROS graph) and `rqt_plot` (plotting topic data).

### 3.7 Security in ROS 2: SROS 2

Given that robots are increasingly deployed in real-world, potentially sensitive or dangerous environments, security is paramount. ROS 2 addresses this with **Security in ROS 2 (SROS 2)**, which leverages the security features inherent in DDS [^8].

SROS 2 provides:
*   **Authentication:** Ensures that only authorized nodes can join the ROS 2 graph.
*   **Authorization:** Defines which authenticated nodes are allowed to communicate on which topics, services, or actions.
*   **Encryption:** Protects the confidentiality and integrity of messages exchanged between nodes.

<Mermaid chart={`
graph TD;
    A[Node A] -- Authenticated & Authorized & Encrypted --> B[Node B];
    subgraph "SROS 2 Features"
        C[Authentication]
        D[Authorization]
        E[Encryption]
    end
    C & D & E --> A;
    C & D & E --> B;
    style A fill:#ccffcc
    style B fill:#ccffcc
`} />
*Figure 3.13: SROS 2 secures communication between nodes.*

While configuring SROS 2 is beyond the scope of this introductory chapter, understanding its existence and importance is crucial. For any deployment of a humanoid robot in a real-world scenario, SROS 2 should be a primary consideration.

### Conclusion: A Solid Foundation for Robotics

You now possess a comprehensive understanding of the ROS 2 architecture. We've dissected its evolutionary leap from ROS 1, embraced the power of DDS, and explored the core communication patterns—nodes, topics, services, actions, and parameters. We've also touched upon the robust build system, essential command-line tools, and the critical importance of security.

This foundational knowledge is not merely theoretical; it is the practical toolkit you will use to bring our humanoid robot to life. In the next chapter, we will transition from these concepts to hands-on development, creating our very first ROS 2 package and starting the journey of building intelligence from the ground up. The nervous system is ready; it's time to build the brain.

---

## References

[^1]: ROS 2 Documentation. "About ROS 2 — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Concepts/About-ROS-2-introduction.html](https://docs.ros.org/en/humble/Concepts/About-ROS-2-introduction.html).
[^2]: ROS 2 Documentation. "About DDS and ROS Middleware Abstraction — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Concepts/About-DDS-and-ROS-Middleware-Abstraction.html](https://docs.ros.org/en/humble/Concepts/About-DDS-and-ROS-Middleware-Abstraction.html).
[^3]: ROS 2 Documentation. "About Nodes — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Concepts/About-Nodes.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).
[^4]: ROS 2 Documentation. "Understanding ROS 2 Topics — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).
[^5]: ROS 2 Documentation. "Understanding ROS 2 Services — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).
[^6]: ROS 2 Documentation. "Understanding ROS 2 Actions — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).
[^7]: ROS 2 Documentation. "About Parameters — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Concepts/About-Parameters.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Parameters/Using-Parameters.html).
[^8]: ROS 2 Documentation. "SROS 2 — ROS 2 Documentation: Humble documentation." *ROS.org*, [https://docs.ros.org/en/humble/Concepts/About-Security.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/About-Security/About-Security.html).