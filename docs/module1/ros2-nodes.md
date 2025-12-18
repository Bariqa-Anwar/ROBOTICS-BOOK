---
sidebar_position : 1
---

# ROS 2 Nodes and Topics

In ROS 2, the fundamental unit of computation is a **Node**. Nodes are processes that perform specific tasks, such as reading sensor data, controlling a motor, or performing calculations. Nodes communicate with each other by sending and receiving **Messages** over **Topics**. This publish/subscribe messaging pattern is central to ROS 2's distributed architecture.

This section will introduce these core concepts and guide you through creating your first ROS 2 Python nodes using `rclpy`, the Python client library for ROS 2. We will also set up a ROS 2 workspace, which is an organized collection of packages.

## 1. ROS 2 Workspace Setup

A ROS 2 workspace is a directory where you store and build your ROS 2 packages.

### 1.1 Create a new workspace

Let's create a new workspace named `ros2_ws` in your home directory.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 1.2 Initialize the workspace

You can initialize an empty workspace with the following command :

```bash
# From ~/ros2_ws
colcon build --packages-select
```

This command will tell `colcon` (the ROS 2 build tool) to set up the necessary build files, though it won't build any packages yet as `src` is empty.

### 1.3 Source the workspace

Before you can use any packages from your workspace, you need to source its setup files. This overlays your workspace on top of your ROS 2 installation.

```bash
# From ~/ros2_ws
source install/setup.bash
```

It's recommended to add this line to your `~/.bashrc` file if you want it to be sourced automatically every time you open a new terminal :

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 2. Creating a ROS 2 Package

A ROS 2 package is a container for your nodes, libraries, configuration files, and other ROS-related items.

### 2.1 Create a Python Package

Navigate into the `src` directory of your workspace and create a new Python package named `my_robot_pkg`.

```bash
# From ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

This command creates a new directory `my_robot_pkg` with a basic structure, including a `setup.py` and `package.xml` file, which are crucial for defining and building your Python package.

## 3. ROS 2 Nodes : Publisher and Subscriber
<!-- DIAGRAM : ROS2_Computation_Graph -->

We will create two simple Python nodes : a publisher that sends "Hello ROS" messages, and a subscriber that receives and prints them.

### 3.1 Publisher Node

A publisher node sends messages to a topic.

```python
# ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node) :

    def __init__(self) :
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) :
        msg = String()
        msg.data = 'Hello ROS : %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing : "%s"' % msg.data)
        self.i += 1

def main(args=None) :
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
```

### 3.2 Subscriber Node

A subscriber node receives messages from a topic.

```python
# ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node) :

    def __init__(self) :
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg) :
        self.get_logger().info('I heard : "%s"' % msg.data)

def main(args=None) :
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
```

## 4. Building and Running Your Nodes

After creating your Python files and updating `setup.py`, you need to build your ROS 2 package and then you can run your nodes.

### 4.1 Build the Package

Navigate to your workspace root (`~/ros2_ws`) and build your package :

```bash
# From ~/ros2_ws
colcon build --packages-select my_robot_pkg
```

### 4.2 Source the Workspace (again)

After building, you need to re-source your workspace to make the new executables available :

```bash
# From ~/ros2_ws
source install/setup.bash
```

### 4.3 Run the Nodes

Open two separate terminals. In the first terminal, run the publisher :

```bash
ros2 run my_robot_pkg simple_publisher
```

In the second terminal, run the subscriber :

```bash
ros2 run my_robot_pkg simple_subscriber
```

You should see the publisher sending messages and the subscriber receiving them. You can also inspect the topic using `ros2 topic echo /my_topic`.
