---
sidebar_position : 2
---

# Virtual Sensors in Isaac Sim

NVIDIA Isaac Sim excels at providing highly realistic and configurable virtual sensors, which are crucial for developing and testing perception algorithms without the need for physical hardware. These virtual sensors can generate data streams that closely mimic real-world sensors, and through the ROS 2 Bridge, this data can be seamlessly published to ROS 2 topics.

This section will guide you through creating and configuring virtual sensors in Isaac Sim, specifically focusing on a virtual camera, and how to publish its data to a ROS 2 `sensor_msgs/Image` topic.

## 1. Setting up a Basic Isaac Sim Project with ROS 2 Bridge

Before creating virtual sensors, ensure your Isaac Sim environment is set up and the ROS 2 Bridge extension is enabled (as discussed in [Introduction to NVIDIA Isaac Sim](./isaac-sim-intro.md)).

1.  **Launch Isaac Sim** : Start Isaac Sim from the Omniverse Launcher.
2.  **Create a New Stage** : Go to `File -> New -> Empty Stage`.
3.  **Enable ROS 2 Bridge** : Ensure the `omni.isaac.ros2_bridge` extension is enabled in the Extension Manager (`Window -> Extensions`).
4.  **Add a Robot (Optional but Recommended)** : To attach sensors to something meaningful, you can import a robot from the Isaac Sim assets browser (`Window -> Asset Browser -> Isaac Sim Assets -> Robots`). Drag a robot (e.g., `Franka Emika Panda`) onto the stage.

## 2. Creating a Virtual Camera

Isaac Sim allows you to add various types of cameras (RGB, depth, stereo, segmentation) to your scene.

1.  **Add a Camera** : In the `Stage` window, right-click on the robot's link where you want to attach the camera, then `Create -> Camera -> Camera`. Rename it (e.g., `robot_camera`).
2.  **Position and Orient** : Adjust the `Transform` properties of the camera (position, rotation) in the `Properties` window to place it appropriately on your robot.
3.  **Configure Camera Properties** :
    *   **Resolution** : In the `Properties` window for the camera, under `Camera`, adjust `Resolution` (e.g., 640x480).
    *   **Clipping Planes** : Set `Clipping Planes -> Near` and `Far` for rendering range.
    *   **Focal Length/FOV** : Adjust `Focal Length` or `Horizontal Aperture` and `Vertical Aperture` to control the camera's field of view.

## 3. Publishing Camera Data to ROS 2

To publish the virtual camera's data to a ROS 2 topic, you need to add a `ROS2 Camera` component to your camera.

1.  **Select Camera** : Select your `robot_camera` in the `Stage` window.
2.  **Add `ROS2 Camera` Component** : In the `Properties` window, scroll down and click `Add -> ROS 2 -> ROS2 Camera`.
3.  **Configure `ROS2 Camera` Properties** :
    *   **`cameraPrim`** : This should automatically link to your camera.
    *   **`topicName`** : Set this to your desired ROS 2 topic name (e.g., `/isaac_camera/image_raw`).
    *   **`frameId`** : Set this to the camera's frame ID (e.g., `robot_camera_link`).
    *   **`type`** : Select the type of data to publish (e.g., `rgb`, `depth`, `segmentation`).
    *   **`queueSize`** : Number of messages to queue.

Now, when the simulation is running, Isaac Sim will publish image data from this virtual camera to the specified ROS 2 topic. You can verify this using standard ROS 2 tools like `ros2 topic list` and `ros2 topic echo`.

## 4. Processing Sensor Data with ROS 2 (Python)
<!-- DIAGRAM : IsaacSim_Sensor_Pipeline -->

Once Isaac Sim is publishing sensor data to ROS 2 topics, you can create ROS 2 nodes to subscribe to this data, process it, and potentially republish it. Here's an example of a simple Python node that subscribes to camera data and re-publishes it (with placeholders for image processing).

```python
# ros2_ws/src/my_robot_pkg/nodes/image_processor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # You'll need to install OpenCV for Python and ros-humble-cv-bridge

class ImageProcessorNode(Node) :
    def __init__(self) :
        super().__init__('image_processor_node')
        self.subscription = self.create_subscription(
            Image,
            '/isaac_camera/image_raw', # Topic from Isaac Sim
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/processed_image', 10)


    def listener_callback(self, data) :
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

        # Perform some basic image processing (e.g., grayscale)
        # For simplicity, let's just convert it to grayscale
        # You'll need `import cv2` for this
        # gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        # self.publisher_.publish(self.br.cv2_to_imgmsg(gray_frame, "mono8"))

        # For now, just re-publish the original frame to demonstrate pipeline
        self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, "bgr8"))
        

def main(args=None) :
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
```
