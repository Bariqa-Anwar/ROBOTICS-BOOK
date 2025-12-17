---
sidebar_position: 5
---

# Visualizing URDF in RViz2

RViz2 (ROS Visualization) is a powerful 3D visualizer for displaying sensor data, robot models, and planning outputs in ROS 2. It's an indispensable tool for debugging and understanding your robot's behavior. In this section, we'll learn how to load and visualize our URDF models in RViz2.

## 1. Launching RViz2

First, ensure your ROS 2 environment is sourced (as discussed in the [ROS 2 Installation Guide](./ros2-install.mdx)). You can launch RViz2 directly from your terminal.

```bash
rviz2
```

This will open the RViz2 application. Initially, it will appear empty.

## 2. Displaying Your Robot Model

To display your URDF robot model, you need to add a "RobotModel" display type to RViz2 and tell it where to find your URDF file and the robot's description parameter.

### Step 2.1: Set up Robot Description

ROS 2 uses a parameter server to store the robot's description. You'll typically use `xacro` (XML Macros) to parse your URDF and load it onto the parameter server. For simplicity, we'll directly load our simple URDF for now.

First, make sure `ros2 launch` and `urdf_tutorial` are installed if you don't have them:

```bash
sudo apt install ros-humble-ros2launch ros-humble-urdf-tutorial
```

Now, you can launch a minimal setup that publishes the robot description:

```bash
ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix urdf_tutorial)/share/urdf_tutorial/urdf/01-myfirst.urdf
```
*(Note: Replace `01-myfirst.urdf` with the path to your URDF file if needed for testing, or use a custom launch file as shown later)*

### Step 2.2: Add RobotModel Display in RViz2

1.  In the RViz2 window, click the "Add" button in the "Displays" panel (bottom left).
2.  Select "RobotModel" from the list and click "OK".
3.  In the "RobotModel" properties, expand it.
4.  Ensure the "Description" property is set to `robot_description`. This is the default parameter name where the URDF content is typically loaded.
5.  Set the "Fixed Frame" in the "Global Options" panel (top left) to `base_link` (or the name of your robot's root link).

You should now see your robot model rendered in the 3D view!

## Visualizing URDF Models

Let's use a simple launch file to display our `simple_box.urdf` in RViz2. Create a new launch file (e.g., `display_simple_box.launch.py`) in your ROS 2 package.

```python
# display_simple_box.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_pkg' # Replace with your package name
    urdf_file_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'simple_box.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file_path}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
```

Save this to `ros2_ws/src/my_robot_pkg/launch/display_simple_box.launch.py`. You will also need to update your `setup.py` to install this launch file. (We will cover this in detail in Module 1).

You can then launch it with:

```bash
ros2 launch my_robot_pkg display_simple_box.launch.py
```

This will automatically open RViz2 and display your `simple_box.urdf`.

### Visualizing `simple_cylinder_joint.urdf`

Now let's visualize a URDF with a joint. Here's the content of `simple_cylinder_joint.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_cylinder_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="joint1" type="continuous">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="child_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

To launch this in RViz2, you would create a similar launch file, replacing `simple_box.urdf` with `simple_cylinder_joint.urdf`, and then run:

```bash
ros2 launch my_robot_pkg display_simple_cylinder_joint.launch.py
```
