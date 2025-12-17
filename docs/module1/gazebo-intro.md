---
sidebar_position: 2
---

# Introduction to Gazebo Simulation
<!-- DIAGRAM: Gazebo_Interface -->

Gazebo is a powerful 3D robot simulator that is widely used in the robotics community. It allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers the ability to simulate sensors, actuators, and even entire robotic systems with realistic physics.

Integrating your robot models into Gazebo allows you to test your ROS 2 nodes and control algorithms in a virtual environment before deploying them to a real robot. This saves time, reduces risk, and allows for rapid iteration of design and software.

## Key Features of Gazebo

*   **Physics Engine**: Gazebo uses physics engines (like ODE, Bullet, Simbody, DART) to accurately simulate rigid body dynamics, gravity, and various forces.
*   **High-Quality Graphics**: Renders realistic environments and robot models with textures, lighting, and shadows.
*   **Sensor Simulation**: Simulates various sensors such as cameras, LiDAR, depth sensors, IMUs, and more, providing realistic data streams to your ROS 2 nodes.
*   **Plugin Architecture**: Extensible through plugins, allowing you to customize robot behavior, sensor models, and world dynamics.
*   **ROS 2 Integration**: Provides seamless integration with ROS 2, allowing you to control robots and access sensor data through ROS 2 topics and services.

## Simulation Description Format (SDF)

While URDF (Unified Robot Description Format) is primarily used in ROS to describe the kinematic and dynamic properties of a robot, Gazebo uses SDF (Simulation Description Format) for a more comprehensive description of objects, environments, and robots within a simulation.

SDF is a more general XML format that can describe:

*   **Worlds**: The entire environment, including static objects (e.g., ground plane, buildings), lights, and physics properties.
*   **Models**: Robots or other dynamic objects, including their links, joints, sensors, and actuators.
*   **Sensors**: Detailed simulation of various sensors.

Although URDF can be converted to SDF for use in Gazebo, SDF provides more features for simulation-specific aspects like sensor noise, global lighting, and environmental properties. In this section, we will start with simple Gazebo worlds and integrate our URDF-defined robot models.

### Creating a Simple Gazebo World

Let's create a very basic Gazebo world file (`simple_world.world`) that includes a sun and a ground plane.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

### Integrating a Robot Model (SDF)

Now let's include our simple box robot, represented as an SDF model, into the documentation.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_robot">
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.001</iyy>
          <iyz>0.0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

### Launching Gazebo and Spawning the Robot

To launch Gazebo with our custom world and then spawn the robot, you can use the following commands:

1.  **Launch Gazebo with your world file**:
    ```bash
    gazebo --verbose -s libgazebo_ros_factory.so ros2_ws/src/my_robot_pkg/worlds/simple_world.world
    ```

2.  **Spawn your robot model**:
    ```bash
    ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file ros2_ws/src/my_robot_pkg/models/simple_robot.sdf -x 0 -y 0 -z 0.5
    ```

You should see Gazebo launch with a ground plane and a red box robot hovering above it.
