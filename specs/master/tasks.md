# Tasks: AI/Spec-Driven Book: Physical AI & Humanoid Robotics

**Input**: User provided high-level prompt for /sp.tasks
**Prerequisites**: plan.md (conceptual), constitution.md (principles and standards)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/module this task belongs to (e.g., [Module 0], [Module 1])
- Include exact file paths in descriptions

## Path Conventions

- For documentation: `docs/book/<module-name>/<chapter-name>.mdx`
- For code: `src/packages/<package-name>/<file>.py` (or similar for config files)

---

## Phase 0: Module 0 - Introduction/Setup 🎯 Human Checkpoint

**Goal**: Prepare the development environment and create a foundational URDF model.
**Focus**: Environment configuration and basic URDF creation.

### Environment Setup (ROS 2, Python, Docusaurus)

- [X] T001 [Module 0] Write MDX section: ROS 2 Installation Guide (`docs/book/introduction/ros2-install.mdx`)
    - Criterion: MDX file created with basic ROS 2 Humble/Iron installation steps.
- [X] T002 [Module 0] Add code snippet to `ros2-install.mdx`: Verify ROS 2 installation with `ros2 daemon --version`.
    - Criterion: Code snippet added, output shows ROS 2 version.
- [X] T003 [Module 0] Write MDX section: Python Environment Setup (`docs/book/introduction/python-env.mdx`)
    - Criterion: MDX file created with Python 3.10+ virtual environment setup steps.
- [X] T004 [Module 0] Add code snippet to `python-env.mdx`: Verify Python version (`python --version`) and pip list.
    - Criterion: Code snippet added, output shows Python 3.10+ and pip list.
- [X] T005 [Module 0] Write MDX section: Docusaurus Project Setup (`docs/book/introduction/docusaurus-setup.mdx`)
    - Criterion: MDX file created with Docusaurus CLI installation and project creation steps.
- [X] T006 [Module 0] Add code snippet to `docusaurus-setup.mdx`: Verify Docusaurus setup (`npm start`).
    - Criterion: Code snippet added, output shows Docusaurus starting.
- [X] T007 [Module 0] Create initial `_category_.json` for Introduction/Setup (`docs/book/introduction/_category_.json`)
    - Criterion: JSON file created, defining category label and position.

### Basic URDF Creation

- [X] T008 [Module 0] Write MDX section: Introduction to URDF Concepts (`docs/book/introduction/urdf-concepts.mdx`)
    - Criterion: MDX file created, defining link, joint, visual, collision, inertial elements.
- [X] T009 [Module 0] Create `simple_box.urdf` file in a new `urdf/` directory (e.g., `src/urdf/simple_box.urdf`)
    - Criterion: `simple_box.urdf` exists and contains a `<link name="base_link">` element.
- [X] T010 [Module 0] Add code snippet to `urdf-concepts.mdx`: Content of `simple_box.urdf`.
    - Criterion: Code block for `simple_box.urdf` added to MDX.
- [X] T011 [Module 0] Write MDX section: Visualizing URDF in RViz2 (`docs/book/introduction/rviz2-urdf.mdx`)
    - Criterion: MDX file created, explaining RViz2 setup for URDF.
- [X] T012 [Module 0] Add code snippet to `rviz2-urdf.mdx`: Command to launch RViz2 and load `simple_box.urdf`.
    - Criterion: Code snippet for RViz2 launch added.
- [X] T013 [Module 0] Create `simple_cylinder_joint.urdf` (two links, one joint) (`src/urdf/simple_cylinder_joint.urdf`)
    - Criterion: `simple_cylinder_joint.urdf` exists and contains a `<joint name="joint1" type="continuous">` element.
- [X] T014 [Module 0] Add code snippet to `rviz2-urdf.mdx`: Content of `simple_cylinder_joint.urdf`.
    - Criterion: Code block for `simple_cylinder_joint.urdf` added to MDX.
- [X] T015 [Module 0] Add code snippet to `rviz2-urdf.mdx`: Command to launch RViz2 and load `simple_cylinder_joint.urdf`.
    - Criterion: Code snippet for RViz2 launch with `simple_cylinder_joint.urdf` added.
- [X] T016 [Module 0] Add Visualization Tagging for URDF structure diagram in `urdf-concepts.mdx`.
    - Criterion: `<!-- DIAGRAM: URDF_Structure -->` tag placed appropriately.

---

## Phase 1: Module 1 - Foundation 🎯 Human Checkpoint

**Goal**: Create basic ROS 2 rclpy nodes and integrate a robot model into Gazebo.
**Focus**: Basic ROS 2 rclpy node creation and Gazebo world integration.

### ROS 2 rclpy Node Creation

- [X] T017 [Module 1] Write MDX section: ROS 2 Nodes and Topics (`docs/book/module1/ros2-nodes.mdx`)
    - Criterion: MDX file created, introducing nodes, topics, messages, and ROS 2 workspace setup.
- [X] T018 [Module 1] Create ROS 2 workspace (e.g., `ros2_ws/`) and `src/` directory.
    - Criterion: `ros2_ws/src` directory exists.
- [X] T019 [Module 1] Create `my_robot_pkg` ROS 2 Python package (`ros2_ws/src/my_robot_pkg`).
    - Criterion: `my_robot_pkg` directory created with `package.xml` and `setup.py`.
- [X] T020 [Module 1] Create `simple_publisher.py` (`ros2_ws/src/my_robot_pkg/simple_publisher.py`) to publish "Hello ROS" on `/my_topic`.
    - Criterion: Python file created, contains a `Node` and `create_publisher`.
- [X] T021 [Module 1] Add code snippet for `simple_publisher.py` to `ros2-nodes.mdx`.
    - Criterion: Code block for `simple_publisher.py` added to MDX.
- [X] T022 [Module 1] Create `simple_subscriber.py` (`ros2_ws/src/my_robot_pkg/simple_subscriber.py`) to subscribe to `/my_topic`.
    - Criterion: Python file created, contains a `Node` and `create_subscription`.
- [X] T023 [Module 1] Add code snippet for `simple_subscriber.py` to `ros2-nodes.mdx`.
    - Criterion: Code block for `simple_subscriber.py` added to MDX.
- [X] T024 [Module 1] Update `setup.py` and `package.xml` for `my_robot_pkg` to include entry points.
    - Criterion: `setup.py` and `package.xml` modified to allow running publisher/subscriber.
- [X] T025 [Module 1] Add code snippets to `ros2-nodes.mdx`: Build and run publisher/subscriber nodes.
    - Criterion: Code snippets for `colcon build` and `ros2 run` added.
- [X] T026 [Module 1] Add Visualization Tagging for ROS 2 computation graph in `ros2-nodes.mdx`.
    - Criterion: `<!-- DIAGRAM: ROS2_Computation_Graph -->` tag placed appropriately.

### Gazebo World Integration

- [X] T027 [Module 1] Write MDX section: Introduction to Gazebo Simulation (`docs/book/module1/gazebo-intro.mdx`)
    - Criterion: MDX file created, introducing Gazebo and its role.
- [X] T028 [Module 1] Create `simple_world.world` Gazebo world file (`ros2_ws/src/my_robot_pkg/worlds/simple_world.world`).
    - Criterion: XML file created, defining a basic empty Gazebo world.
- [X] T029 [Module 1] Add code snippet for `simple_world.world` to `gazebo-intro.mdx`.
    - Criterion: Code block for `simple_world.world` added to MDX.
- [X] T030 [Module 1] Convert `simple_box.urdf` to `simple_robot.sdf` (SDF format) for Gazebo (`ros2_ws/src/my_robot_pkg/models/simple_robot.sdf`).
    - Criterion: `simple_robot.sdf` created, representing the box model in SDF.
- [X] T031 [Module 1] Add code snippet for `simple_robot.sdf` to `gazebo-intro.mdx`.
    - Criterion: Code block for `simple_robot.sdf` added to MDX.
- [X] T032 [Module 1] Add code snippets to `gazebo-intro.mdx`: Launch Gazebo with `simple_world.world` and spawn `simple_robot.sdf`.
    - Criterion: Code snippets for Gazebo launch and robot spawning added.
- [X] T033 [Module 1] Add Visualization Tagging for Gazebo interface in `gazebo-intro.mdx`.
    - Criterion: `<!-- DIAGRAM: Gazebo_Interface -->` tag placed appropriately.

---

## Phase 2: Module 2 - Sensors & Perception 🎯 Human Checkpoint

**Goal**: Integrate NVIDIA Isaac Sim/Vision and write perception nodes.
**Focus**: Integrating NVIDIA Isaac Sim/Vision and writing perception nodes.

### NVIDIA Isaac Sim/Vision Research (Internal Checkpoint)

- [X] T034 [Module 2] Write MDX section: Introduction to NVIDIA Isaac Sim (`docs/book/module2/isaac-sim-intro.mdx`)
    - Criterion: MDX file created, describing Isaac Sim's features and its ROS 2 bridge.
- [X] T035 [Module 2] Research Isaac Sim installation and ROS 2 bridge setup process.
    - Criterion: Installation steps for Isaac Sim and its ROS 2 bridge are documented.
- [X] T036 [Module 2] Identify key Isaac Sim features for sensor simulation (e.g., cameras, LiDAR, depth sensors).
    - Criterion: List of relevant Isaac Sim sensor features compiled.
- [X] T037 [Module 2] Document differences and trade-offs between Gazebo and Isaac Sim for sensor modeling (`isaac-sim-intro.mdx`).
    - Criterion: Comparison table or detailed paragraphs on Gazebo vs. Isaac Sim sensor modeling added to MDX.

### ROS 2 Driver Implementation (Virtual Sensors)

- [X] T038 [Module 2] Write MDX section: Virtual Sensors in Isaac Sim (`docs/book/module2/virtual-sensors.mdx`)
    - Criterion: MDX file created, outlining how to create virtual sensors in Isaac Sim and publish data via ROS 2.
- [X] T039 [Module 2] Set up a basic Isaac Sim project with ROS 2 bridge enabled and a simple environment.
    - Criterion: Isaac Sim project launched, and ROS 2 bridge verified as active.
- [X] T040 [Module 2] Create a virtual camera in Isaac Sim attached to the simulated robot.
    - Criterion: Virtual camera configured within Isaac Sim, capturing scene.
- [X] T041 [Module 2] Configure Isaac Sim to publish virtual camera data as ROS 2 `sensor_msgs/Image`.
    - Criterion: ROS 2 topic `/isaac_camera/image_raw` (or similar) is actively publishing image data.
- [X] T042 [Module 2] Create `image_processor_node.py` (`ros2_ws/src/my_robot_pkg/nodes/image_processor_node.py`) to subscribe to virtual camera data.
    - Criterion: Python file created, contains `create_subscription` for `sensor_msgs/Image`.
- [X] T043 [Module 2] Add code snippet for `image_processor_node.py` to `virtual-sensors.mdx`.
    - Criterion: Code block for `image_processor_node.py` added to MDX.
- [X] T044 [Module 2] Implement basic image processing (e.g., grayscale conversion) within `image_processor_node.py`.
    - Criterion: `image_processor_node.py` successfully converts images to grayscale.
- [X] T045 [Module 2] Publish processed image to a new ROS 2 topic (e.g., `/processed_image`).
    - Criterion: `image_processor_node.py` publishes data on `/processed_image` topic.
- [X] T046 [Module 2] Add Visualization Tagging for Isaac Sim/ROS 2 sensor pipeline in `virtual-sensors.mdx`.
    - Criterion: `<!-- DIAGRAM: IsaacSim_Sensor_Pipeline -->` tag placed appropriately.

---

## Phase 3: Module 3 - Control & Action 🎯 Human Checkpoint

**Goal**: Implement advanced control loops and inverse kinematics for robotic manipulation.
**Focus**: Advanced control loops and inverse kinematics implementation.

### Control Algorithm Research (Internal Checkpoint)

- [ ] T047 [Module 3] Write MDX section: Robotic Control Fundamentals (`docs/book/module3/control-fundamentals.mdx`)
    - Criterion: MDX file created, introducing PID control and basic Jacobian concepts.
- [ ] T048 [Module 3] Research principles of PID control for joint position and velocity.
    - Criterion: Key PID equations and tuning considerations are documented.
- [ ] T049 [Module 3] Research the basic Jacobian matrix for forward and inverse kinematics.
    - Criterion: Explanation of Jacobian's role in relating joint and end-effector velocities is documented.
- [ ] T050 [Module 3] Summarize selected control approaches for a multi-joint robotic arm within `control-fundamentals.mdx`.
    - Criterion: Summary of common robotic arm control strategies (e.g., joint space, task space) added to MDX.

### Inverse Kinematics Implementation

- [ ] T051 [Module 3] Write MDX section: Inverse Kinematics for Robotic Arms (`docs/book/module3/inverse-kinematics.mdx`)
    - Criterion: MDX file created, explaining IK concepts and implementation approaches.
- [ ] T052 [Module 3] Extend an existing URDF model (`src/urdf/simple_arm.urdf`) to include a simple 3-DOF arm.
    - Criterion: `simple_arm.urdf` created or modified to represent a 3-DOF arm.
- [ ] T053 [Module 3] Create `arm_controller_node.py` (`ros2_ws/src/my_robot_pkg/nodes/arm_controller_node.py`) for receiving target poses and publishing joint commands.
    - Criterion: Python file created, contains `create_subscription` for target poses and `create_publisher` for joint commands.
- [ ] T054 [Module 3] Implement a basic IK solver (e.g., pseudo-inverse Jacobian) within `arm_controller_node.py`.
    - Criterion: `arm_controller_node.py` includes a functional IK calculation logic.
- [ ] T055 [Module 3] Add code snippet for `arm_controller_node.py` to `inverse-kinematics.mdx`.
    - Criterion: Code block for `arm_controller_node.py` added to MDX.
- [ ] T056 [Module 3] Add Visualization Tagging for IK solution process in `inverse-kinematics.mdx`.
    - Criterion: `<!-- DIAGRAM: IK_Solution_Process -->` tag placed appropriately.

---

## Phase 4: Module 4 - HRI & VLA 🎯 Human Checkpoint

**Goal**: Integrate LLM (GPT) and VLA (Whisper) for human-robot interaction through natural language.
**Focus**: LLM (GPT) and VLA (Whisper) integration for human interaction.

### Voice Interaction (Whisper)

- [ ] T057 [Module 4] Write MDX section: Speech-to-Text with OpenAI Whisper (`docs/book/module4/whisper-stt.mdx`)
    - Criterion: MDX file created, introducing Whisper and its API integration for STT.
- [ ] T058 [Module 4] Set up OpenAI API key securely (e.g., environment variable).
    - Criterion: OpenAI API key configured for use in Python environment.
- [ ] T059 [Module 4] Create `speech_to_text_node.py` (`ros2_ws/src/my_robot_pkg/nodes/speech_to_text_node.py`) to record audio and send to Whisper API.
    - Criterion: Python file created, contains audio recording logic and Whisper API call.
- [ ] T060 [Module 4] Configure `speech_to_text_node.py` to publish transcribed text to a ROS 2 topic (e.g., `/human_speech_text`).
    - Criterion: `speech_to_text_node.py` publishes string messages on `/human_speech_text`.
- [ ] T061 [Module 4] Add code snippet for `speech_to_text_node.py` to `whisper-stt.mdx`.
    - Criterion: Code block for `speech_to_text_node.py` added to MDX.

### LLM Integration (GPT) & Response Generation

- [ ] T062 [Module 4] Write MDX section: LLM for Command Understanding and Response (`docs/book/module4/gpt-hri.mdx`)
    - Criterion: MDX file created, explaining GPT integration for command parsing and response generation.
- [ ] T063 [Module 4] Create `command_parser_node.py` (`ros2_ws/src/my_robot_pkg/nodes/command_parser_node.py`) to subscribe to transcribed text.
    - Criterion: Python file created, contains `create_subscription` for `/human_speech_text`.
- [ ] T064 [Module 4] Integrate GPT API within `command_parser_node.py` to interpret commands and generate responses.
    - Criterion: `command_parser_node.py` successfully calls GPT API and receives text responses.
- [ ] T065 [Module 4] Publish robot's natural language response to a ROS 2 topic (e.g., `/robot_response_text`).
    - Criterion: `command_parser_node.py` publishes string messages on `/robot_response_text`.
- [ ] T066 [Module 4] Add code snippet for `command_parser_node.py` to `gpt-hri.mdx`.
    - Criterion: Code block for `command_parser_node.py` added to MDX.
- [ ] T067 [Module 4] Create `text_to_speech_node.py` (`ros2_ws/src/my_robot_pkg/nodes/text_to_speech_node.py`) to convert robot response to audio.
    - Criterion: Python file created, contains `create_subscription` for `/robot_response_text` and TTS logic.
- [ ] T068 [Module 4] Add Visualization Tagging for HRI workflow in `gpt-hri.mdx`.
    - Criterion: `<!-- DIAGRAM: HRI_Workflow -->` tag placed appropriately.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Module 0 (Introduction/Setup)**: No dependencies - can start immediately.
- **Module 1 (Foundation)**: Depends on Module 0 completion.
- **Module 2 (Sensors & Perception)**: Depends on Module 1 completion.
- **Module 3 (Control & Action)**: Depends on Module 2 completion.
- **Module 4 (HRI & VLA)**: Depends on Module 3 completion.

### Internal Checkpoints

- **Module 2**: "NVIDIA Isaac Sim/Vision Research" must be completed before "ROS 2 Driver Implementation (Virtual Sensors)".
- **Module 3**: "Control Algorithm Research" must be completed before "Inverse Kinematics Implementation".

---

## Implementation Strategy

### Incremental Delivery (Module by Module)

1. Complete Module 0: Introduction/Setup → Human Checkpoint & Validate.
2. Complete Module 1: Foundation → Human Checkpoint & Validate.
3. Complete Module 2: Sensors & Perception → Human Checkpoint & Validate.
4. Complete Module 3: Control & Action → Human Checkpoint & Validate.
5. Complete Module 4: HRI & VLA → Human Checkpoint & Validate.

---

## Notes

- Each task is designed to be atomic (15-30 minutes).
- Each task has a single, clear acceptance criterion.
- `[Module X]` label maps task to specific module for traceability.
- Verify each task's criterion upon completion.
