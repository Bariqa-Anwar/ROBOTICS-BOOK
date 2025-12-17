// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  mySidebar: [
    'intro',
    {
      type: 'category',
      label: 'Introduction & Setup',
      items: [
        'introduction/docusaurus-setup',
        'introduction/python-env',
        'introduction/ros2-install',
        'introduction/rviz2-urdf',
        'introduction/urdf-concepts',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS2 Foundations',
      items: [
        'modules/ros2/overview',
        'modules/ros2/first-ros2-humanoid-package',
        'modules/ros2/urdf-xacro-mastery',
        'module1/gazebo-intro',
        'module1/ros2-nodes',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'modules/simulation/overview',
        'modules/simulation/gazebo-ignition-deep-dive',
        'modules/simulation/unity-robotics-hub',
        'modules/simulation/indoor-environments-sensor-noise',
        'module2/isaac-sim-intro',
        'module2/virtual-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim & Control',
      items: [
        'modules/isaac/overview',
        'modules/isaac/isaac-sim-deep-dive',
        'modules/isaac/isaac-ros-jetson',
        'modules/isaac/bipedal-locomotion-whole-body-control',
        'module3/control-fundamentals',
        'module3/inverse-kinematics',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & AI',
      items: [
        'modules/vla/overview',
        'modules/vla/vision-language-action-models',
        'modules/vla/voice-to-action',
        'modules/vla/llm-to-ros2-translation',
        'modules/vla/capstone-full-autonomous-humanoid',
        'module4/gpt-hri',
        'module4/whisper-stt',
      ],
    },
    {
      type: 'category',
      label: 'Conclusion & Appendices',
      items: [
        'conclusion/future-household-humanoids',
        'conclusion/sim-to-real-deployment',
        'appendices/appendix-a-hardware-guide',
        'appendices/appendix-b-jetson-student-kit',
        'appendices/appendix-c-cloud-on-prem-costs',
        'appendices/appendix-d-prompt-history',
      ],
    },
  ],
};

module.exports = sidebars;