import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Robotics Book Sidebar Configuration
 *
 * Structure:
 * - Introduction
 * - Module 1: The Robotic Nervous System (ROS 2)
 *   - Chapter 1: ROS 2 Fundamentals
 *   - Chapter 2: Python Agents to Robot Control
 *   - Chapter 3: Humanoid Modeling with URDF
 * - Module 2: The Digital Twin (Gazebo & Unity)
 *   - Chapter 1: Gazebo Simulation Basics
 *   - Chapter 2: Unity for Robotics - Digital Twins
 *   - Chapter 3: Sensor Simulation
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/ros2-fundamentals',
        'module-1-ros2/python-ros-control',
        'module-1-ros2/humanoid-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'doc',
        id: 'module-2-simulation/index',
      },
      items: [
        'module-2-simulation/gazebo-simulation',
        'module-2-simulation/unity-digital-twin',
        'module-2-simulation/sensor-simulation',
      ],
    },
  ],
};

export default sidebars;
