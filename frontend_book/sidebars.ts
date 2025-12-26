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
 * - Module 3: The AI-Robot Brain (NVIDIA Isaac)
 *   - Chapter 1: Isaac Sim
 *   - Chapter 2: Isaac ROS
 *   - Chapter 3: Nav2 for Humanoids
 * - Module 4: The AI-Robot Brain (VLA)
 *   - Chapter 1: Voice-to-Action
 *   - Chapter 2: Cognitive Planning
 *   - Chapter 3: Capstone Project
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
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      link: {
        type: 'doc',
        id: 'module-3-isaac/index',
      },
      items: [
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/nav2-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: The AI-Robot Brain (VLA)',
      link: {
        type: 'doc',
        id: 'module-4-vla/index',
      },
      items: [
        'module-4-vla/voice-to-action',
        'module-4-vla/cognitive-planning',
        'module-4-vla/capstone-project',
      ],
    },
  ],
};

export default sidebars;
