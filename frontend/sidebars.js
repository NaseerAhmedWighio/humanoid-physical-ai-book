// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro/index',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: Introduction to ROS 2',
      items: [
        'module-1-ros2/index',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Simulation with Gazebo and Unity',
      items: [
        'module-2-simulation/index',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac for Humanoid Control',
      items: [
        'module-3-nvidia-isaac/index',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision Language Action (VLA) Models',
      items: [
        'module-4-vla/index',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Weekly Content',
      items: [
        'weeks/week-01-02',
        'weeks/week-03',
        'weeks/week-04-05',
        'weeks/week-06',
        'weeks/week-07-09',
        'weeks/week-10-13',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/index',
        'appendices/mathematical-foundations',
        'appendices/hardware-specifications',
        'appendices/software-installation',
        'appendices/troubleshooting',
        'appendices/ros2-messages',
        'appendices/simulation-best-practices',
        'appendices/safety-guidelines',
        'appendices/ethics',
        'appendices/glossary',
        'appendices/additional-resources',
      ],
      collapsed: false,
    },
    'conclusion',
  ],
};

export default sidebars;