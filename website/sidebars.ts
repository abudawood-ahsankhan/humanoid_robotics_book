import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of your site
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar configuration
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: ROS 2 Robotics with Gazebo Simulation',
          items: ['modules/module-1/index'],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 2: Isaac AI Brain for Humanoid Robots',
          items: ['modules/module-2/index'],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 3: Isaac Navigation for Humanoid Robots',
          items: ['modules/module-3/index'],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA) System',
          items: ['modules/module-4/index'],
          collapsed: false,
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: ['ros2-fundamentals'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Python Integration',
      items: ['python-integration'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Robot Modeling',
      items: ['robot-modeling'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Development Workflow',
      items: ['development-workflow'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Spec-Driven Development',
      items: ['spec-driven-development'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Conclusion',
      items: ['conclusion'],
      collapsed: false,
    },
  ],
};

export default sidebars;