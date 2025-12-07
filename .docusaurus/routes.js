import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '567'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '528'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '48f'),
            routes: [
              {
                path: '/docs/additional-materials/',
                component: ComponentCreator('/docs/additional-materials/', '4bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/additional-materials/cloud',
                component: ComponentCreator('/docs/additional-materials/cloud', 'cc8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/additional-materials/final_materials',
                component: ComponentCreator('/docs/additional-materials/final_materials', '0bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/additional-materials/hardware',
                component: ComponentCreator('/docs/additional-materials/hardware', 'f5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone-project/chapter1',
                component: ComponentCreator('/docs/capstone-project/chapter1', '83d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/', 'cbf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/chapter1',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/chapter1', '1c6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/chapter2',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/chapter2', '4a4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/chapter3',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/chapter3', 'eed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/chapter4',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/chapter4', 'dc8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/chapter5',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/chapter5', '62f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/citation_guide',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/citation_guide', 'c2a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1-ros2-nervous-system/glossary',
                component: ComponentCreator('/docs/module1-ros2-nervous-system/glossary', 'c43'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2-digital-twin-simulation/',
                component: ComponentCreator('/docs/module2-digital-twin-simulation/', '8a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2-digital-twin-simulation/chapter1',
                component: ComponentCreator('/docs/module2-digital-twin-simulation/chapter1', '9d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2-digital-twin-simulation/chapter2',
                component: ComponentCreator('/docs/module2-digital-twin-simulation/chapter2', '985'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2-digital-twin-simulation/chapter3',
                component: ComponentCreator('/docs/module2-digital-twin-simulation/chapter3', '562'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3-ai-brain-isaac/',
                component: ComponentCreator('/docs/module3-ai-brain-isaac/', '82b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3-ai-brain-isaac/chapter1',
                component: ComponentCreator('/docs/module3-ai-brain-isaac/chapter1', '270'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3-ai-brain-isaac/chapter2',
                component: ComponentCreator('/docs/module3-ai-brain-isaac/chapter2', 'b71'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3-ai-brain-isaac/chapter3',
                component: ComponentCreator('/docs/module3-ai-brain-isaac/chapter3', '878'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4-vla-robotics/',
                component: ComponentCreator('/docs/module4-vla-robotics/', '9cd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4-vla-robotics/chapter1',
                component: ComponentCreator('/docs/module4-vla-robotics/chapter1', '8f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4-vla-robotics/chapter2',
                component: ComponentCreator('/docs/module4-vla-robotics/chapter2', 'db8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro',
                component: ComponentCreator('/docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro', '9d6'),
                exact: true
              },
              {
                path: '/docs/weekly-roadmap',
                component: ComponentCreator('/docs/weekly-roadmap', '83e'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
