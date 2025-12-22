import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/auth',
    component: ComponentCreator('/auth', '4fe'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', 'f20'),
    exact: true
  },
  {
    path: '/personalization-settings',
    component: ComponentCreator('/personalization-settings', 'fc9'),
    exact: true
  },
  {
    path: '/personalize',
    component: ComponentCreator('/personalize', '6e5'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', 'e7f'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'bf8'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '115'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '376'),
            routes: [
              {
                path: '/docs/accessibility-guide',
                component: ComponentCreator('/docs/accessibility-guide', '852'),
                exact: true
              },
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
                path: '/docs/authentication-guide',
                component: ComponentCreator('/docs/authentication-guide', 'fb9'),
                exact: true
              },
              {
                path: '/docs/backup-recovery-guide',
                component: ComponentCreator('/docs/backup-recovery-guide', '530'),
                exact: true
              },
              {
                path: '/docs/capstone-project/chapter1',
                component: ComponentCreator('/docs/capstone-project/chapter1', '83d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/deployment-guide',
                component: ComponentCreator('/docs/deployment-guide', '1ec'),
                exact: true
              },
              {
                path: '/docs/integration-guide',
                component: ComponentCreator('/docs/integration-guide', '9a2'),
                exact: true
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
                path: '/docs/monitoring-alerting-guide',
                component: ComponentCreator('/docs/monitoring-alerting-guide', '646'),
                exact: true
              },
              {
                path: '/docs/performance-optimization',
                component: ComponentCreator('/docs/performance-optimization', '9e3'),
                exact: true
              },
              {
                path: '/docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro',
                component: ComponentCreator('/docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro', '9d6'),
                exact: true
              },
              {
                path: '/docs/security-guide',
                component: ComponentCreator('/docs/security-guide', '5d6'),
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
