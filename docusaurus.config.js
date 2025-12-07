import { themes } from 'prism-react-renderer';
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics Textbook',
  tagline: 'Learn about Physical AI, Humanoid Robotics, and ROS 2',
  url: 'https://your-docusaurus-site.com',
  baseUrl: '/',
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    hooks: {
      onBrokenMarkdownImages: 'warn', // Handle broken images gracefully
    }
  },
  favicon: 'img/favicon.ico',
  organizationName: 'your-org',
  projectName: 'physical-ai-humanoid-robotics-textbook',

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Robotics',
        logo: {
          alt: 'My project logo',
          src: 'img/logo2.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Physical AI & Humanoid Robotics Textbook',
                to: '/',
              },
              {
                label: 'Module 1: ROS 2 Nervous System',
                to: '/docs/module1-ros2-nervous-system/',
              },
              {
                label: 'Module 2: Digital Twin Simulation',
                to: '/docs/module2-digital-twin-simulation/',
              },
              {
                label: 'Module 3: AI Brain (NVIDIA Isaac)',
                to: '/docs/module3-ai-brain-isaac/',
              },
              {
                label: 'Module 4: Vision-Language-Action Robotics',
                to: '/docs/module4-vla-robotics/',
              },
            ],
          },
          {
            title: 'Sections',
            items: [
              {
                label: 'Capstone Project',
                to: '/docs/capstone-project/',
              },
              {
                label: 'Additional Materials',
                to: '/docs/additional-materials/',
              },
              {
                label: 'Weekly Roadmap',
                to: '/docs/weekly-roadmap/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai-and-humanoid-robotics-textbook',
              },
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: lightTheme,
        darkTheme: darkTheme,
      },
    }),
};

module.exports = config;
