import { themes } from 'prism-react-renderer';
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics Textbook',
  tagline: 'Learn about Physical AI, Humanoid Robotics, and ROS 2',
  url: 'https://physical-ai-and-humanoid-robotics-t-lake.vercel.app',
  baseUrl: '/',
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    hooks: {
      onBrokenMarkdownImages: 'warn', // Handle broken images gracefully
    }
  },
  favicon: 'img/logo2.png',
  organizationName: 'Hudaibkhan', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-textbook',

  // Custom fields - accessible via window.docusaurus.customFields
  customFields: {
    authBackendUrl: process.env.NEXT_PUBLIC_AUTH_BACKEND_URL || 'http://localhost:8000',
    ragBackendUrl: process.env.NEXT_PUBLIC_RAG_BACKEND_URL || 'http://localhost:8000',
    frontendUrl: process.env.NEXT_PUBLIC_FRONTEND_URL || 'http://localhost:3002',
  },

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

  scripts: [
  {
    src: "/js/env-loader.js",
    async: false,  // Load synchronously FIRST
    defer: false
  },
  {
    src: "https://unpkg.com/react@18/umd/react.production.min.js",
    async: true
  },
  {
    src: "https://unpkg.com/react-dom@18/umd/react-dom.production.min.js",
    async: true
  },
  {
    src: "/js/chatbot-widget.js",
    async: true,
    defer: true
  }
],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/logo2.png',
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
            type: 'custom-Auth',
            position: 'right',
          },
          {
            href: 'https://github.com/Hudaibkhan/Physical-AI-and-Humanoid-Robotics-Textbook',
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
                to: '/docs/capstone-project/chapter1',
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
