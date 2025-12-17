import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'ROS 2 and NVIDIA Isaac Sim Capstone Guide',
  url: 'https://bariqa-anwar.github.io',
  baseUrl: '/', 

  onBrokenLinks: 'ignore', // Changed to ignore temporarily to help you get the site up
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  organizationName: 'bariqa-anwar',
  projectName: 'humanoid-robotics',
  trailingSlash: false,

  // --- MERMAID SUPPORT START ---
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
  // --- MERMAID SUPPORT END ---

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
          editUrl: 'https://github.com/bariqa-anwar/humanoid-robotics/edit/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          { to: '/docs/intro', label: 'Home', position: 'left' },
          { 
            type: 'docSidebar',
            sidebarId: 'mySidebar', 
            label: 'Modules', 
            position: 'left' 
          },
          {
            href: 'https://github.com/bariqa-anwar/humanoid-robotics',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discord.gg/your-invite',
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Physical AI Book – Authored by Gemini 2.5 Flash`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: [
          'bash', 'python', 'cpp', 'cmake', 'yaml', 'docker', 'rust'
        ],
      },
    }),
};

module.exports = config;