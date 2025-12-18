import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'ROS 2 and NVIDIA Isaac Sim Capstone Guide',
  url: 'https://bariqa-anwar.github.io',
  baseUrl: '/', 

  onBrokenLinks: 'ignore', 
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  organizationName: 'bariqa-anwar',
  projectName: 'humanoid-robotics',
  trailingSlash: false,

  // --- MERMAID SUPPORT ---
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Webpack Config Fix for Node.js Modules
  plugins: [
    () => ({
      name: 'webpack-config-fix',
      configureWebpack() {
        return {
          resolve: {
            fallback: {
              fs: false,
              path: false,
              os: false,
              net: false,
              tls: false,
              child_process: false,
              stream: false,
              zlib: false,
              util: false,
              assert: false,
              readline: false,
              http: false,
              https: false,
              url: false,
              process: false,
              crypto: false,
              dns: false,
              module: false,
            },
          },
        };
      },
    }),
  ],

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
        // Removed broken logo config to only show Title text
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
        copyright: `© ${new Date().getFullYear()} Physical AI Book - Authored by Gemini 3 Flash`,
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