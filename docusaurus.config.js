import { themes as prismThemes } from 'prism-react-renderer';

// @ts-check
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: '100% Written by Gemini 2.5 Flash CLI',
  url: 'https://bariqa-anwar.github.io',
  baseUrl: '/humanoid-robotics/',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment config
  organizationName: 'bariqa-anwar',
  projectName: 'humanoid-robotics',
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  themeConfig: {
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        { to: '/docs/intro', label: 'Home', position: 'left' },
        { to: '/docs/modules/01-ros2/00-overview', label: 'Modules', position: 'left' },
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
      copyright: `© ${new Date().getFullYear()} – 100% authored by Gemini 2.5 Flash CLI`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: [
        'bash',
        'python',
        'cpp',
        'cmake',
        'yaml',
        'docker',
        'rust',
        'matlab',
      ],
    },

    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
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
};

module.exports = config;