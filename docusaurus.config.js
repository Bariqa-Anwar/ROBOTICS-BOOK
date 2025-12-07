module.exports = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: '100% Written by Gemini 2.5 Flash CLI',
  url: 'http://localhost:3000',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'your-name', // Replace with your name or leave as is
  projectName: 'humanoid-robotics',
  themeConfig: {
    algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'YOUR_INDEX_NAME',
      contextualSearch: true,
    },
    navbar: {
      title: 'Physical AI Book',
      items: [
        {to: 'docs/intro', label: 'Home', position: 'left'},
        {to: 'docs/modules/01-ros2/00-overview', label: 'Modules', position: 'left'},
      ],
    },
    footer: {
      style: 'dark',
      copyright: '© 2025 – 100 % Gemini 2.5 Flash CLI authored',
    },
    prism: {
      theme: 'github',
      darkTheme: 'dracula',
    },
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://example.com/edit/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
