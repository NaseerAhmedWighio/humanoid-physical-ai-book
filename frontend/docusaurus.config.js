// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';
require('dotenv').config();

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'AI-native textbook for 13-week course on humanoid robotics',
  favicon: 'img/favicon.png',

  // Set the production url of your site here
  url: 'https://humanoid-physical-ai-robotics-textb.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Humanoid Robotics Textbook', // Usually your GitHub org/user name.
  projectName: 'AI Textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/NaseerAhmedWighio/humanoid-physical-ai-book',
          remarkPlugins: [
            // Add any remark plugins if needed
          ],
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themes: [
    // Add any custom themes here
  ],



  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/favicon.png',
      navbar: {
        title: 'Humanoid AI Textbook',
        logo: {
          alt: 'Humanoid AI Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'custom-page-controls',
            position: 'right', // Will be styled to appear centered
          },
          {
            type: 'custom-search-button',
            position: 'right',
          },
          {
            type: 'custom-auth-buttons',
            position: 'right',
          },
          // {
          //   href: 'https://github.com/NaseerAhmedWighio',
          //   label: 'GitHub',
          //   position: 'right',
          // },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Intro',
                to: '/docs/intro',
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
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/NaseerAhmedWighio',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Humanoid AI Textbook. Built with Docusaurus.`,
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),

  // Add custom fields to make API URL available to components
  customFields: {
    apiBaseUrl: process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000',
  },

  // Enable environment variables for client using DefinePlugin
  plugins: [
    function(context, options) {
      return {
        name: 'webpack-define-env-plugin',
        configureWebpack(config, isServer, utils) {
          const { DefinePlugin } = require('webpack');

          return {
            plugins: [
              new DefinePlugin({
                'process.env.REACT_APP_API_BASE_URL': JSON.stringify(process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'),
              }),
            ],
          };
        },
      };
    },
  ],
};

export default config;