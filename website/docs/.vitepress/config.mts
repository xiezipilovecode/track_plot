import { defineConfig } from 'vitepress'

export default defineConfig({
  title: 'TAT Dataset',
  description: 'TunnelAutopilot-Tunnel (TAT): 61-vehicle CARLA tunnel dataset with 24K+ RGB images, instance masks, and 27K+ COCO 2D bboxes.',
  lang: 'zh-CN',
  head: [
    ['link', { rel: 'icon', href: '/favicon.ico' }],
  ],
  lastUpdated: true,
  cleanUrls: true,

  themeConfig: {
    // -- 导航栏 --
    nav: [
      { text: 'Home',       link: '/' },
      { text: 'Sensors',    link: '/sensors' },
      { text: 'Data Format', link: '/data-format' },
      { text: 'Tasks',      link: '/tasks' },
      { text: 'Download',   link: '/download' },
      { text: 'DevKit',     link: '/devkit' },
      { text: 'License',    link: '/license' },
    ],

    // -- 侧边栏 --
    sidebar: [
      {
        text: 'TAT Dataset',
        items: [
          { text: 'Home',         link: '/' },
          { text: 'Sensors',      link: '/sensors' },
          { text: 'Data Format',  link: '/data-format' },
          { text: 'Tasks',        link: '/tasks' },
          { text: 'Download',     link: '/download' },
          { text: 'DevKit',       link: '/devkit' },
          { text: 'License & Citation', link: '/license' },
        ],
      },
    ],

    // -- 页脚 --
    footer: {
      message: 'TunnelAutopilot-Tunnel (TAT) Dataset · CARLA tunnel scenario',
      copyright: '© 2026 TAT Dataset Contributors',
    },

    // -- 搜索（本地） --
    search: {
      provider: 'local',
    },

    // -- 社交链接 --
    socialLinks: [],
  },
})
