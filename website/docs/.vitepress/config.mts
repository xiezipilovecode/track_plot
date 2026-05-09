import { defineConfig } from 'vitepress'

export default defineConfig({
  title: 'TAT Dataset',
  description: 'TunnelAutopilot-Tunnel (TAT)：包含 61 辆车、24K+ RGB 图像、实例掩码及 27K+ COCO 二维边界框的 CARLA 隧道数据集。',
  lang: 'zh-CN',
  head: [
    ['link', { rel: 'icon', href: '/favicon.ico' }],
  ],
  lastUpdated: true,
  cleanUrls: true,

  themeConfig: {
    // -- 导航栏 --
    nav: [
      { text: '首页',       link: '/' },
      { text: '传感器',    link: '/sensors' },
      { text: '数据格式', link: '/data-format' },
      { text: '任务',      link: '/tasks' },
      { text: '下载',   link: '/download' },
      { text: '开发工具',     link: '/devkit' },
      { text: '许可与引用',    link: '/license' },
    ],

    // -- 侧边栏 --
    sidebar: [
      {
        text: 'TAT Dataset',
        items: [
          { text: '首页',         link: '/' },
          { text: '传感器',      link: '/sensors' },
          { text: '数据格式',  link: '/data-format' },
          { text: '任务',        link: '/tasks' },
          { text: '下载',     link: '/download' },
          { text: '开发工具',       link: '/devkit' },
          { text: '许可与引用', link: '/license' },
        ],
      },
    ],

    // -- 页脚 --
    footer: {
      message: 'TunnelAutopilot-Tunnel (TAT) 数据集 · CARLA 隧道场景',
      copyright: '© 2026 TAT 数据集贡献者',
    },

    // -- 搜索（本地） --
    search: {
      provider: 'local',
    },

    // -- 社交链接 --
    socialLinks: [],
  },
})
