# Marvin 配置工作室

解压后，在包含 dist 的目录打开终端。已安装 Python 3 的 Windows 用户运行：

    py -m http.server 8000 --bind 127.0.0.1 --directory dist

macOS 或 Linux 使用 python3 替代 py。浏览器打开 http://localhost:8000 。

dist/index.html 包含页面与样式，dist/app.js 包含选配、编号、PDF 和邮件询价逻辑，dist/catalog.json 包含型号与选配数据，dist/marvin.webp 为产品图片。

## 3D 配置预览（2026-11-25 第二版）

- dist/viewer.js：WebGL2 渲染（PBR 材质、阴影、ACES 色调），依赖为零。
- dist/m/：网格数据。common = 底座、支撑架、控制器盒；ee = 两款力传感器、SHW5G/D405 腕部相机套件、Omni-Gripper；arm-<型号> = 双臂（按所选型号按需加载）。
- 六种型号均有独立模型：696 与 Lite 615 来自高精度 STL；809/920 由 696 按 URDF 连杆长度（上臂 0.400、前臂 0.314/0.425 m）拉伸，680/750 由 615 拉伸（上臂 0.329/0.400 m）。
- 底座/支撑架来自 marvin_frame_base_drawing260811（base_m6 + Dual_arm_mount-m6）；力传感器为按规格书尺寸绘制的示意模型。
- tools/build2.py 重新生成 dist/m（另需 Drive 上的源 STL 与 gento-docs 仓库的 static/models）。

## 语言与发布

- 页面右上角可切换中文 / EN；英文文案在 dist/i18n.js（以中文原文为键），也可用 `?lang=en` 直接打开英文版。
- 推送到 main 后，.github/workflows/pages.yml 会把 dist/ 发布到 GitHub Pages（仓库 Settings → Pages → Source 选 GitHub Actions）。
- dist/ 为完整静态网站，无需构建，可直接放到任意静态托管（如阿里云 OSS、腾讯云 COS）。
