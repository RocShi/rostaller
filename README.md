# 1 特性

用于在 Linux Ubuntu 环境中一键安装 ROS 1，通过 `rosdistro` 库的最新版本（使用 `wget` 获取）或本地内置版本自动处理 `rosdep init` 和 `rosdep update` 失败的问题（域名污染），无需繁琐设置。

- 支持版本：

  - Ubuntu 16.04（Xenial） + ROS 1 Kinetic
  - Ubuntu 18.04（Bionic） + ROS 1 Melodic
  - Ubuntu 20.04（Focal） + ROS 1 Noetic

- 执行过程同时输出至终端及 log 文件

- 若连接 Key 服务器失败，请多尝试几次

# 2 用法

## 2.1 克隆仓库

```bash
git clone https://github.com/RocShi/rostaller.git
```

## 2.2 进入仓库目录

```bash
cd rostaller
```

## 2.3 为脚本添加可执行权限

```bash
chmod +x ./run.sh
```

## 2.4 运行脚本

```bash
./run.sh
```

**Enjoy ROS！**
