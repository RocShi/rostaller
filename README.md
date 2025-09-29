# 1 用途

在 Linux Ubuntu 环境中一键安装 ROS 1 和 ROS 2。支持版本：

| Ubuntu 版本 | 代号 | ROS 1 | ROS 2 |
|------------|------|--------|--------|
| 24.04 | Noble | - | Jazzy |
| 22.04 | Jammy | - | Humble |
| 20.04 | Focal | Noetic | Galactic |
| 18.04 | Bionic | Melodic | - |
| 16.04 | Xenial | Kinetic | - |
| 14.04 | Trusty | Indigo | - |

# 2 特性

- 通过 [rosdistro](https://github.com/ros/rosdistro) 库自动处理安装 ROS 1 时存在的 `rosdep init` 和 `rosdep update` 失败的问题（域名污染），无需繁琐设置

- 执行过程同时输出至终端及 log 文件

- 若连接 Key 服务器失败，请多尝试几次

- 变更历史请参考 [CHANGELOG](CHANGELOG.md)

# 3 用法

## 3.1 克隆仓库（最新提交）

```bash
git clone --depth 1 https://github.com/RocShi/rostaller.git
```

## 3.2 进入仓库目录，并为脚本添加可执行权限

```bash
cd rostaller && chmod +x ./run.sh
```

## 3.3 运行脚本

```bash
./run.sh
```

**Enjoy ROS！**
