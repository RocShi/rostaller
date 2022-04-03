## 20220403 - v0.7.1

- **[fix]：** 修复为 ROS 2 Galactic 设置源时指定的错误的镜像地址。

## 20220403 - v0.7

- **[new]：** 添加对 Ubuntu 20.04（Focal）下 ROS 2 Galactic 的支持。

## 20220402 - v0.6.2

- **[update]：** 更新本地预置的 `rosdistro` 版本：`adf8deb` → `b226ab5`；

## 20211006 - v0.6.1

- **[fix]：** 更正部分有误的终端输出信息；
- **[update]：** 更新 README 文档。

## 20211005 - v0.6

- **[new]：** 添加对 Ubuntu 14.04（Trusty）下 ROS 1 Indigo 的支持；
- **[update]：** 更新本地预置的 `rosdistro` 版本：`cb9a972` → `adf8deb`；
- **[update]：** 更新 README 文档。

## 20211003 - v0.5.1

- **[update]：** 更新 README 文档。

## 20210825 - v0.5

- **[new]：** 添加对 Ubuntu 16.04（Xenial）下 ROS 1 Kinetic 的支持；
- **[new]：** 添加对 Ubuntu 20.04（Focal）下 ROS 1 Noetic 的支持；
- **[update]：** 更新本地预置的 `rosdistro` 版本：`6a23f46` → `cb9a972`；
- **[update]：** 优化 demo 运行逻辑；
- **[update]：** 更新 README 文档。

## 20210708 - v0.4.1

- **[fix]：** 修复 `.gitignore` 忽略文件错误的问题。

## 20210708 - v0.4

- **[fix]：** 修复重复执行安装时，向 `.bashrc` 中重复添加环境配置的问题。

## 20210706 - v0.3

- **[fix]：** Git LFS 免费用户每月有带宽限制，下载量超限后导致仓库无法 `clone`，改用 `wget` 在线获取 `rosdistro` 的最新压缩包，同时本地预置一个可用版本；
- **[update]：** 更新 README 文档。

## 20210706 - v0.2

- **[new]：** 将 `source.list` 文件的源自动变更为清华源，并自动执行系统软件更新与升级，无需手动设置；
- **[fix]：** 修复首次执行脚本时无法正常运行 demo 的问题；
- **[new]：** 完成安装后，可选择是否运行 demo，默认运行；
- **[update]：** 更新 README 文档。

## 20210705 - v0.1

- **[new]：** 释放首个版本。
