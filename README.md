# 1 特性

- 用于一键安装 ROS Melodic，自动处理 `rosdep init` 和 `rosdep update` 失败的问题，自动尝试获取最新的 `YAML` 文件

- 执行过程同时输出至终端及 log 文件

- 若连接 Key 服务器失败，可多尝试几次

- `rosdistro.tar` 文件超过 100 MB，使用 Git Large File Storage (LFS) 进行托管，因此，在 `clone` 本仓库前请务必确保已安装 Git LFS（参考下文）。若在安装 Git LFS 前已 `clone` 仓库，则可在完成 Git LFS 安装后在仓库中进行 LFS 拉取操作以完成 `rosdistro.tar` 的同步：

  ```bash
  git lfs pull
  ```

# 2 用法

## 2.1 安装 Git LFS

### Windows

直接[官网](https://git-lfs.github.com/)下载二进制文件进行安装即可

### Ubuntu

- 安装 `curl`

  ```bash
  sudo apt install curl
  ```

- 请求并执行 Git LFS 前置脚本

  ```bash
  curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
  ```

- 安装 Git LFS

  ```bash
  sudo apt install git-lfs
  ```

- 为当前用户配置 Git LFS（每个用户只需执行一次）

  ```bash
  git lfs install
  ```

## 2.2 克隆仓库

```bash
git clone https://github.com/RocShi/rostaller.git
```

## 2.3 运行脚本

```bash
chmod +x ./run.sh

./run.sh
```

**Enjoy ROS！**
