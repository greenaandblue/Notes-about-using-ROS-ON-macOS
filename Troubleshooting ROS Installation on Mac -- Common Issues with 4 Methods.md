在mac上使用以下4种方法安装ROS遇到的问题汇总
1. 按照官方的程序安装
2. 使用鱼香ros脚本安装（recommend）
3. 使用docker容器配置
4. 虚拟机安装Ubuntu配置

---


### 1. 按照官方的程序安装

我尝试了很多次，最后也没有配置成功。我觉得问题可能不再是简单的 Python 版本冲突或工具缺失，而是更深层次的 C++ 编译环境问题、特定的库依赖问题，或者与 macOS 本身的一些编译特性有关。

---

**如果您也在官方安装过程中屡次碰壁，我强烈建议您：**

- **深入查看 `launch_testing_ament_cmake` 的详细日志**，这通常能揭示构建过程中的关键瓶颈。
    
- **仔细检查任何一个中止的 C++ 包（例如 `fastcdr`）的详细日志**，这能帮助您定位是哪个具体的C++组件出了问题。
    
- 考虑**尝试修改Mac的源代码**，尽管这听起来有些复杂，但这有时是解决顽固编译问题的唯一途径。

---

我的具体问题主要围绕着**Python环境冲突**、**构建工具缺失或配置不当**，以及**网络依赖下载失败**这三个方面。

#### 1. 问题：`colcon build` 报错 `launch_testing package not found`

- **问题内容：** 最初尝试使用 `colcon build` 构建 ROS 2 时，`colcon` 报告 `1 package failed: launch_testing_ament_cmake`，并且在 `launch_testing_ament_cmake` 包的日志中，详细错误是 `CMake Error at CMakeLists.txt:24 (message): launch_testing package not found`。同时，还有多个 C++ 包（如 `fastcdr`、`iceoryx_posh` 等）显示 `aborted`，并且大量包 `had stderr output` 或 `not processed`。
    
- **根本原因：**
    
    - **Python 版本冲突：** 这是最核心的原因。您的 macOS 系统中同时安装了 **Python 3.12 (来自 `/Library/Frameworks/Python.framework`)** 和 **Python 3.11 (通过 Homebrew 安装在 `/opt/homebrew`)**。`CMake` 在构建过程中错误地优先找到了并使用了 Python 3.12，而 ROS 2 Rolling 版本当时主要支持 Python 3.8 到 3.11，不兼容 Python 3.12 中的某些变化（特别是 `distutils` 模块的移除）。
        
    - `launch_testing` 是一个 Python 包，`launch_testing_ament_cmake` 是一个 CMake 包，它需要依赖 `launch_testing`。由于 `CMake` 使用了不兼容的 Python 3.12，导致它无法正确找到或使用 `launch_testing` 包，从而导致 `launch_testing_ament_cmake` 构建失败。
        
    - C++ 包的 `aborted` 错误通常是由于 Python 环境问题引起的 `rosdep install` 不完全，或者后续依赖关系链中断。
        
- **解决方案：**
    
    1. **诊断 Python 环境：** 通过 `echo $PATH`、`which python3`、`python3 --version` 来确认系统默认使用的 Python 版本。我们发现 `PATH` 中 `/Library/Frameworks/Python.framework/Versions/3.12/bin` 路径的存在导致了冲突。
        
    2. **强制指定 `PYTHON_EXECUTABLE` (初步尝试，未彻底解决)：** 尝试在 `colcon build` 命令中添加 `-DPYTHON_EXECUTABLE=/opt/homebrew/opt/python@3.11/libexec/bin/python3.11` 参数，试图强制 `CMake` 使用 Python 3.11。
        
        - **结果：** `CMake` 日志显示它确实尝试了我们指定的 Python 3.11，但仍然报告 `CMake Warning: Manually-specified variables were not used by the project: PYTHON_EXECUTABLE`，并最终还是找到了 Python 3.12。这表明手动参数无法完全覆盖 `CMake` 自身的查找逻辑。
            
    3. **彻底卸载 Python 3.12 Framework：** 鉴于手动指定无效，我们判断 `/Library/Frameworks/Python.framework/Versions/3.12` 的存在是根本祸源。我们执行了以下步骤：
        
        - 关闭所有 Python 相关应用。
            
        - 从 `/Applications` 文件夹删除 "Python 3.12"。
            
        - 使用 `sudo rm -rf /Library/Frameworks/Python.framework/Versions/3.12` 命令彻底删除 Python 3.12 的 Framework 文件。**强调了此步骤的危险性，需要仔细核对路径。**
            
        - 清理 `~/.zshrc` 文件，删除所有旧的 `PATH` 设置，并精确地设置 `PATH`，将 Homebrew Python 3.11 的路径放在最前面，确保系统默认使用它。
            
        - 重启终端并再次验证 `echo $PATH`、`which python3`、`which pip3`、`which vcs`、`which colcon`，确认它们都指向 Homebrew 的 Python 3.11。
            
    4. **降低 `setuptools` 版本 (临时解决方案)：** 针对 `distutils` 和 `setuptools` 相关的错误，推荐将 `setuptools` 降级到兼容版本，例如 `python3 -m pip install setuptools==65.5.1`。
        
    5. **清理工作空间并重试构建：** 在解决了 Python 环境问题后，每次尝试构建前都执行 `rm -rf build install log` 来确保一个干净的构建环境，然后运行 `colcon build --symlink-install --packages-skip-by-dep python_qt_binding`。
        

#### 2. 问题：`vcs` 命令找不到 (`zsh: command not found: vcs`)

- **问题内容：** 在执行 `vcs import` 命令时，终端提示 `zsh: command not found: vcs`。
    
- **根本原因：**
    
    - `vcs` (vcstool) 是一个 Python 包，不是 Homebrew 的一个直接公式。
        
    - 尽管之前 `vcs` 曾被成功找到并位于 `PATH` 中，但它可能在某种清理操作或环境加载问题中丢失了符号链接或可执行性。
        
    - 当终端启动时 `~/.zshrc` 文件没有正确加载，或者 `PATH` 环境变量被意外修改，导致 `/opt/homebrew/bin` 不在其中，或者虽然在其中，但 `vcs` 文件本身已不存在或不可执行。
        
- **解决方案：**
    
    1. **诊断 `PATH` 和 `vcs` 状态：** 运行 `echo $PATH` 和 `which vcs` 来确认当前终端会话的 `PATH` 内容以及 `vcs` 的查找状态。发现 `PATH` 包含 Homebrew 路径但 `vcs` 仍未找到，表明文件本身有问题。
        
    2. **通过 `pip` 重新安装 `vcstool`：** 由于 `vcstool` 是一个 Python 包，我们使用 `python3 -m pip install -U vcstool` 命令将其安装到 Homebrew 的 Python 3.11 环境中。
        
    3. **验证 `vcs` 安装：** 运行 `which vcs` 和 `vcs --version` 确认 `vcs` 已经可以被找到并正常运行。
        

---

#### 3. 问题：`rosdep` 命令找不到 (`zsh: command not found: rosdep`)

- **问题内容：** 在执行 `rosdep update` 或 `rosdep install` 命令时，终端提示 `zsh: command not found: rosdep`。
    
- **根本原因：**
    
    - 与 `vcs` 类似，`rosdep` 也是一个 Python 包，不是 Homebrew 的一个直接公式。
        
    - 它可能因为类似的原因（环境加载问题、符号链接丢失或文件本身被影响）而无法被当前终端找到。
    
- **解决方案：**
    
    1. **诊断 `PATH` 和 `rosdep` 状态：** 运行 `echo $PATH` 和 `which rosdep` 来确认。发现 `PATH` 包含 Homebrew 路径但 `rosdep` 仍未找到。
        
    2. **通过 `pip` 重新安装 `rosdep`：** 使用 `python3 -m pip install -U rosdep` 命令将其安装到 Homebrew 的 Python 3.11 环境中。
        
    3. **验证 `rosdep` 安装：** 运行 `which rosdep` 和 `rosdep --version` 确认 `rosdep` 已经可以被找到并正常运行。
    

#### 4. 问题：`vcs import` 报错 `urlopen error [Errno 8] nodename nor servname provided, or not known`

- **问题内容：** 在执行 `vcs import` 命令下载 ROS 2 仓库时，程序报错 `urlopen error [Errno 8] nodename nor servname provided, or not known`。
    
- **根本原因：**
    
    - 这是一个网络连接错误，具体是 **DNS (Domain Name System) 解析问题**。您的 Mac 无法将 `raw.githubusercontent.com` 这个域名解析成对应的 IP 地址。
        
    - 常见原因包括：网络断开、Wi-Fi/路由器故障、DNS 服务器配置错误、防火墙或代理阻挡。
        
     - 我的原因是我使用的是VPN，连接不太稳定，开全局会好一些
    
- **解决方案：**
    
    1. **检查网络连接：** 确认能否正常访问其他网站（如 `google.com`、`github.com`）。
        
    2. **诊断 DNS 解析：** 在终端运行 `ping raw.githubusercontent.com`。如果提示“未知主机”或类似错误，则确认是 DNS 问题。
        
    3. **更改 DNS 服务器设置：**
        
        - 进入 macOS 的 **系统设置 (System Settings) -> 网络 (Network)**。
            
        - 选择当前活跃的网络连接（Wi-Fi 或以太网）。
            
        - 点击 **“详细信息...” (Details...)**。
            
        - 进入 **“DNS”** 标签页。
            
        - 添加可靠的公共 DNS 服务器，例如 **Google DNS (8.8.8.8, 8.8.4.4)** 或 **Cloudflare DNS (1.1.1.1, 1.0.0.1)**。
            
        - **（可选）** 移除或禁用旧的 DNS 服务器，只保留公共 DNS。
            
        - 应用设置。
            
    4. **重新运行 `vcs import`：** 确保网络稳定后，再次执行 `vcs import`。

---

#### 5. 问题：`source ~/ros2_rolling/install/setup.zsh` 报错 `not found: ...ament_index-argcomplete.zsh`

- **问题内容：** 在尝试激活 ROS 2 环境时，终端提示 `not found: "/Users/huihang/ros2_rolling/install/ament_index_python/share/ament_index_python/environment/ament_index-argcomplete.zsh"`。
    
- **根本原因：**
    
    - 这个错误表明 `~/ros2_rolling/install` 目录下的某些文件（特别是 `ament_index_python` 包应该安装的文件）是**缺失**的。
        
    - `source setup.zsh` 命令是用来**激活已经成功构建的 ROS 2 环境**的。它会去查找 `install` 目录下由各个包生成的配置文件。
        
    - 如果之前的 `colcon build` 尝试没有完全成功完成（即使摘要显示部分包 `Finished`），那么 `install` 目录可能是不完整或损坏的。
        
- **解决方案：**
    
    1. **理解构建顺序：** 明确 `source setup.zsh` 应该在 `colcon build` 成功完成后再执行。在构建前或构建过程中尝试 `source` 一个不完整的环境会导致错误。
        
    2. **专注于 `colcon build` 的成功：** 这个错误本身不是导致 `colcon build` 失败的原因，而是 `colcon build` 失败（或不完整）的结果。解决这个问题的核心是让 `colcon build` 成功完成。
        
    3. **在 `colcon build` 之前，始终执行 `rm -rf build install log`** 来确保一个干净的构建环境。

---




### 2. 使用鱼香ROS脚本安装

鱼香ros还是很好用的，但是有时候遇到一些问题，可能需要重装系统。
#### 鱼香 ROS

“鱼香 ROS”通常指的是 [fishros.com](https://fishros.com/) 网站提供的一系列**简化 ROS 安装和使用的工具及教程**。

- **安装方式：** 鱼香 ROS **主要提供基于 Ubuntu 系统的 ROS 一键安装脚本**，目标是简化在 **Ubuntu 系统**上的 ROS 安装过程，解决依赖和源配置等问题。它并**不提供 Docker 配置**，因为它本身就是为了简化在原生 Linux 系统上的安装。
    
- **优缺点 (基于在 Ubuntu 上的表现)：**
    
    - **优点：**
        
        - **安装简便：** 极大地简化了在 Ubuntu 上安装 ROS 的过程，通常只需要几个命令即可完成。
            
        - **中文支持：** 提供详尽的中文教程和排错指南，对国内用户非常友好。
            
        - **常用工具预配置：** 可能会预先配置好一些常用的 ROS 工具和环境变量。
            
    - **缺点：**
        
        - **依赖特定脚本：** 安装过程依赖于鱼香 ROS 团队维护的脚本，如果脚本更新不及时或与最新系统有兼容性问题，可能会遇到问题。
            
        - **缺乏透明度：** 对于不熟悉 ROS 的用户，脚本背后做了什么可能不清楚，一旦出问题较难手动排查。

### 3.使用docker容器配置

#### 1. 问题：GPG 密钥过期，导致 apt update 和 apt install 无法使用

#### 报错信息：

```
W: GPG error: ... EXPKEYSIG ...
E: The repository is not signed.
```

####  根本原因：

ROS 官方的软件源签名公钥已过期，而 APT 要求每个软件源都必须有有效签名才能使用。你系统中使用的是旧的 `ros1-latest-archive-keyring.gpg`。

####  解决方法：

手动更新 GPG 密钥并重新设置 source list：

```bash
apt update && apt install curl gnupg2 lsb-release -y
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
  > /etc/apt/sources.list.d/ros-latest.list
apt update
```

---

#### 2. 问题：Signed-By 冲突，导致 apt update 无法执行

#### 报错信息：

```
E: Conflicting values set for option Signed-By regarding source ...
```

#### 根本原因：

你有两个 ROS 软件源配置文件：

- `/etc/apt/sources.list.d/ros-latest.list`
    
- `/etc/apt/sources.list.d/ros1-latest.list`
    

它们都指向同一个 ROS 源地址，但使用了**不同的 GPG 公钥路径**，APT 无法确定该信任哪一个，发生冲突。

#### 解决方法：

删除或备份其中一个文件（推荐保留 `ros-latest.list`）：

```bash
mv /etc/apt/sources.list.d/ros1-latest.list /root/ros1-latest.list.bak
apt update
```

---



### 4.虚拟机安装Ubuntu配置

#### 虚拟机安装 Ubuntu 配置

这是指你在 Mac 上通过 VMware (或其他虚拟机软件如 Parallels Desktop、UTM) 创建一个虚拟机，并在虚拟机中安装完整的 Ubuntu 操作系统，然后在 Ubuntu 中按照 ROS 官方教程或鱼香 ROS 的脚本来安装 ROS。

- **安装方式：** 首先是**虚拟化层**，其次是**在虚拟机内的 Ubuntu 系统上安装 ROS**。
    
- **优缺点 (基于在 Mac 上虚拟化运行)：**
    
    - **优点：**
        
        - **ROS 最佳兼容性：** Ubuntu 是 ROS 的主要开发平台，所有 ROS 包、驱动和工具都为其优化，**RViz 在这里运行的稳定性最高**。
            
        - **完整的 Linux 环境：** 提供一个标准、隔离的 Linux 开发环境，不会干扰你的 macOS 主系统。
            
        - **广泛的社区支持：** 任何 ROS 问题在 Ubuntu 环境下都更容易找到解决方案和社区支持。
            
        - **硬件加速 (有限)：** 现代虚拟机软件（如 VMware Fusion、Parallels Desktop）在 M 芯片 Mac 上已经能为 ARM64 Ubuntu 提供不错的虚拟化图形加速，RViz 性能通常可以接受。
            
    - **缺点：**
        
        - **性能开销：** 虚拟机需要分配独立的 CPU、内存和磁盘空间，会额外占用 Mac 资源。
            
        - **资源消耗：** 运行虚拟机本身会增加 Mac 的功耗和发热。
            
        - **初始设置：** 首次安装虚拟机和 Ubuntu 系统需要一些时间，但通常是一次性的投入。