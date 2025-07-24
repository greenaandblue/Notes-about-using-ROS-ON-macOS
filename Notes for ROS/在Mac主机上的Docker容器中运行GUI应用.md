主要参考这篇博客： [实现Mac主机上的Docker容器中的图形界面显示（运行GUI应用）](https://www.cnblogs.com/noluye/p/11405358.html) 该blog介绍了**用 socat + XQuartz** 让 Docker 容器里的 GUI 应用在 Mac 上显示的方法。  
这个方法**比单纯用 XQuartz 更稳定**，尤其是在 Mac ARM64 + Docker amd64 镜像的场景下。

---

### Mac ARM64 + Docker amd64 + socat + XQuartz 显示 GUI 详细方案

#### 1. 安装 XQuartz 和 socat

在 Mac 终端执行：

```bash
brew install socat
```

XQuartz 官网下载安装：[https://www.xquartz.org/](https://www.xquartz.org/)

---

#### 2. 配置 XQuartz

1. 启动 XQuartz（可在 Launchpad 里点开，或用 `open -a XQuartz`）。
2. 在 XQuartz 菜单栏，依次点击：**XQuartz > Preferences > Security**，勾选  
   - “Allow connections from network clients”
3. 关闭 XQuartz。

---

#### 3. 启动 socat 转发 X11

在 Mac 终端执行（**不要关闭这个终端窗口**）：

```bash
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
```

> 这一步必须在 XQuartz 关闭后再执行，然后再重新打开 XQuartz。

---

#### 4. 重新启动 XQuartz

```bash
open -a XQuartz
```
（多点几次确保 XQuartz 启动）

---

#### 5. 查找你的本地 IP

```bash
ipconfig getifaddr en0
```
假设输出为 `192.168.0.106`，后面用你的实际 IP。

---

#### 6. 启动 Docker 容器（amd64）并设置 DISPLAY

```bash
docker run --platform=linux/amd64 -it \
  --name kmriiwa_mac_socat \
  -e DISPLAY=192.168.0.106:0 \
  -v ~/ros1_ws:/root/ros1_ws \
  osrf/ros:noetic-desktop-full
```
> 替换 `192.168.0.106` 为你的实际 IP。

---

#### 7. 容器内操作

```bash
source /opt/ros/noetic/setup.bash
cd /root/ros1_ws
catkin build
source devel/setup.bash
roslaunch kmriiwa_gazebo kmriiwa_empty_world.launch
```

---

#### 8. 测试 GUI

你可以先在容器内测试一个简单的 GUI 程序，比如：

```bash
apt update
apt install -y x11-apps
xeyes
```
如果 `xeyes` 能在 Mac 上弹出窗口，说明 socat + XQuartz 配置成功，Gazebo GUI 也有希望能显示。

---

#### 9. 注意事项

- **socat 进程必须一直开着**，否则 X11 转发会断。
- 每次新开容器都要设置正确的 `DISPLAY` 环境变量。
- 如果遇到 `XIO:  fatal IO error 11`，多试几次，或重启 socat/XQuartz。
- 由于 Mac ARM64 + Docker amd64 的 OpenGL 兼容性，Gazebo GUI 依然有概率崩溃，但 socat 方案比单纯 XQuartz 更稳定。

---

#### 10. 参考原文

- [实现Mac主机上的Docker容器中的图形界面显示（运行GUI应用）](https://www.cnblogs.com/noluye/p/11405358.html)

---

#### 总结

- socat + XQuartz 是 Mac 上 Docker GUI 的最佳实践之一。
- 按上述步骤操作，能大幅提升 GUI 应用（如 Gazebo、RViz）在 Mac ARM64 + Docker amd64 下的显示成功率。
- 但 Gazebo GUI 依然可能因 OpenGL/QEMU 兼容性崩溃，这属于平台限制。

---

![[14_1753352596.mp4]]
![[Screenshot 2025-07-16 at 9.39.59 PM.png]]
![[Screenshot 2025-07-10 at 1.49.34 AM.png]]
![[15_1753352598.mp4]]