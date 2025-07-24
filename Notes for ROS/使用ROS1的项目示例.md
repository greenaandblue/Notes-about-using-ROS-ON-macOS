项目链接：：@https://github.com/stoic-roboticist/kmriiwa_ros_stack
你现在已经在 Mac ARM64 上用 `--platform=linux/amd64` 成功运行了 Ubuntu 容器，并且在容器里安装好了 ROS1 Noetic。  
你想把 [kmriiwa_ros_stack](https://github.com/stoic-roboticist/kmriiwa_ros_stack) 这个项目挂载到容器里进行开发和运行。

---

## 1.挂载项目

### 方案一：直接挂载本地目录到容器（推荐）

#### 步骤

1. **在 Mac 上 clone 项目到本地**  
   在你的 Mac 终端执行：

   ```bash
   git clone https://github.com/stoic-roboticist/kmriiwa_ros_stack.git
   ```

   假设 clone 到 `~/ros1_ws/src/kmriiwa_ros_stack`

2. **启动 Docker 容器并挂载本地 ROS 工作空间**  
   例如你的本地 ROS 工作空间在 `~/ros1_ws`，用如下命令启动容器并挂载：

   ```bash
   docker run --platform=linux/amd64 -it \
     -v ~/ros1_ws:/root/ros1_ws \
     ubuntu:20.04
   ```

   这样容器内 `/root/ros1_ws` 就和你 Mac 上的 `~/ros1_ws` 同步了。

3. **在容器内编译和使用你的项目**  
   在容器内执行：

   ```bash
   cd /root/ros1_ws
   # 安装依赖
   apt update
   apt install -y python3-rosdep python3-catkin-tools git
   rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   # 编译
   catkin_make
   # source 环境
   source devel/setup.bash
   ```

   之后就可以用 ROS 命令运行你的包了。

---

### 方案二：直接在容器内 clone 项目（不挂载）

如果你不需要和 Mac 共享代码，也可以直接在容器内 clone：

```bash
cd /root
mkdir -p ros1_ws/src
cd ros1_ws/src
git clone https://github.com/stoic-roboticist/kmriiwa_ros_stack.git
# 然后同上，rosdep、catkin_make
```

---

### 方案三：写 Dockerfile 自动化（进阶）

如果你想每次都自动构建环境，可以写一个 Dockerfile，把 clone、依赖安装、编译都写进去。

---

### 总结

- 推荐用 `-v` 参数挂载本地工作空间到容器，开发体验最好。
- 只要你用的是 amd64 容器，kmriiwa_ros_stack 项目可以直接用 ROS1 Noetic 正常编译和运行。

---
## 2.安装环境（已安装可跳过）

### 2.1 安装ROS开发工具

在容器内，先确保ROS环境变量已加载（假设你已装好ROS Noetic）：

```bash
source /opt/ros/noetic/setup.bash
```

然后安装开发工具：

```bash
apt update
apt install -y python3-rosdep python3-catkin-tools
```

如果`python3-catkin-tools`找不到，可以用：

```bash
apt install -y python3-rosdep
pip3 install -U catkin_tools
```

如果`pip3`没装，先装：

```bash
apt install -y python3-pip
```

### 2.2 初始化rosdep

```bash
rosdep init
rosdep update
```

### 2.3 安装依赖

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 2.4 编译工作空间

```bash
catkin_make
```

### 2.5 加载环境

```bash
source devel/setup.bash
```

---

## 3. 项目能实现什么功能？如何测试？

项目 [kmriiwa_ros_stack](https://github.com/stoic-roboticist/kmriiwa_ros_stack) 是一个KMRIIWA机器人完整的ROS功能包，支持仿真、MoveIt、导航、可视化等。  
**你可以测试如下功能：**

### 1. Gazebo仿真

###### 你需要先启动 bringup，再启动仿真

启动空世界仿真：

```bash
roslaunch kmriiwa_gazebo kmriiwa_empty_world.launch
```

启动测试区仿真：

```bash
roslaunch kmriiwa_gazebo kmriiwa_test_zone.launch
```

### 2. 机器人Bringup

```bash
roslaunch kmriiwa_bringup kmriiwa_bringup.launch
```

### 3. MoveIt运动规划

```bash
roslaunch kmriiwa_moveit move_group.launch
```

可视化MoveIt：

```bash
roslaunch kmriiwa_vis moveit_view.launch
```

### 4. 导航功能

无地图导航：

```bash
roslaunch kmriiwa_navigation mapless_navigation.launch rviz:=true
```

建图：

```bash
roslaunch kmriiwa_navigation gmapping.launch
```

---


## 4.编译
我觉得`catkin_make`没有`catkin build`好用，推荐用 catkin build 替代 catkin_make，更灵活

## 5.CMake 报错和依赖问题

### 1. 依赖包缺失的根本解决办法

**每当遇到如下 CMake 报错：**
```
Could not find a package configuration file provided by "XXX"
```
就需要安装对应的 ROS 包，包名通常是 `ros-noetic-xxx`。

---

### 2. 你的具体操作步骤

#### A. 安装缺失依赖

你刚才缺的是 `move_base_msgs`，所以执行：

```bash
apt update
apt install -y ros-noetic-move-base-msgs
```

如果后续还报缺别的包，比如 `tf2_sensor_msgs`、`hector_gazebo`、`actionlib_msgs` 等，依次执行：

```bash
apt install -y ros-noetic-tf2-sensor-msgs
apt install -y ros-noetic-hector-gazebo-plugins ros-noetic-hector-gazebo-worlds
apt install -y ros-noetic-actionlib-msgs
```

**遇到缺什么包就装什么包！**

---

#### B. 自动补全依赖（推荐）

你可以多次执行下面命令，自动补全大部分依赖：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
如果 `rosdep` 没装，先装：

```bash
apt install -y python3-rosdep
rosdep init
rosdep update
```

---

#### C. 清理旧的 build/devel 目录（如有需要）

如果你切换过工作空间或编译方式，建议清理：

```bash
rm -rf build devel
```

---

#### D. 重新编译

```bash
catkin_make
```
或者用更推荐的 `catkin build`（需要先安装 catkin_tools）：

```bash
apt install -y python3-pip
pip3 install -U catkin_tools
catkin build
```
> 参考：[CSDN博客-编译方式的更新](https://blog.csdn.net/2301_79970562/article/details/136237507)

---

#### E. 遇到新依赖缺失怎么办？

- 只要有 `Could not find a package configuration file provided by "XXX"`，就 `apt install -y ros-noetic-xxx`
- 如果是第三方库（如 Ceres、GTSAM），按博客里的方法单独源码安装，但**不要把源码包放到 src 目录下**，否则会导致 catkin_make 报错。

---

#### 3. 总结

1. **遇到缺包就装包**，包名一般是 `ros-noetic-xxx`
2. **多用 `rosdep` 自动补全依赖**
3. **清理 build/devel 目录后重新编译**
4. **推荐用 `catkin build` 替代 `catkin_make`，更灵活**

---

#### 4. 参考资料

- [CSDN：运行ROS包时cmake报错及解决方法汇总](https://blog.csdn.net/2301_79970562/article/details/136237507)
- [CSDN：ROS1常见依赖安装与编译问题](https://blog.csdn.net/weixin_52905802/article/details/135521791)

---

## 5.会出现的错误结果：segmentation fault

你的 Gazebo GUI (`gzclient`) 报错：

```
Segmentation fault
[gazebo_gui-2] process has died [pid ..., exit code 139, ...]
```

**这就是“分段错误”，即 Segmentation fault，exit code 139。**

我gazebo和RViz都没成功，它会突然闪现出软件页面然后消失，反正用不了 : (

---

### 这是什么问题？

- **根本原因**：在 Mac ARM64 上用 Docker 跑 amd64 镜像时，Gazebo GUI（gzclient）经常因为 OpenGL/QEMU 虚拟化兼容性问题崩溃。
- 这不是你的项目、依赖、控制器或 PID 配置的问题，而是**平台本身的兼容性限制**。
- 你已经做到了 socat + XQuartz + Docker 的最佳实践，`xeyes` 能弹出说明 X11 通道没问题，但 Gazebo GUI 涉及 3D 加速，QEMU 的 OpenGL 支持在 ARM Mac 上很不稳定。

---

### 你还能做什么？

#### 1. 确认你已经安装了所有依赖

你已经装了 `ros-noetic-ros-controllers`，这没问题。

#### 2. 尝试用软件渲染（低概率成功）

可以尝试在容器内用软件渲染（Mesa），但大概率还是会崩溃：

```bash
export LIBGL_ALWAYS_INDIRECT=1
roslaunch kmriiwa_gazebo kmriiwa_empty_world.launch
```

#### 3. 只用物理仿真（无 GUI）

```bash
roslaunch kmriiwa_gazebo kmriiwa_empty_world.launch gui:=false
```
你可以用 ROS topic、RViz、rqt_image_view 等工具与仿真交互。

##### 经测试物理仿真可以正常启动。

我的物理仿真日志如下：

` [INFO] gazebo_ros_control plugin is waiting for model URDF in parameter [/kmriiwa/robot_description] on the ROS param server.`
 `[ERROR] No p gain specified for pid. Namespace: /kmriiwa/arm/gazebo_ros_control/pid_gains/kmriiwa_joint_X`
`[INFO] Loaded gazebo_ros_control.`
`[INFO] Controller Spawner: Loaded controllers: joint_state_controller, manipulator_controller`
`[INFO] Started controllers: joint_state_controller, manipulator_controller`

1. 仿真已经正常启动

- 控制器（joint_state_controller 和 manipulator_controller）都已加载并启动。

- 没有致命错误或崩溃，物理仿真（gzserver）在后台正常运行。

1. PID 报错可以暂时忽略

- 这些 [ERROR] No p gain specified for pid... 只是警告，说明你没有为每个关节配置 PID 参数。

- 这会导致机械臂控制不够精确或运动不理想，但不会影响仿真进程的正常运行。

---

##### 你现在可以做什么？

- 通过 ROS topic 与仿真交互，比如用 MoveIt 规划机械臂、用导航包让机器人运动、用 rostopic echo 查看状态等。

- 用 RViz 可视化机器人状态（如果你能打开 GUI）。

- 用 rqt_image_view、rostopic 等工具查看仿真传感器数据。
#### 4. 用 RViz 可视化机器人状态

RViz 有时比 Gazebo GUI 更容易在 X11 下显示（但也可能有 OpenGL 问题）：

```bash
rosrun rviz rviz
```

#### 5. 如需完整 GUI，建议用 x86_64 物理机或云服务器

- 只有在原生 x86_64 Linux 上，Gazebo GUI 才能100%稳定运行。

---

### 总结

- **你遇到的分段错误是 Mac ARM64 + Docker amd64 + OpenGL/QEMU 的已知兼容性问题。**
- 这不是你的项目或配置问题，**是平台限制**。
- 你可以用无 GUI 方式跑仿真，或用 RViz、rqt_image_view 等工具可视化数据。
- 如需完整 Gazebo GUI，建议换到 x86_64 物理机或云服务器。

---

## 6.正常运行会出现的结果


![[Screenshot 2025-07-16 at 10.55.43 PM.png]]

![[123.mov]]