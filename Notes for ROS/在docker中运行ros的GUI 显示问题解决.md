#### **图形界面（GUI）应用程序问题 (Turtlesim 和 RViz)**

- **问题描述（Turtlesim）：** `qt.qpa.xcb: could not connect to display` 和 `Aborted`。
    
- **问题描述（RViz）：**
    
    - `[rospack] Error: package 'rviz' not found` (首次尝试)。
        
    - `qt.qpa.xcb: could not connect to display ...` 和 `Unable to create the rendering window after 100 tries.` (安装 RViz 后)。
        
    - `libGL error: No matching fbConfigs or visuals found` 和 `libGL error: failed to load driver: swrast` (尝试 `glxgears` 验证时)。
        
    - `bash: glxgears: command not found` (在新容器中未安装 `mesa-utils`)。
        
- **问题原因：**
    
    1. **RViz 未安装：** 初始 `osrf/ros:noetic-robot-ros-base` 镜像不包含 RViz。
        
    2. **X11 转发未配置：** Docker 容器默认无法访问 Mac 的图形显示系统。需要 XQuartz 和特定的 Docker 运行参数。
        
    3. **`DISPLAY` 环境变量格式错误：** Mac 上的 `DISPLAY` 可能是 Unix socket 格式（`/private/tmp/...`），而 Docker 容器需要 TCP/IP 格式（`host.docker.internal:0`）。
        
    4. **OpenGL 软件渲染库问题：** 容器内部缺少或未能正确加载 Mesa（软件 OpenGL 驱动），导致 3D 应用（RViz）无法创建渲染上下文。即使 X11 转发成功，也无法渲染。
        
    5. **`mesa-utils` 未安装：** `glxgears` 命令在容器内找不到，因为它属于 `mesa-utils` 包。
        
- **解决方案 (分步排查与最终方案)：**
    
    **阶段一：解决 RViz 未找到和基础 X11 转发**
    
    1. **在 Mac 上安装 XQuartz 并重启 Mac。**
        
    2. **启动 XQuartz，并在其偏好设置 -> 安全中勾选“允许从网络客户端连接”。**
        
    3. **在 Mac 终端中运行 `xhost +`**，确保输出 `access control disabled...`。
        
    4. **停止并移除旧容器（例如 `test2`），启动新的 `ros_gui_test_final` 容器，并确保 Docker 命令包含所有 X11 转发参数和正确的 `DISPLAY` 变量：**
        
        Bash
        
        ```
        # 在 Mac 终端
        docker stop <旧容器名，如test2> && docker rm <旧容器名，如test2> # 如果有旧的GUI测试容器，也要停止和移除
        docker run -it --name ros_gui_test_final --network host \
        -e DISPLAY=host.docker.internal:0 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        osrf/ros:noetic-desktop-full bash
        ```
        
        - **注意：** 使用 `osrf/ros:noetic-desktop-full` 镜像会包含 RViz，省去了单独安装 RViz 的步骤。
            
    5. **进入新容器后，验证 `DISPLAY` 变量：** `echo $DISPLAY`，确保其为 `host.docker.internal:0`。
        
    
    **阶段二：解决 OpenGL 软件渲染问题**
    
    6. **在新容器内部安装所有必要的 Mesa 和 X11 库：**
        
        Bash
        
        ```
        # 在 ros_gui_test_final 容器内部
        sudo apt update
        sudo apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa libegl1-mesa libgbm1 libosmesa6 xorg-dev libx11-dev libxext-dev libxrandr-dev libxi-dev libxrender-dev
        ```
        
    7. **设置 `LIBGL_ALWAYS_SOFTWARE=1` 环境变量：**
        
        - **方法一 (临时测试)：** 在容器终端中运行 `export LIBGL_ALWAYS_SOFTWARE=1`。
            
        - **方法二 (永久性，推荐)：** 将 `echo 'export LIBGL_ALWAYS_SOFTWARE=1' | sudo tee -a /root/.bashrc > /dev/null` 运行在容器中，然后 `exit` 容器并 `docker exec -it ros_gui_test_final bash` 重新进入，验证 `echo $LIBGL_ALWAYS_SOFTWARE` 是否为 `1`。
            
    8. **验证 `glxgears`：** 在容器中运行 `glxgears`。
        
        - **如果出现 `command not found`：** 再次确认 `sudo apt install -y mesa-utils` 已经成功运行。
            
        - **预期：** 应该会看到带有旋转齿轮的窗口弹出。
            
    
    **阶段三：最终尝试 RViz (如果上述方法仍失败，则考虑 VNC)**
    
    9. **如果 `glxgears` 能够显示，再次尝试运行 `rosrun rviz rviz`。**
        
    10. **如果所有直接 X11 转发方法（包括上述所有步骤）都失败，并且 `glxgears` 仍然报告 `libGL error: failed to load driver: swrast` 等问题，则使用 VNC 解决方案：**
        
        - **原因：** 这通常意味着 Docker/macOS/XQuartz/OpenGL 驱动之间存在深层兼容性问题，直接 X11 转发难以解决。
            
        - **VNC 解决方案：**
            
            1. **停止并移除所有旧的 ROS GUI 容器。**
                
            2. **在 Mac 终端中运行预配置 VNC 的 ROS 镜像：**
                
                Bash
                
                ```
                docker run -it --rm --name ros_noetic_vnc -p 6080:80 -p 5900:5900 \
                robostack/ros-noetic-desktop-vnc
                ```
                
            3. **从 Mac 连接 VNC 桌面：**
                
                - **通过浏览器：** 访问 `http://localhost:6080`，输入 VNC 密码（通常是 `ros` 或 `robot`）。
                    
                - **通过 VNC 客户端：** 连接 `localhost:5900`。
                    
            4. **在 VNC 桌面内部的终端中：** 启动 `roscore`，然后 `rosrun turtlesim turtlesim_node` 和 `rosrun rviz rviz`。图形界面将在这个 VNC 窗口内显示。
                

---
![[Screenshot 2025-07-10 at 1.35.00 AM.png]]
![[Screenshot 2025-07-10 at 1.55.58 AM.png]]