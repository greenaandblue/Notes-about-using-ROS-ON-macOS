s####  1. 查找和命名容器

- 用 `docker ps` 查看正在运行的容器（名字或 ID）
    
- 用以下命令进入容器：
    
    ```bash
    docker exec -it 容器名 bash
    ```
    

#####  建议：

启动容器时加 `--name`，例如：

```bash
docker run -it --name ros_container ros:noetic bash
```

这样以后就能用名字快速进入：

```bash
docker exec -it ros_container bash
```

#### 2. 如何运行节点

在尝试运行一个 ROS 节点前（如 `talker.py`）要先运行 `roscore`，这样节点才可以注册到 ROS Master。

##### 正确流程：


1. 进入容器：

	 ```bash
	 docker exec -it ros_container bash
	 ```
	 
2. 在容器里先运行：
    
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```
    
3. 再开另一个终端进入同一个容器，运行节点：
    
    ```bash
    source /opt/ros/noetic/setup.bash
    rosrun rospy_tutorials talker.py
    ```

#### 3. 如何启动程序

##### 1. 检查 Docker 是否运行中

```bash
docker --version
```

如果这个命令输出了 Docker 的版本，说明 Docker 正常安装；你还可以检查服务是否在运行：

```bash
docker info
```

---

#####  2. 查看已有的镜像（可选）

```bash
docker images
```

如果你已经拉取过镜像，比如 `ubuntu`、`python`、`ros` 等，它们会列在这里。

---

##### 3. 运行 Docker 容器

格式：

```bash
docker run [OPTIONS] IMAGE_NAME [COMMAND]
```

举例：

##### - 运行一个交互式 Ubuntu 容器：

```bash
docker run -it ubuntu bash
```

##### - 如果你用的是 ROS（如 `ros:noetic`）：

```bash
docker run -it ros:noetic bash
```

##### - 给容器挂载当前目录（常用于开发）：

```bash
docker run -it -v $(pwd):/workspace ubuntu bash
```

---

##### 查看正在运行的容器（可选）

```bash
docker ps
```

---

##### 5. 查看所有容器（包括退出的）

```bash
docker ps -a
```

---

如果你想“进入”一个已经运行的容器（不是新开一个），你可以先用 `docker ps` 找到容器 ID，然后：

```bash
docker exec -it <容器ID或名字> bash
```

---

