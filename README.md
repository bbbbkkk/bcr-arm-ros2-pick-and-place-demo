# BCR Arm ROS2 Pick and Place Demo

本项目是一个基于 **ROS2 Humble + MoveIt2 + Gazebo/Ignition + ros2_control** 的机械臂抓取与放置仿真项目。

项目实现了一个完整的 Pick and Place 流程：

1. 在 Gazebo 中加载机械臂、桌面、木块、障碍物和放置区域
2. 通过 MoveIt2 进行机械臂运动规划
3. 通过 ros2_control 控制机械臂关节和夹爪
4. 使用 Gazebo 物理接触实现夹爪抓取
5. 机械臂抓取木块、抬起、绕过障碍物、移动到放置区域并释放
6. 使用 Docker 封装 ROS2 / MoveIt2 / Gazebo 环境，方便别人快速复现

当前版本使用的是 **物理抓取**，不是 Gazebo attach / detach 强制绑定。

---

## 1. 项目环境

推荐宿主机环境：

```text
Ubuntu 22.04
Docker
Docker Compose
X11 图形显示
```

检查 Docker 是否可用：

```bash
docker --version
docker compose version
```

本项目使用的 Docker 镜像：

```text
carson0301/bcr_arm:humble-dev
```

镜像中已经包含：

```text
ROS2 Humble
MoveIt2
Gazebo / Ignition
ros2_control
ros2_controllers
RViz2
colcon
xacro
常用编译工具
```

---

## 2. 拉取项目代码

```bash
git clone https://github.com/bbbbkkk/bcr-arm-ros2-pick-and-place-demo.git
cd bcr-arm-ros2-pick-and-place-demo
```

查看项目结构：

```bash
ls
```

主要目录如下：

```text
.
├── src/
│   ├── bcr_arm_description/      # 机械臂 URDF / Xacro / RViz 配置
│   ├── bcr_arm_gazebo/           # Gazebo world / ros2_control 配置
│   ├── bcr_arm_moveit_config/    # MoveIt2 配置
│   ├── bcr_arm_bringup/          # 综合启动 launch 文件
│   └── bcr_arm_demo/             # Pick and Place demo 代码
├── docker/
│   ├── Dockerfile.humble
│   └── entrypoint.sh
├── docker-compose.yml
└── README.md
```

---

## 3. 启动 Docker 容器

因为 Gazebo 和 RViz 需要图形界面，所以先允许 Docker 使用宿主机 X11：

```bash
xhost +local:docker
xhost +local:root
```

启动容器：

```bash
docker compose up -d bcr_arm_dev
```

如果本地没有镜像，Docker 会自动拉取：

```bash
docker pull carson0301/bcr_arm:humble-dev
```

确认容器启动成功：

```bash
docker ps | grep bcr_arm
```

进入容器：

```bash
docker exec -it bcr_arm_dev bash
```

进入容器后，默认工作目录应该是：

```bash
/workspace/my_brc_arm_ws
```

如果不是，手动进入：

```bash
cd /workspace/my_brc_arm_ws
```

---

## 4. 在容器内编译工作空间

在容器内执行：

```bash
cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

检查 ROS2 包是否存在：

```bash
ros2 pkg list | grep bcr_arm
```

正常应该看到类似：

```text
bcr_arm_description
bcr_arm_gazebo
bcr_arm_moveit_config
bcr_arm_bringup
bcr_arm_demo
```

如果编译失败，可以清理后重新编译：

```bash
rm -rf build install log

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## 5. 启动 Bringup

保持 Docker 容器运行。

新开一个宿主机终端，进入同一个 Docker 容器：

```bash
docker exec -it bcr_arm_dev bash
```

在容器内执行：

```bash
cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bcr_arm_bringup bringup.launch.py
```

这一步会启动机械臂仿真相关组件，包括：

```text
Gazebo / Ignition
机械臂模型
桌面
木块
障碍物
放置区域
ros2_control 控制器
MoveIt / RViz 相关组件
```

如果 Gazebo 或 RViz 无法显示，请确认宿主机已经执行：

```bash
xhost +local:docker
xhost +local:root
```

---

## 6. 运行 Pick and Place Demo

保持 `bringup.launch.py` 继续运行。

新开一个宿主机终端，进入同一个 Docker 容器：

```bash
docker exec -it bcr_arm_dev bash
```

在容器内执行：

```bash
cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bcr_arm_demo pick_place_demo.launch.py
```

正常流程如下：

```text
1. 机械臂移动到木块上方
2. 夹爪打开
3. 机械臂竖直下降到抓取位置
4. 夹爪闭合，物理夹住木块
5. 机械臂竖直抬起木块
6. 移动到放置区域上方
7. 竖直下降
8. 打开夹爪释放木块
9. 机械臂抬起并回到 home
```

---

## 7. 一键运行流程总结

### 终端 1：启动 Docker 容器

```bash
git clone https://github.com/bbbbkkk/bcr-arm-ros2-pick-and-place-demo.git
cd bcr-arm-ros2-pick-and-place-demo

xhost +local:docker
xhost +local:root

docker compose up -d bcr_arm_dev
```

---

### 终端 2：编译工作空间

```bash
docker exec -it bcr_arm_dev bash

cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

### 终端 3：启动 bringup

```bash
docker exec -it bcr_arm_dev bash

cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bcr_arm_bringup bringup.launch.py
```

---

### 终端 4：运行 Pick and Place Demo

```bash
docker exec -it bcr_arm_dev bash

cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bcr_arm_demo pick_place_demo.launch.py
```

---

## 8. 检查 ros2_control 控制器

如果 demo 没有正常执行，可以先检查控制器状态。

进入容器：

```bash
docker exec -it bcr_arm_dev bash
```

执行：

```bash
cd /workspace/my_brc_arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

正常应该看到类似：

```text
joint_state_broadcaster       active
joint_trajectory_controller   active
gripper_controller            active
```

查看 FollowJointTrajectory action：

```bash
ros2 action list | grep follow_joint_trajectory
```

正常应该看到类似：

```text
/joint_trajectory_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

如果看不到 `/gripper_controller/follow_joint_trajectory`，说明夹爪控制器没有正常启动。

---

## 9. 物理抓取说明

本项目使用的是 Gazebo 物理抓取。

木块不是通过 attach 插件强制绑定到夹爪上，而是依赖：

```text
夹爪 collision geometry
木块 collision geometry
Gazebo 物理接触
摩擦参数
夹爪闭合距离
机械臂轨迹执行
```

因此，物理抓取对下面参数比较敏感：

```text
GRIPPER_OPEN
GRIPPER_GRASP
grasp_z
place_release_z
木块 mass
木块 friction
夹爪 collision box 尺寸
机械臂速度 / 加速度
Gazebo physics 参数
```

如果木块容易掉，可以尝试：

```text
增大木块与夹爪的摩擦
减小 GRIPPER_GRASP，让夹爪夹得更紧
降低抬升速度
降低移动速度
```

如果木块容易被夹飞，可以尝试：

```text
增大 GRIPPER_GRASP，让夹爪不要夹太死
降低夹爪闭合速度
降低接近速度
```

---

## 10. 常见问题

### 10.1 Docker 容器无法打开 RViz / Gazebo

先确认宿主机执行过：

```bash
xhost +local:docker
xhost +local:root
```

确认容器中有显示环境变量：

```bash
echo $DISPLAY
echo $QT_OPENGL
echo $LIBGL_ALWAYS_SOFTWARE
echo $QT_XCB_GL_INTEGRATION
```

如果 RViz 报 `xcb` 或 OpenGL 相关错误，可以重启容器：

```bash
docker compose down
docker compose up -d bcr_arm_dev
docker exec -it bcr_arm_dev bash
```

---

### 10.2 Docker 镜像拉取失败

可以手动拉取：

```bash
docker pull carson0301/bcr_arm:humble-dev
```

如果出现 registry mirror 403，例如：

```text
docker.m.daocloud.io 403 Forbidden
```

说明 Docker daemon 走了第三方镜像源，该镜像源可能无法代理个人仓库。

可以临时移除 `/etc/docker/daemon.json` 中的 `registry-mirrors`，然后重启 Docker。

示例：

```bash
sudo cp /etc/docker/daemon.json /etc/docker/daemon.json.bak

sudo tee /etc/docker/daemon.json <<'INNER_EOF'
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "100m",
    "max-file": "3"
  }
}
INNER_EOF

sudo systemctl daemon-reload
sudo systemctl restart docker
```

然后重新拉取：

```bash
docker pull carson0301/bcr_arm:humble-dev
```

---

### 10.3 编译时报 rosdep 问题

如果 `rosdep update` 网络超时，可以先跳过 rosdep，直接编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

本项目 Docker 镜像中已经预装了主要依赖。

---

### 10.4 夹爪只有 visual，没有 collision

如果日志中出现：

```text
gripper_left_finger_link has visual geometry but no collision geometry
gripper_right_finger_link has visual geometry but no collision geometry
```

说明夹爪没有物理碰撞体，Gazebo 中无法真实夹住木块。

需要检查：

```text
bcr_arm_description
gripper xacro
left_finger_link
right_finger_link
```

确认左右夹爪 link 都包含 `<collision>`。

---

### 10.5 夹爪 action abort

检查 gripper controller：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces | grep gripper
ros2 topic echo /joint_states
```

可以单独测试夹爪：

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['gripper_left_finger_joint', 'gripper_right_finger_joint'],
    points: [
      {
        positions: [0.035, 0.035],
        time_from_start: {sec: 5, nanosec: 0}
      }
    ]
  }
}"
```

如果手动命令成功，但 demo 失败，需要检查 demo 中夹爪打开/闭合时机和 Gazebo 物理状态。

---

### 10.6 RViz 和 Gazebo 中物体位置不一致

Gazebo 中的物体来自 world 文件。

RViz / MoveIt 中的物体来自 PlanningScene 脚本。

两者需要同步：

```text
木块尺寸和位置
障碍物尺寸和位置
放置区域尺寸和位置
桌面尺寸和位置
```

如果只改 Gazebo world，而没有改 PlanningScene，RViz 中看到的位置可能不一致。

---

## 11. Docker Compose 文件说明

本项目的 `docker-compose.yml` 使用预构建 Docker Hub 镜像：

```yaml
image: carson0301/bcr_arm:humble-dev
```

并挂载当前项目源码：

```yaml
volumes:
  - .:/workspace/my_brc_arm_ws
```

这意味着：

```text
Docker 镜像负责提供 ROS2 / MoveIt2 / Gazebo 环境
GitHub 仓库负责提供项目源码
容器启动后，需要在容器内 colcon build
```

这种方式适合开发和复现。

---

## 12. 从源码重新构建 Docker 镜像

如果不想使用 Docker Hub 上的镜像，也可以本地重新构建：

```bash
docker compose build
```

或者：

```bash
docker build -f docker/Dockerfile.humble -t bcr_arm:humble-dev .
```

构建完成后启动：

```bash
docker compose up -d bcr_arm_dev
docker exec -it bcr_arm_dev bash
```

---

## 13. 项目当前状态

当前版本已完成：

```text
ROS2 Humble Docker 环境
Gazebo/Ignition 仿真场景
MoveIt2 机械臂规划
ros2_control 控制
简化夹爪物理抓取
障碍物避障
木块抓取与放置
GitHub 源码管理
Docker Hub 开发环境镜像
```

---

## 14. 后续优化方向

后续可以继续优化：

```text
1. 一键 bringup 启动
2. 参数化 pick / place 位置
3. 抓取成功检测
4. MoveIt Task Constructor
5. 随机木块位置抓取
6. 摄像头 / AprilTag / 点云感知
7. Isaac Sim 复现
8. Release Docker 镜像一键运行
```

