# rmu_gazebo_simulator

## 1. Introduction

rmu_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmu_gazebo_simulator 提供以下功能：

- rmul_2024, rmuc_2024, rmul_2025, rmuc_2025 仿真世界模型

- 网页端局域网联机对战

- 机器人底盘、云台、射击控制

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

## 2. Quick Start

### 2.1 Option 1: Docker

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- 允许本地的 Docker 容器访问主机的 X11 显示

    ```bash
    xhost +local:docker
    ```

#### 2.1.2 Create Container

```bash
docker run -it --rm --name rmu_gazebo_simulator \
  --network host \
  --runtime nvidia \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ghcr.io/smbu-polarbear-robotics-team/rmu_gazebo_simulator:1.0.0
```

### 2.2 Option 2: Build From Source

#### 2.2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

#### 2.2.2 Create Workspace

```bash
sudo pip install vcstool2
pip install xmacro
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

本仿真器不再维护自己的依赖清单。rmoss 系列源码依赖与机器人描述包
（`ats_robot_description`）统一由 ATS 工作区根清单导入到 `src/dependencies`
与 `src/ats_robot_description`；若再导入嵌套清单，会解析出第二份
`rmoss_interfaces`/`sdformat_tools`。

```bash
git clone https://github.com/liukong1220/ATS_2026_snetry_test.git ats_ws
```

```bash
vcs import --recursive ats_ws < ats_ws/dependencies.repos
```

#### 2.2.3 Build

以下命令在工作区根目录（上例为 `ats_ws`）执行：

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --base-paths src --packages-select rmu_gazebo_simulator
```

### 2.3 Running

启动仿真环境

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

> [!NOTE]
> `gazebo.launch.py` 以 `-r` 启动，仿真会自动开始运行，无需再在 GUI 中点击播放。
> 其默认 `headless:=true`、`use_viewer:=false`，即不打开 Gazebo GUI；需要界面时传入
> `headless:=false use_viewer:=true`。

#### 2.3.1 Test Commands

以下测试命令依赖 `rmoss_gz_base` 的 robot_base 节点。`spawn_robots.launch.py`
默认 `launch_robot_base:=false`，单独调试时需传入 `launch_robot_base:=true`；
ATS 导航入口（见 2.4）固定为 `false`，否则会出现第二个底盘命令发布者。

控制机器人移动

```sh
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
#根据提示进行输入，支持平移与自旋
```

机器人云台

```sh
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入，支持绝对角度控制
```

机器人射击

```sh
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入
```

#### 2.3.2 网页端控制

支持局域网内联机操作，只需要将 localhost 改为主机 ip 即可。

操作手端

<http://localhost:5000/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py
```

裁判系统端

<http://localhost:2350/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_web/main.py
```

#### 2.3.3 切换仿真世界

`bringup_sim.launch.py` 读取 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`（当前为 `rmuc_2025`）。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`。出生位姿取自同一文件 `robots.<world>` 条目。

`ats_gazebo_nav.launch.py` 不读取该文件中的 `world`，而是使用 launch 参数 `world`（默认 `rmuc_2025`），出生位姿仍取自 gz_world.yaml。

### 2.4 ATS 导航闭环入口

```sh
ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py use_rviz:=true
```

速度链（单一发布者约束写在 launch 文件头注释中）：

```text
ats_swerve_mpc -> /cmd_vel/autonomy_raw
手动 /cmd_vel ----+
                  +-> cmd_vel_arbiter -> /cmd_vel/selected
                      -> gz_chassis_cmd_adapter -> /motion_control + /<robot>/cmd_vel -> Gazebo 底盘
```

本 profile 中 arbiter 以 `require_serial_link:=false` 运行；`fake_vel_transform`、`chassis_vel_transform` 不启动，大 yaw 旋转由 `gz_chassis_cmd_adapter` 完成。

TF 相关：

- `gimbal_yaw_odom -> front_mid360` 由 launch 内静态 TF 发布（`lidar_static_tf_*` 参数），默认 `(-0.1, 0.245, 0.325)`，roll 75°、pitch 0、yaw -161°。该值必须与实际 spawn 的 xmacro 中 Mid360 挂载一致，否则点云与 TF 不一致。注意：launch 注释称其对齐 `ats_sentry_robot.sdf.xmacro`，但当前 `ats_robot_description` 中该文件的挂载为 `-0.2 0 0 0 0 -61°`，上述默认值实际与 `simulation_robot` / `simulation_nav_robot.sdf.xmacro` 相同；使用默认 `robot_description_xmacro` 时请先核对并按需覆盖 `lidar_static_tf_*`。
- `gazebo_robot_tf_relay` 把命名空间下 robot_state_publisher 的 `/<robot>/tf`、`/<robot>/tf_static` 转发到全局 `/tf`、`/tf_static`：丢弃 `chassis -> gimbal_yaw_odom` 边并发布其逆链 `gimbal_yaw_odom -> base_footprint`，避免 `gimbal_yaw_odom` 出现两个父节点；`front_mid360` 已有全局发布者，不转发。
- RViz 配置 `rviz/ats_gazebo_nav.rviz` 包含 RobotModel（`/red_standard_robot1/robot_description`）。

常用 launch 参数：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `world` | `rmuc_2025` | 解析为 `resource/worlds/<world>_world.sdf`；`world_sdf_path` 可显式指定 |
| `params_file` | `ats_sentry_bringup/params/node_params.yaml` | 根仓导航参数 |
| `map_yaml` | `ats_sentry_bringup/map/rmuc_2025.yaml` | 静态规划地图 |
| `use_gazebo_gt_odometry` | `true` | 用 `/<robot>/gimbal_odometry_gt` 中继 `/odometry` 与 `/registered_scan`，不启动 Point-LIO/loam_interface；`false` 走 LIO 链 |
| `launch_small_gicp_relocalization` | `true` | 先验地图 GICP 重定位，需要 `prior_pcd_file`（默认 `ats_sentry_bringup/pcd/<world>.pcd`，缺失时启动失败） |
| `launch_planning` | `true` | 启动 ROGMap、adapter、goal manager、MINCO、MPC、arbiter、底盘 adapter |
| `headless` / `use_viewer` | `true` / `false` | 无 GUI 运行 |
| `use_rviz` | `false` | 启动 RViz |
| `require_gimbal_status` | `false` | 仿真没有串口云台桥，不发布 `/gimbal/yaw_status` |
| `enable_camera_sensors` | `false` | 关闭工业相机传感器，避免与 Mid360 争用渲染线程 |
| `robot_description_package` / `robot_description_xmacro` | `ats_robot_description` / `ats_sentry_robot.sdf.xmacro` | 机器人模型来源 |

完整参数列表见 `ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py --show-args`。

## 配套导航仿真仓库

- ATS 2026 哨兵导航（本工作区主链）

    机器人描述由 `ats_robot_description` 提供，仿真接入入口见
    [ats_gazebo_nav.launch.py](./rmu_gazebo_simulator/launch/ats_gazebo_nav.launch.py)，用法见 2.4。

- SMBU PolarBear Sentry Navigation（上游算法参考，非本工作区主链）

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmu_gazebo_simulator is provided under Apache License 2.0.
