# calculate_attack_pose_if_available

ROS 2 攻击状态判断与追击位置计算功能包，用于衔接自瞄、敌方状态查询、行为树和导航模块。

主要功能：

- 查询目标是否可攻击，并对已有云台指令进行转发或关闭开火建议。
- 根据目标相对位置、机器人 TF 和栅格地图，在敌人周围选择一个追击目标点。
- 发布敌人位置、可行候选点和最终目标点，便于在 RViz 中查看。

本包输出的是地图中的二维目标位置，不包含导航路径、底盘速度或目标朝向，也不负责弹道解算。

> 当前仓库存在参数读取和配置匹配问题，详见[已知问题与限制](#已知问题与限制)。以下内容基于源码整理，尚未完成编译和运行验证。

## 工作流程

### 攻击状态判断

节点缓存 `/process_gimbal` 的云台指令，并在收到 `/front/armor_solver/target` 后处理：

| 条件 | 行为 |
| --- | --- |
| 目标 ID 为空 | 将 `pitch`、`yaw` 置零，设置 `fire_advice=false`，发布指令 |
| ID 为 `1`、`2`、`3`、`4` 或 `sentry` | 调用 `/enemy_invincible_status` 查询 `attack_available` |
| 查询结果为可攻击 | 转发缓存的云台指令，不主动将 `fire_advice` 改为 `true` |
| 查询结果为不可攻击 | 将 `pitch`、`yaw` 置零，关闭开火建议并发布 |
| 服务在 2 秒等待内不可用，或 ID 不在上述集合内 | 转发缓存的云台指令 |

输出话题为 `/serial/process_gimbal_aft_inv`。敌方状态查询服务由外部节点提供，本仓库不包含其实现。

### 追击目标点计算

以 100 ms 为周期（10 Hz）执行：

1. 查询 `map` 到 `base_link_frame_id` 的 TF，获得机器人地图位置。
2. 检查云台消息中的 `distance < max_chase_distance` 且 `fire_advice=true`。
3. 使用 `distance`、`pitch_diff`、`yaw_diff` 推算敌人在机器人坐标系中的位置，再转换到 `map`。角度差按度处理；此处没有直接使用 `Target.position`。
4. 检查机器人当前位置距行为树提供的 `(x, y)` 是否不超过 `max_distance_from_goal`。
5. 以敌人为圆心，在半径 `attack_radius` 的圆周上均匀生成 `num_sectors` 个候选点。
6. 使用 `/map` 筛选：候选点必须在地图内，栅格值在 `[0, cost_threshold]` 内，且机器人到候选点的直线上最大栅格值小于 100。
7. 在可行点中选择距机器人欧氏距离最近的一点，发布 `chase_ready=true` 和 `chase_x`、`chase_y`。

条件不满足、缺少地图、没有可行点或 TF 查询异常时，发布 `chase_ready=false`，坐标字段为零。下游应先检查 `chase_ready` 再使用坐标。

默认几何参数为：最大追击距离 8 m，攻击圆半径 3 m，圆周采样 15 点。

## ROS 接口

- 包名：`calculate_attack_pose_if_available`
- 可执行文件：`dec_shoot_and_pose`
- 节点名：`decide_shot_and_pose`

以下话题名按默认空命名空间列出。

### 订阅

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/front/armor_solver/target` | `rm_interfaces/msg/Target` | 根据目标 `id` 触发攻击状态判断 |
| `/process_gimbal` | `rm_interfaces/msg/GimbalCmd` | 缓存云台指令；提供距离、角度差和开火建议 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 候选点和直线路径的栅格检查 |
| `/tree/able_chase` | `rm_interfaces/msg/IsAbleToChaseSend` | 提供参考点 `x`、`y` 和允许偏离距离 `max_distance_from_goal` |

目标与云台订阅使用 `SensorDataQoS`；地图订阅使用 Reliable、Transient Local、Keep Last 10，接入时需匹配发布端 QoS。

### 发布

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/serial/process_gimbal_aft_inv` | `rm_interfaces/msg/GimbalCmd` | 经过攻击状态判断后的云台指令 |
| `/tree/chase_goal` | `rm_interfaces/msg/IsAbleToChaseCall` | `chase_ready`、`chase_x`、`chase_y` |
| `/enemy_markers` | `visualization_msgs/msg/MarkerArray` | RViz 可视化，参考坐标系为 `map` |

源码中的 `serial/process_gimbal_aft_inv` 和 `enemy_markers` 为相对话题名，设置命名空间后会随之变化。

### 服务与 TF

- 服务客户端：`/enemy_invincible_status`，类型 `rm_interfaces/srv/InvincibleInquire`，发送 `enemy_id`，读取响应的 `attack_available`。
- TF：需要能够查询从 `base_link_frame_id` 到 `map` 的变换。
- 地图与行为树参考点应采用与计算一致的地图坐标；代码没有为其他地图坐标系另做转换。

## 参数

配置文件：[config/chase_params.yaml](config/chase_params.yaml)。表中的源码默认值是节点声明参数时的值，不代表当前代码都能正确读取。

| 参数 | 源码默认值 | YAML 值 | 含义 |
| --- | --- | --- | --- |
| `base_link_frame_id` | `base_link` | `base_link_kf` | 用于机器人定位和目标坐标转换的坐标系 |
| `attack_radius` | `3.0` | `3.0` | 候选点圆周半径，单位 m |
| `num_sectors` | `15` | `15` | 圆周采样点数 |
| `cost_threshold` | `20` | `20` | 候选点可接受的最大栅格值 |
| `max_chase_distance` | `8.0` | `8.0` | 启动追击计算的目标距离上限，单位 m，判断为严格小于 |
| `chase_visualize` | `true` | `true` | 是否发布可视化标记 |

`max_distance_from_goal` 来自 `/tree/able_chase` 消息，不是 ROS 参数。

## 依赖与构建

需要 ROS 2、`colcon` 和 `ament_cmake`。仓库未指定或验证具体 ROS 2 发行版。

构建声明涉及：`rclcpp`、`std_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`visualization_msgs`、`nav_msgs`、`nav2_msgs`，以及项目自定义包 `rm_interfaces` 和 `referee_msg`。启用测试时还需要 `ament_lint_auto`、`ament_lint_common`。

自定义接口包不包含在此仓库中，需要从配套工程获取，并放入同一工作空间或提前构建、加载其环境。`referee_msg` 当前仍在构建依赖中，即使本节点没有直接使用其消息，也需要满足该依赖。

先加载所用 ROS 2 发行版的环境，再执行：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/movoQAQ/calculate_attack_pose_if_available.git

# 在 src 中补齐兼容的 rm_interfaces、referee_msg 等配套包。
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to calculate_attack_pose_if_available
source install/setup.bash
```

`rosdep` 不会自动获取缺失的项目自定义源码包。使用 zsh 时可改为加载 `install/setup.zsh`。

## 启动与查看

先处理下述参数读取与配置节点名问题，再构建并加载工作空间环境。启动前应提供地图、TF、自瞄消息、行为树追击范围消息，以及敌方状态查询服务。

```bash
ros2 launch calculate_attack_pose_if_available calculatepose.launch.py
```

也可以直接运行节点，使用源码默认参数：

```bash
ros2 run calculate_attack_pose_if_available dec_shoot_and_pose
```

检查接口与参数：

```bash
ros2 node info /decide_shot_and_pose
ros2 param list /decide_shot_and_pose
ros2 topic echo /tree/chase_goal
ros2 service type /enemy_invincible_status
```

在 RViz 中将 Fixed Frame 设为 `map`，添加 `MarkerArray`，选择 `/enemy_markers`：红色球体表示敌人位置，蓝色表示可行候选点，绿色表示选中的目标点。标记只在成功计算且启用可视化时发布，存活时间为 0.5 s。

## 已知问题与限制

以下问题记录的是当前实现，本文档没有修改源码。

1. **参数名拼写错误。** [src/decide_shoot_and_pose.cpp](src/decide_shoot_and_pose.cpp) 中读取的是 `cost_th(reshold`，声明的却是 `cost_threshold`。应统一为 `cost_threshold`；目前成员变量也没有初始值，不能依赖该阈值正常工作。
2. **YAML 节点名不匹配。** 配置顶层为 `/calculate_attack_pose_if_available`，实际节点名为 `/decide_shot_and_pose`。应改为实际节点名，或相应调整启动节点名称，否则配置无法按预期匹配。
3. **追击范围输入缺少就绪检查。** `goal_x`、`goal_y` 在收到 `/tree/able_chase` 前没有初始化。启动追击前需要收到有效消息，代码也需要补充初始化和输入就绪判断。
4. **可行性检查较简单。** 仅检查候选点和“机器人到候选点”的直线栅格，不计算完整导航路径，不检查“候选点到敌人”的射击视线，也没有显式车体轮廓检查。沿线未知栅格（`-1`）不会被当作障碍拒绝。
5. **范围约束只检查机器人当前位置。** 没有再次检查最终候选点是否仍处于行为树指定的允许范围内。
6. **消息没有超时失效处理。** 追击计算使用缓存的云台消息；敌方状态查询结果和后续云台消息也没有严格绑定。不可将当前逻辑理解为完备的持续攻击许可判断。
7. **构建声明仍需整理。** 源码使用了部分依赖于间接包含的标准库头文件；`CMakeLists.txt` 查找 `std_msgs`，而 `package.xml` 未显式声明该依赖。实际编译结果需在配套环境中验证。

## 目录结构

```text
.
├── CMakeLists.txt
├── package.xml
├── config/chase_params.yaml
├── launch/calculatepose.launch.py
├── include/calculate_attack_pose_if_available/decide_shoot_and_pose.hpp
└── src/decide_shoot_and_pose.cpp
```

## 许可证

`package.xml` 中许可证仍为 `TODO: License declaration`，仓库尚未提供正式许可证文件。
