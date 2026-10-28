# FINS (FineVision) 框架使用经验

> 逐步更新的经验总结。每次遇到新的框架接口 / 构建问题就往对应小节里补一条，并更新文末「变更记录」。
> 最近更新：2026-10-28

本仓库 `fins_navstack` 是一个 FINS 工作区，包含插件包：`livox_driver2`、`FAST_LIO`、`pointcloud_converter`、`ros_bridge`、`global_localization`。

---

## 1. 构建链路

### 1.1 命令

```bash
source /opt/ros/humble/setup.bash
cd ~/fins_navstack
fins dep solve <包名>     # 按 package.yaml 解析/安装依赖（ROS 内的 "system" 依赖会被跳过或交给系统）
fins build [包名]         # 不写包名 = 构建工作区内全部插件（README 用法）
```

产物落在 `~/.fins/install/<包名>/lib/lib<包名>.so`（节点插件）以及仓库内的 `build/`、`install/`（生成物，不要手改、不要提交）。

### 1.2 `fins build` 实际做了什么（关键）

`build/<包名>/CMakeLists.txt` 是 **fins 自动生成的 wrapper**，每次构建都会重写，它不是仓库代码。里面包含：

1. `CMAKE_INSTALL_RPATH` / `FINS_DEP_PATHS`：把 `~/.fins/install`、`~/.fins/dependencies/install/<dep>/<ver>-<hash>` 塞进 rpath 和 `CMAKE_PREFIX_PATH`
2. `# --- FINS PRE-LOAD DEPENDENCIES ---` 段：对 `package.yaml` 中**走 recipes 安装的非 ROS 依赖**执行 `find_package(... REQUIRED)` 并锁版本（例如 `livox_msg`、`Eigen3`、`LivoxSDK`）
3. WSL 环境清洗：剔除 `PATH` 与 cache 中的 `/mnt/*`（Windows 路径），避免 CMake 误找到 Windows 侧的同名库
4. 定义宏 `fins_add_node()`、`fins_link_ros_dependencies()`、`fins_optional_ros_dependency()`
5. `add_subdirectory(<仓库>/src/<包名>)` 把你的 CMakeLists 挂进来

**推论**：`/opt/ros/<distro>` 本来就在 `CMAKE_PREFIX_PATH` 里，所以 **纯 ROS 系统包（含 `livox_ros_driver2`）不需要写进 `depends` 也能 `find_package` 到**；只有需要 fins 下载/编译的第三方库才写 `depends` + `recipes`。

### 1.3 改了依赖之后

改了 `package.yaml` 的 `depends` 后，旧的 `build/<包名>/CMakeCache.txt` 里还留着老依赖变量（如 `livox_msg_DIR`），一般无影响；想干净一点直接删掉生成物重建：

```bash
cd ~/fins_navstack && rm -rf build install && fins build
```

---

## 2. `package.yaml` 字段

```yaml
name: "FAST_LIO"          # fins 包名 = 插件 .so 名 = 生成 wrapper 的子目录名
version: "0.1.0"
description: "..."
maintainers:
  - name: "fins"
    email: "fins@sjtu.edu.cn"
depends:                  # 依赖列表
  Eigen3: "3.4.0"         # 走 recipes，由 fins 下载编译，带版本号
  OpenMP: "system"        # 系统提供，fins 不负责安装
  sensor_msgs: "system"   # ROS 消息包也写 system
recipes:                  # 声明 depends 里非 system 依赖怎么获取
  Eigen3:
    git: "https://..."
    build_system: "cmake" # 也有 ament/catkin 等
    tag/branch + cmake_args
licenses:
  - "GPL-3.0-only"
```

- `recipes` 里的 `git` 可以是 fork（本仓库大量使用 `github.com/Han-Yu-Meng/*` 的定制分支）。
- 版本号字符串必须与 `versions:` 下的 key 完全一致。
- **注意**：`depends` 的 key 会被 `fins dep solve` 拿去找依赖来源。像 `livox_ros_driver2` 这种「已装在 /opt/ros 里、但不是独立 rosdep key」的包，**不要**写进 `depends`，否则解析可能失败；直接在 CMakeLists 里 `find_package` 即可。

---

## 3. 节点插件（Node）写法

### 3.1 一个 .so = 一个节点

```cpp
class FastLIO : public fins::Node {
  void define() override;      // 只做注册：IO 槽位、参数、名字/分类
  void initialize() override;  // 启动：建线程、开设备、加载配置
  void run() override;         // 可选的主循环（很多节点这里是空的）
  void pause() override;       // 停：join 线程、停设备
  void reset() override;       // 复位：pause + 重新初始化
};
EXPORT_NODE(FastLIO)
DEFINE_PLUGIN_ENTRY(fins::STATEFUL)   // 或 fins::STATELESS
```

- `STATEFUL`：框架会保留节点实例（有状态，如建图）；`STATELESS`：纯函数式（如驱动、转换器）。
- 一个翻译单元只能有一个 `EXPORT_NODE` + 一个 `DEFINE_PLUGIN_ENTRY`；多节点就拆文件，用 `fins_add_node` 分别建库。

### 3.2 IO 与参数

```cpp
register_input<sensor_msgs::msg::Imu>("imu", &FastLIO::on_imu);
register_input<livox_ros_driver2::msg::CustomMsg>("lidar", &FastLIO::on_livox);
register_output<nav_msgs::msg::Odometry>("odometry");
register_parameter<double>("publish_freq", &Node::on_freq_changed, 10.0);  // 带默认值 + 变更回调
```

- 槽位名字可以写数学式：`register_output<geometry_msgs::msg::TransformStamped>("$T_{odom}^{base}$")`，UI 里按 LaTeX 渲染。
- 回调形如 `void on_x(const fins::Msg<T> &msg)`；`msg->field`、`*msg`、`msg.ptr()`、`msg.acq_time`。
- 发送：`send(slot, msg_obj, acq_time)`。时间用 `fins::from_seconds(double)` 或 `fins::from_ros_time(stamp)`。
- `required("slot")` 判断下游是否有消费者 —— **没有消费者就别算**，这是省 CPU 的主要手段（本仓库的驱动节点用它决定要不要额外生成 `PointCloud2`）。

### 3.3 消息类型与 ABI

- `fins::AnyMsg` 用 `std::type_index(typeid(T))` + `AbiTag<T>` 做类型校验；`AbiTag` 对 ROS 消息取的是 **rclcpp 版本号**，对 `pcl::PointCloud<T>` 取的是 PCL 版本号（见 `fins/abi_traits.hpp`）。
- 依赖 `rosidl_generator_traits::is_message<T>` 判定「是不是 ROS 消息」，所以**必须把消息头文件 include 进来**，否则会被当成普通类型（ABI tag = 0，跨插件类型校验可能不匹配）。
- 类型名（UI 显示用）来自 `TypeRegister`，默认 `demangle(typeid(T).name())`，所以换了消息包名字，UI 里显示的类型名也会跟着变，属于正常现象。

### 3.4 CMake 侧

```cmake
fins_add_node(<target> SHARED src/xxx.cpp)   # 建 SHARED 库 + 链 fins_sdk + 装到 lib/
fins_link_ros_dependencies(<target> sensor_msgs nav_msgs pcl_conversions)
fins_optional_ros_dependency(<target> livox_ros_driver2)  # 找到才链，并定义 WITH_LIVOX_ROS_DRIVER2
```

`fins_link_ros_dependencies` 的三段逻辑（生成 wrapper 里可见）：

1. 存在 `${pkg}::${pkg}` 目标 → 链它；
2. 否则存在同名目标 `${pkg}` → 链它；
3. 否则退化为 `${pkg}_INCLUDE_DIRS` + `${pkg}_LIBRARIES`（即「一次性把这个包导出的所有库都链上」）。

### 3.5 把内部槽位透出成 ROS 话题

fins 的 slot 只在图内部流动，**不会自动出现在 `ros2 topic list` 里**。要让外部（rviz2、rosbag2、别的 ROS 节点）能拿到数据，就在 launch 里挂一个 `ros_bridge` 的发布节点，把 slot 接到 `topic` 参数上。`ros_bridge` 已按消息类型把发布节点都造好了（`src/ros_bridge/src/nodes.cpp` 里的 `DEFINE_ROS_PUB_NODE`）：

| 节点名 | 消息类型 |
|---|---|
| `PointCloudPublisher` | `sensor_msgs/msg/PointCloud2` |
| `ImuPublisher` | `sensor_msgs/msg/Imu` |
| `OdometryPublisher` | `nav_msgs/msg/Odometry` |
| `PathPublisher` | `nav_msgs/msg/Path` |
| `LivoxCustomMsgPublisher` | `livox_ros_driver2/msg/CustomMsg` |
| `TFBroadcaster` / `StaticBroadcaster` | `tf2` 广播（`transform` 槽位） |

统一接口：输入槽位名固定叫 **`msg`**，参数 `topic / history / depth / reliability / durability`：

```python
Node(
    package="ros_bridge",
    name="LivoxCustomMsgPublisher",
    parameters={
        "topic": "/livox/lidar",
        "history": "Keep Last",
        "depth": "10",
        "reliability": "Reliable",
        "durability": "Volatile",
    },
    inputs={"msg": "sensor/lidar"},
),
```

注意事项：

- 挂上发布节点后，被连的那个槽位才有消费者，上游 `required("<slot>")` 才开始返回 true（驱动节点的 `lidar_standard` 目前没有节点接，所以 PointCloud2 转换一直是被跳过的）。搭链路时要意识到：**没接的槽位等于不算**，接上就真的开始产生数据、也真的开始吃 CPU。
- QoS 要对得上：rviz2 / 大部分订阅者默认 `Reliable`，如果这里写成 `Best Effort` 会出现「话题在、但一直没数据」（QoS 不兼容）。本仓库一律用 `Reliable`。
- `/livox/imu` + `/livox/lidar`（CustomMsg）是 Livox 官方 `livox_ros_driver2` 的默认话题名，按这套命名透出后，原版 FAST-LIO、外部定位模块、现成的 rviz 配置都能直接复用。
- `depth` 在 launch 里写成字符串（`"10"`）或数字（`10`）都能用，仓库现有写法是字符串。

---

## 4. 坑：ROS 消息包的链接方式

### 4.1 `${pkg}::${pkg}` 基本不存在

Humble 里消息包导出的目标叫 `nav_msgs::nav_msgs__rosidl_generator_cpp`、`...::nav_msgs__rosidl_typesupport_cpp` 等，**没有** `nav_msgs::nav_msgs`。所以 `fins_link_ros_dependencies` 基本都走第 3 条分支，把 `${pkg}_LIBRARIES`（含 generator_c / typesupport_* / generator_py 等一整套）全链上。对普通消息包没问题，但对「包名 = 驱动包、消息只是附带」的包就危险（见 4.2）。

### 4.2 `livox_ros_driver2` 的两个特殊之处

1. **目标名与库名不一致**：`/opt/ros/humble` 里的这个包，CMake 目标前缀是 `livox_interfaces2`（历史原因），实际 .so 却叫 `liblivox_ros_driver2__*`。所以正确的目标是：
   - `livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp`（纯头文件，带 include dir）
   - `livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp`
   - `livox_ros_driver2::livox_interfaces2__rosidl_typesupport_introspection_cpp`（需要内省时）
2. **`livox_ros_driver2_LIBRARIES` 里带 `liblivox_ros_driver2.so`**（即 Livox 官方驱动本体），其中已经定义了 `livox_ros::Lddc`、`livox_ros::DriverNode`、`livox_ros::Lds` 等符号。本仓库的 `livox_driver2` 包**自己也实现了一套同名类**，如果把该库链进来，动态链接时可能被系统库抢先解析，导致「代码看起来是本仓库的，实际跑的是官方实现」这类难查问题。

因此本仓库统一**只链 typesupport 目标，不链 `${livox_ros_driver2_LIBRARIES}`**：

```cmake
find_package(livox_ros_driver2 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE
  livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp   # 头文件 + include dir
  livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp # 序列化，rclcpp 发布要用
)
```

> 注：自己打的 msg-only deb 也先按同样命名核对一遍（`rosidl_generate_interfaces()` 的 library 名决定目标前缀）。本仓库使用的 deb 已确认沿用 `livox_interfaces2__*`，与 `/opt/ros` 版本一致，无需做目标名兼容。

`find_package(livox_ros_driver2)` 内部会 `find_package` 它的依赖（std_msgs、builtin_interfaces、rclcpp、sensor_msgs、pcl_conversions…），所以不需要额外声明。

**四个包统一策略**：`livox_driver2`、`FAST_LIO`、`pointcloud_converter`、`ros_bridge` 都只链上面这两个目标，`ros_bridge` 里**不要**再用 `fins_optional_ros_dependency(ros_bridge livox_ros_driver2)`（它等价于兜底分支，会把 `liblivox_ros_driver2.so` 一起链上），而是自己写 `find_package(... QUIET)` + `if(..._FOUND)` 手动链目标，并保留 `WITH_LIVOX_ROS_DRIVER2` 宏（`nodes.cpp` 用 `#ifdef` 控制是否注册 Livox 相关节点）。

---

## 5. 迁移记录：自建 `livox_msg` → ROS `livox_ros_driver2`

### 5.1 为什么要换

自建 `livox_msg`（`github.com/Han-Yu-Meng/livox_msg`，版本 1.0.1）把 `CustomMsg/CustomPoint` 以 `livox_driver2/msg/*.hpp` 的头路径 + `livox_driver2::msg::` 命名空间发布（1.0.0 时代用的是 `livox_ros_driver2` 命名空间，名字被改过一次）。换成系统 ROS 的 `livox_ros_driver2` 后：

- 不再需要 fins 下载编译一份消息包，`fins dep solve` 更轻
- 与 `ros_bridge`（已经在用 `livox_ros_driver2::msg::CustomMsg`）、rviz、bag 录制等生态工具天然一致

### 5.2 新旧对照

| | 自建 livox_msg 1.0.1 | ROS livox_ros_driver2 |
|---|---|---|
| include | `livox_driver2/msg/custom_msg.hpp` | `livox_ros_driver2/msg/custom_msg.hpp` |
| 命名空间 | `livox_driver2::msg::` | `livox_ros_driver2::msg::` |
| 安装位置 | `~/.fins/dependencies/install/livox_msg/1.0.1-default/include` | `/opt/ros/humble/include/livox_ros_driver2` |
| CMake 目标 | `livox_msg::livox_msg` | `livox_ros_driver2::livox_interfaces2__rosidl_*` |

字段完全一致（`header / timebase / point_num / lidar_id / rsvd / points`，点字段 `offset_time / x / y / z / reflectivity / tag / line`），所以只是类型名替换，不需要改取值逻辑。

### 5.3 改动清单

| 文件 | 改动 |
|---|---|
| `src/livox_driver2/src/include/livox_driver2_api.h` | include + `using CustomMsg = livox_ros_driver2::msg::CustomMsg` |
| `src/livox_driver2/src/lddc.h` | include + `using CustomMsg/CustomPoint` |
| `src/livox_driver2/CMakeLists.txt` | `find_package(livox_msg)` → `find_package(livox_ros_driver2)`；链接改 typesupport 目标 |
| `src/livox_driver2/package.yaml` | 删除 `livox_msg` 依赖与 recipe（保留 `LivoxSDK`） |
| `src/FAST_LIO/src/preprocess.h` / `preprocess.cpp` | `Preprocess::process`、`avia_handler` 签名 |
| `src/FAST_LIO/src/laserMapping.cpp` | `livox_pcl_cbk` 签名 |
| `src/FAST_LIO/src/fast_lio2_node.hpp` | `register_input<...>` 与 `on_livox` |
| `src/FAST_LIO/CMakeLists.txt` / `package.yaml` | 同上（find_package / 链接 / 依赖） |
| `src/pointcloud_converter/util_funcs.hpp` | `Livox2PCL`、`Livox2ROS` 两个 Function 的输入类型 |
| `src/pointcloud_converter/CMakeLists.txt` / `package.yaml` | 同上 |
| `src/ros_bridge/CMakeLists.txt` | `livox_ros_driver2` 从 `ROS_OPTIONAL_PACKAGES` 移出，改为手动 `find_package(... QUIET)` + 只链两个 typesupport 目标，保留 `WITH_LIVOX_ROS_DRIVER2` |

`ros_bridge/src/nodes.cpp` 本来就用 `livox_ros_driver2::msg::CustomMsg`（外面包着 `#ifdef WITH_LIVOX_ROS_DRIVER2`），源码无需改动。

### 5.4 迁移后注意

- `build/`、`install/` 里旧的 `CMakeCache.txt` 仍指 `livox_msg_DIR`，建议删掉重建。
- `.github/workflows/fins-build.yml` 的 CI 镜像（`ros:humble-ros-base`）**不含** `livox_ros_driver2`。方案是自己打一个 msg-only 的 deb，CI 里 `sudo apt-get install -y /tmp/xxx.deb`（或建内网/公网 apt 源）——完整做法见 `docs/apt-package-livox-msgs.md`。
- 话题槽位名（`sensor/lidar` 等）没有变，`launch/*.py` 不需要改；变的只是槽位上的 C++ 类型名。

---

## 6. 从 Windows 操作 WSL 仓库的注意事项

- 编辑器/工具走 UNC 路径：`\\wsl.localhost\Ubuntu-22.04\home\steven\fins_navstack`。
- 通过 UNC 批量改文件时偶发 `EBUSY: resource busy or locked`，或「返回成功但内容未变」；**改完要回读确认**，必要时重试。
- 编译必须在 WSL 内部执行（`fins`/`finsd` 是 Linux 程序）。若 Windows 侧的安全策略屏蔽了 `wsl.exe`，就无法从 Windows 发起构建，只能在 WSL 终端里手动跑：
  ```bash
  source /opt/ros/humble/setup.bash
  cd ~/fins_navstack && fins build
  ```

---

## 7. 参数文件的真实解析规则（读源码确认）

`finsd` 的 `apply_parameters` 最终走 `~/.fins/sdk/sdk/fins/server/parameter_server.cpp` 的 `load_string()`，
它**不是** YAML 解析器，而是一个「按缩进拼点分键」的手写解析器。规则：

1. **支持嵌套**。看到 key 后面的值不是空，就把它当成叶子；否则压栈当前缩进层，
   下一层 key 会拼成 `父.子`。所以
   ```yaml
   finenav_mppi_controller:
     CostCritic:
       cost_weight: 3.81
   ```
   等价于键 `finenav_mppi_controller.CostCritic.cost_weight`。
2. **Studio 导出的「模板格式」不能用**。那种写法是
   ```yaml
   cost_weight:
     default_value: 3.81
     type: double
   ```
   会多出一层 → 全键变成 `...cost_weight.default_value` → `get("cost_weight")` 找不到 →
   **静默回落代码默认值**（只打一条 `Parameter not found` 的 warn）。这是最容易踩的坑。
3. 顶层 key 必须等于节点里 `fins::ParamLoader("前缀")` 的字符串。
4. 所有值都以字符串保存、查询时现场转换；`bool` 只认 `true` / `1` / `on`。
5. 行内 `#` 之后的内容被**无条件**当注释砍掉（即使写在引号里），路径里不要出现 `#`。
6. 查不到精确键时还有一层「后缀匹配」兜底（`get_impl` 里遍历 `params_` 找同后缀的键），
   所以少写一层前缀有时也能查到值 —— 但这会让两个同名子键互相串味，不要依赖它。

## 8. 仿真导航链路（`launch/simulation.py`）

把三块 finenav 节点接到 Gazebo 仿真上，跑通「A* 全局规划 + MPPI 局部控制」：

```
/Odometry (nav_msgs/Odometry, 位姿+速度) --[OdometrySubscriber]--> nav/odom
/cloud_registered_body (base_link 系点云) --[PointCloudSubscriber]--> finenav_terrain_map
/goal_pose (rviz 2D Goal Pose)            --[PoseStampedSubscriber]--> finenav_astar_planner

nav/odom + nav/goal        --> finenav_astar_planner   --> nav/global_path / nav/costmap
nav/odom + nav/global_path --> finenav_mppi_controller --> nav/cmd_vel / nav/optimal_path
nav/cmd_vel                --> [TwistPublisher] --> /cmd_vel
```

要点：

- **服务必须成对声明**。MPPI 的 `CostCritic` 通过 `register_client<RollingGridData(void)>("get_rolling_grid_data")`
  向地图节点要「以机器人为中心的滚动栅格」，而 `SimpleMapNode` 用 `register_server` 提供它。
  launch 里要让两边的 `topic` 字符串一致：
  ```python
  Node(package="finenav_terrain_map", name="SimpleMapNode", ...,
       servers={"get_rolling_grid_data": SVC, "get_base_frame_id": SVC2})
  Node(package="finenav_mppi_controller", name="MPPIControllerNode", ...,
       clients={"get_rolling_grid_data": SVC, "get_base_frame_id": SVC2})
  ```
  漏了 `clients`，MPPI 拿到的是默认构造的 `RollingGridData`（`map_size = 0`），
  `getIndexX()` 里 `1/resolution` 直接除零。`IMapView::isValid()` + 控制循环里的判空就是挡这个的。
- **节点顺序**：`SimpleMapNode` 要排在 `MPPIControllerNode` 前面，让服务端先注册。
- **点云必须是 base_link 系**。`SimpleMapNode::onPointCloud` 自己用 odom 的 yaw 把它旋到地图系
  （`gx = lx*cos - ly*sin + origin`），喂地图系的 `/cloud_registered` 会转两次。
- `/Odometry` 的 `header.frame_id` 是 `odom`，但仿真里 `map -> odom` 是 identity，
  所以栅格地图（`map` 帧）的坐标可以直接和它对上，规划器输出 `frame_id: map` 没问题。
- 参数统一放 `launch/simulation_config/nav_sim.yaml`（`AstarPlanner` / `SimpleMap` /
  `finenav_mppi_controller` 三段）。同目录的 `mppi.yaml`、`simple_map.yaml` 是单节点参数表，
  `simulation.py` 不加载。

### 8.1 A* 规划器的实现选择

`finenav_astar_planner` 重写成了「静态栅格地图 + 车体尺寸约束」的版本（原来是直线插值）：

1. 自己读 `map.yaml` + PGM（`P5`/`P2`、8/16 bit），不引 yaml-cpp / OpenCV，
   依赖只有 `geometry_msgs` / `nav_msgs`。

   > **PGM 头部的空白陷阱**：`maxval` 之后**恰好有且只有一个**空白字符，之后就是二进制像素。
   > 而像素里 `0x00` / `0x0A` / `0x20` 都是合法灰度值，多吞一个字节整幅图就错位，报错是
   > `short read of pgm pixel data`（本次 406×406 的图正好少 1 字节）。
   > 实现上 `nextToken()` 用 `peek()` 定位、**不消费**结尾空白，再由 `loadPgm` 精确吃掉一个
   > （含 `"\r\n"` 的情况）。用 map_server 的 `map_saver` 出的图验证过：header 66 字节 +
   > 164836 像素字节 = 文件 164902 字节，正好读到底。
   >
   > 另外 `map_to_pgm.py` 出的图是 `0=occupied / 254=free / 205=unknown`，配
   > `negate: 0` + `occupied_thresh 0.65` / `free_thresh 0.196`：205 换算出的 occ 恰好
   > 0.196078 > free_thresh，落进 unknown 分支，三值语义得以保留。

2. 用**精确欧氏距离变换**（Felzenszwalb & Huttenlocher，二维可分离两遍）算每格到最近障碍的距离，
   再按 `costmap_2d` 的 inflation 语义赋值：
   - `d <= inscribed_radius` → `253` 且标 lethal（车体中心进入即碰撞，禁止通行）
   - `inscribed < d <= inflation_radius` → `252 * exp(-k*(d - inscribed))`
3. 8 邻域 A*，octile 启发，边权乘 `1 + penalty_weight * cost/252`，
   并禁止「擦着墙角斜穿」（对角线走时两侧正交邻居必须都通行）。
4. 视线（Bresenham）拉直去掉锯齿，再按 `step_size` 等距重采样成稠密 `nav_msgs/Path`
   （MPPI 需要密路径），终点朝向沿用目标四元数。

车体尺寸是通过**膨胀**体现在栅格上的，所以只要路径走在非 lethal 格里，就等价于车体不碰障碍。

### 8.2 启动排错：`Node class 'xxx/YYY@default' not found`

这是第一次跑 `simulation.py` 撞到的坑，值得单独记一笔。

节点全名是**三段**：`{source}/{类名}@{version}`

```
nodelib.hpp:354   std::string full_key = source + "/" + node_name + "@" + version;
node.hpp:90       std::string version = "default";        // 只有 set_version() 能改
node.hpp:387-397  #ifdef PKG_SOURCE → meta_.source      // 编译期注入
                  #ifdef PKG_NAME   → meta_.package_name // 不参与查找键！
```

- `version` 默认就是 `"default"`，没人调 `set_version()`，所以 launch 里不用管。
- **`source` 来自编译宏 `PKG_SOURCE`**，由 `fins build` 的生成脚本注入成**工作区名**：
  ```
  build/ros_bridge/CMakeLists.txt:138  add_compile_definitions(PKG_SOURCE="${FINS_META_SOURCE}")
  build/livox_driver2/build.ninja     -DPKG_SOURCE=\"fins_navstack\"
  ```
- Python 侧 `Node.source` 在 `__init__` 里就定死：不显式传就只有 `fins.core` 的模块级默认
  `_CURRENT_DEFAULT_SOURCE = "common"`；进入 `with DefaultSource():` 才换成
  `_detect_workspace_name()` 自动探测的值（按 `~/.fins/config.yaml` 的 `local_packages`
  排序匹配 launch 脚本路径 → `fins_navstack`）。
- **坑**：把 `Node(...)` 写成模块级常量（`DATAFLOW = [...]`）时，节点在 import 阶段
  就构造完了，那时还没进 `with` 块，`source` 永远是 `"common"` →
  `Agent` 去查 `common/OdometrySubscriber@default`，而插件注册的是
  `fins_navstack/OdometrySubscriber@default`，于是报 not found。

  正确写法就是 `sensor.py` 那样：节点在**函数体内部**构造，函数在 `with` 块里调用。
  ```python
  def simulation_group():
      return Group([Node(...), ...])       # ← 构造发生在这里

  with Agent(name="nav_sim") as agent:
      agent.add_config(...)
      with DefaultSource():
          agent.launch(simulation_group()) # ← 此时 source 已确定
  ```

排查手段：报错里 `xxx/` 那一段就是 source，拿它跟 `build/<pkg>/build.ninja` 里的
`-DPKG_SOURCE=` 对一下就知道了。

### 8.3 `fins build` 不带参数只编 `workspace.yaml` 里的包

工作区根目录跑 `fins build` 时，`build.go` 只要看到 `workspace.yaml` 就 `buildAll=true`，
把 `{"workspace": <path>}` 丢给 `finsd`，由守护进程按 `workspace.yaml` 里的 `repos` 列表
决定编哪些——**不在列表里的本地包会被静默跳过**（不报错，只是没出现在 `Building...` 里）。

本工作区 `workspace.yaml` 只有 5 个上游仓库，三个 `finenav_*` 是我们自己加的本地包，
所以要显式点名构建：

```bash
fins build finenav_astar_planner
fins build finenav_terrain_map
fins build finenav_mppi_controller
```

是否成功看 `install/` 里有没有新的 `lib<pkg>.so`，以及启动日志里
`[NodeLib] Load complete: N success, 0 fail.` 的 N 有没有涨。

### 8.4 MPPI 角速度抖动（wz 上下乱摆）的定位与调参

症状：仿真里能跑，但 `wz` 在 ±0.2~0.4 rad/s 之间持续反复横跳，车身左右摇。

读源码确认的三个机制（按贡献大小排序）：

**① `VelocityDeadbandCritic` 的 wz 门槛是主因**

`velocity_deadband_critic.cpp:95-101` 的代价是

```
cost = Σ_t [ (db_vx-|vx_t|)⁺ + (db_wz-|wz_t|)⁺ ] * model_dt * weight
```

它对**符号不敏感**，只在速度绝对值低于门槛时罚。代入当时配置
（`db_wz=0.2`、`weight=35`、`model_dt=0.1`、`time_steps=30`）：

| 样本 | deadband 代价 |
|---|---|
| 直行、`wz≡0` | `0.2*0.1*30*35 = 21.0` |
| 一直以 `wz=0.2` 转 | `0` |

而"一直转"让轨迹偏离路径约 0.5 m，`PathAlignCritic`（weight 14、平方距离）
也只付 `14*0.25 ≈ 3.5`。**21 : 3.5 = 6:1**，等于用压倒性权重命令"必须一直转、
且至少 0.2 rad/s"，但对正负中立 → 由噪声决定往左还是往右 → ±wz 反复横跳。

结论：Gazebo 差速插件没有电机死区，这个 critic 在仿真里应该 `enabled: false`
（参考的 `mppi.yaml` 也是关的）。要开就用 `[0.05, 0.05, 0.05]` + `weight 1~2`。

**② 采样的 `std` 直接等于抖动幅度**

`updateControlSequence()`（`optimizer.hpp:514-546`）做的是
`control_sequence = Σ softmax_i * (cs + noise_i)`。当 softmax 接近 argmin 时
它退化成 `cs + noise_best`，也就是**"发出去的指令 = 那条获胜样本的采样噪声"**。
所以 `wz_std: 0.4` 就字面意义地意味着 10 Hz 的 ±0.4 rad/s 随机摆动。

配套关系：`std ≈ 0.25 × 上限`。`wz_max 0.6 / wz_std 0.15`、
`vx_max 0.30 / vx_std 0.08`。std 也不能太小（<0.05），否则样本全挤在一起、
代价差分辨不出来，轨迹会退化。

**③ `temperature` 太小 → softmax 变成硬 argmin**

`costs_normalized = costs - min(costs)`，权重 ∝ `exp(-ΔC / temperature)`。
`temperature: 0.3` 时 ΔC=1 的样本只有 3.6% 权重，几乎没有"平均"效果；
提到 0.5，让 10~30 个相近样本一起参与加权，噪声才被平均掉。
`batch_size` 同理（加权平均方差 ∝ 1/N_eff），从 1000 提到 2000。

**速度包络直接对齐 `active_place_recognition_sim` 的 simple_nav**：
`coverage_mission.py` 里 `linear_speed 0.3`、`angular_speed 0.6`，
所以 `vx_max 0.30 / wz_max 0.60`。给到 0.5/1.0 只会让指令在限幅边界来回撞。

**还没动、可以升级的两处代码改动**（都要重新 `fins build`）：

1. `optimizer.hpp:168` 的 `utils::savitskyGolayFilter(control_sequence_, control_history_, settings_)`
   被注释掉了——这是上游 nav2 自带的跨周期平滑（9 点二次 SG 核，
   `utils.hpp:458`，`time_steps < 20` 时不生效，我们 30 步刚好可用）。
   取消注释即可，它自己会维护 `control_history_`。
2. `optimizer.hpp:169` 的 `auto control = getControlFromSequenceAsTwist(...)` 是**死代码**：
   `evalControl` 的返回类型被注释掉了（现在返回 `void`），算出来的 twist 直接丢弃，
   真正发布的是 `controller.cpp` 里 `getImmediateCommand()` 读的 `control_seq(0)`。
   两者取值一致（都是 shift 前的第 1 个元素），所以不影响正确性，只是白算一次。

### 8.5 路径「擦着障碍物边缘过」然后卡死

症状：全局路径看着贴着障碍物走，机器人到障碍物旁边就不动了（有时是整个节点挂掉）。

**不是"没做膨胀"** —— A* 本体确实做了（EDT + inscribed lethal + 指数代价），
实测它自己规划出来的栅格路径留白有 0.55 m。问题是后面两步把它浪费掉了：

**① 视线拉直只看 `isLethal`，等于把路径拉回 lethal 边界**

原来的 `lineOfSight` 用 Bresenham + `isLethal` 判定。`inscribed_radius: 0.28` 时
「不碰 lethal」的含义就是「离障碍物中心 ≥0.28 m」——拉直后的直线正好贴着这个边界走。
用脚本复刻整条链路实测（`docs` 同目录的思路见下）：

| 参数 | A* 本体最小留白 | **旧拉直后最小留白** |
|---|---|---|
| 0.28 / 0.55 | 0.55 m | **0.269 ~ 0.292 m** |
| 0.45 / 0.85 | 0.85 m | 0.447 ~ 0.472 m |

自车半径 0.25，所以旧配置的实际余量只有 **2~4 cm**。

**② 全局留白比下游 MPPI 的碰撞包络小**

`finenav_mppi_controller` 的 `CostCritic` 用 `IMapView::costAtPose`/`isCollision`，
它们是「中心 + 4 角」5 点压在本地障碍格上判碰：

```
对角方向需求 = sqrt(l_half² + w_half²)
```

原 `l_half: 0.28 / w_half: 0.25` → 0.375 m，再加本地栅格 0.1 m 的离散误差 ~0.05
→ **实际要求 ~0.42 m**，而全局路径只给 0.27~0.29 m。于是路径一进本地地图，
所有采样轨迹都被判成碰撞。

**③ 后果：`Optimizer::fallback()` 抛异常 → 进程 terminate**

```cpp
// optimizer.hpp:604-620
if (++counter > settings_.retry_attempt_limit) {   // retry_attempt_limit = 1
  counter = 0;
  throw std::runtime_error("Optimizer fail to compute path");
}
```

这个 throw 发生在 `evalControl()` 里，而 `evalControl` 是在 `controller.cpp` 的
`controlLoop()`（`std::thread`）中调用的 —— **没有 catch，就是 `std::terminate`**。
所以连续两个周期「所有轨迹都碰撞」= 整个导航进程直接死掉，现象就是「卡死」。

**修法（三处一起改才算数）**

1. `planner.cpp`：`lineOfSight` 改成 **超覆盖（supercover）+ clearance 距离场**：
   - 用 `clearance >= inscribed + smooth_clearance_margin` 判，而不是只判 lethal；
   - 斜走时额外检查两个正交邻格（普通 Bresenham 会从两个障碍的夹角里穿过去）。
2. `nav_sim.yaml`：
   - `inscribed_radius 0.28 → 0.45`、`inflation_radius 0.55 → 0.85`
     （0.45 = 本地包络 0.40 + 5 cm 余量）；新增 `smooth_clearance_margin: 0.05`；
   - `SimpleMap` 的足迹改成真实车体：`l_half: 0.25, w_half: 0.25, robot_radius: 0.25`
     （原来是 0.28/0.25，等于假装车是 0.56×0.50 的方盒子）。
3. `controller.cpp`：`evalControl` 外面包 `try/catch`，失败时发零速度停车 + 限流日志，
   不再让一个规划失败把进程带走。

另外顺手修了两处会让「到点前卡住」的地方：

- `toPath` 原来把终点无条件改写成**用户点的原始目标**。目标落在膨胀带里时，
  `nearestFree` 已经把它挪到可通行格，但终点又被改回去 → 最后一段直接插进 lethal 带
  → 同样触发全套碰撞。现在用 `plan_goal_`（目标格可通行才用原始目标）。
- `nearestFree` 原来返回环上扫描到的第一格（环的左上角），现在按欧氏距离取环内最近格。

**怎么复现这些数字**：仓库里放了 `tools/astar_probe.py`，它把 `map.pgm` 读进来，
按 `buildInflation` 的规则重建 EDT + lethal + multiplier，跑同样的 8 邻域 A*，
再分别按「旧拉直（只判 lethal + 普通 Bresenham）」和「新拉直（超覆盖 + clearance）」
拉直，沿折线以 0.5 格步长采样查 `clearance` 最小值：

```bash
python3 tools/astar_probe.py 0.45 0.85 3.0    # inscribed inflation penalty_weight
```

改参数前后各跑一次，直接对比最小留白，比在 rviz 里目测靠谱。
（`RES/ORIGIN` 写死在脚本头部，换地图记得同步。）

---

## 9. 变更记录

- 2026-10-28：建立本文档；记录构建链路、`package.yaml`、节点写法、ROS 消息链接坑；完成 `livox_msg` → `livox_ros_driver2` 迁移（`livox_driver2` / `FAST_LIO` / `pointcloud_converter` 三个包）。
- 2026-10-28：`ros_bridge` 也改为只声明 `livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp` + `...__rosidl_typesupport_cpp` 两个目标（不再走 `fins_optional_ros_dependency` 的兜底链接）。
- 2026-10-28：新增 `docs/apt-package-livox-msgs.md` 记录 deb 打成 apt 源的方案（本地 flat 源 / reprepro 签名源 / CI 直接装）。四个包的链接目标保持写死 `livox_ros_driver2::livox_interfaces2__rosidl_*`（已核对自己打的 msg-only deb 也沿用该命名，不需要目标名兼容层）。
- 2026-10-28：补 §3.5「把内部槽位透出成 ROS 话题」；`launch/sensor.py` 里挂 `ImuPublisher` / `LivoxCustomMsgPublisher`，把原始消息透出到 `/livox/imu`、`/livox/lidar`。
- 2026-10-28：`finenav_astar_planner` 从直线插值重写为栅格地图 A*（含膨胀/车体约束），新增 `launch/simulation.py` 把 A*/MPPI/terrain_map 接进 Gazebo 仿真；新增 §7（参数文件解析规则，读 `parameter_server.cpp` 确认：**嵌套可以，Studio 模板格式不行**）、§8（仿真导航链路）。`launch/simulation_config/nav_sim.yaml` 为仿真参数。
- 2026-10-28：修 `simulation.py` 启动报 `Node class 'common/OdometrySubscriber@default' not found` —— `Node(...)` 原本写在模块级 `DATAFLOW` 常量里，import 阶段就构造完，`source` 停在默认值 `"common"`；改成在 `simulation_group()` 内部构造、并在 `with DefaultSource():` 里调用。补 §8.2（source/节点全名三段式）与 §8.3（`fins build` 不带参数只编 `workspace.yaml` 的包）。
- 2026-10-28：修 A* 读 PGM 报 `short read of pgm pixel data` —— `nextToken()` 已消费 token 结尾空白，P5 分支又多吃一个字节，把第一个像素吃掉（406×406 图差 1 字节）。改成 `peek()` 定位 + 精确消费一个空白。详见 §8.1 的「PGM 头部空白陷阱」。
- 2026-10-28：修 `PathAngleCritic` 读 `vx_min` 前缀错误（上游 bug）：用的是 `finenav_mppi_controller.PathAngleCritic` 前缀的 loader，查不到项目顶层的 `vx_min`，`reversing_allowed_` 恒为 false（允许倒车的底盘也被当成只能前进）。改为和 `ConstraintCritic` 一样另开 `ParamLoader("finenav_mppi_controller")`。另补 `nav_sim.yaml` 里缺失的 `VelocityDeadbandCritic.deadband_velocities`。
- 2026-10-28：修仿真里 `wz` 剧烈上下摆动。主因是上一轮给 `VelocityDeadbandCritic` 补参数时把 `db_wz` 抬到 0.2、`weight 35`（直行样本白付 21 的代价，是对手的 6 倍，于是被逼着「必须转」、方向由噪声决定）。改为关掉该 critic；同时 `wz_max 0.8→0.60`、`vx_max 0.35→0.30`（对齐 simple_nav 的 0.3/0.6）、`wz_std 0.25→0.15`、`vx_std 0.10→0.08`、`temperature 0.3→0.5`、`batch_size 1000→2000`。新增 §8.4 记录完整推导。
- 2026-10-28：修「全局路径擦着障碍物边缘过、到障碍旁卡死」。三层原因：视线拉直只看 `isLethal` 把路径拉回 lethal 边界（实测 0.269~0.292 m）；`inscribed 0.28` 小于 MPPI 5 点足迹的对角需求 ~0.42；`Optimizer::fallback()` 抛 `runtime_error` 而控制线程没 catch → `std::terminate` 直接崩进程（这才是「卡死」的直接原因）。改动：`lineOfSight` 改超覆盖 + clearance 判据、`inscribed 0.45/inflation 0.85` + 新增 `smooth_clearance_margin`、SimpleMap 足迹改 0.25/0.25、`planLocked` 用 `plan_goal_`（目标在膨胀带里时不再把终点改回原始点）、`nearestFree` 取环内欧氏最近格、`controller.cpp` 包 try/catch 失败发零速度。新增 §8.5。
