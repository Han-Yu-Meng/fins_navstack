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

## 7. 变更记录

- 2026-10-28：建立本文档；记录构建链路、`package.yaml`、节点写法、ROS 消息链接坑；完成 `livox_msg` → `livox_ros_driver2` 迁移（`livox_driver2` / `FAST_LIO` / `pointcloud_converter` 三个包）。
- 2026-10-28：`ros_bridge` 也改为只声明 `livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp` + `...__rosidl_typesupport_cpp` 两个目标（不再走 `fins_optional_ros_dependency` 的兜底链接）。
- 2026-10-28：新增 `docs/apt-package-livox-msgs.md` 记录 deb 打成 apt 源的方案（本地 flat 源 / reprepro 签名源 / CI 直接装）。四个包的链接目标保持写死 `livox_ros_driver2::livox_interfaces2__rosidl_*`（已核对自己打的 msg-only deb 也沿用该命名，不需要目标名兼容层）。
- 2026-10-28：补 §3.5「把内部槽位透出成 ROS 话题」；`launch/sensor.py` 里挂 `ImuPublisher` / `LivoxCustomMsgPublisher`，把原始消息透出到 `/livox/imu`、`/livox/lidar`。
