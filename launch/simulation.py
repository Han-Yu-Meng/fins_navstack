"""Gazebo 仿真里的导航链路（launch/simulation.py）

依赖仿真侧（active_place_recognition_sim）已经提供的东西：

  /Odometry               nav_msgs/Odometry                       绝对位姿 + 体坐标系速度
                          pose 在 odom 帧（map == odom，identity），twist 由真值差分
  /localization_pose      geometry_msgs/PoseWithCovarianceStamped 同一位姿（本链路不用，见文末）
  /cloud_registered_body  sensor_msgs/PointCloud2                 base_link 系点云
  /cmd_vel                geometry_msgs/Twist                     差速底盘速度指令
  TF: map -> odom (identity), odom -> base_link

本文件把这些 ROS 话题接进 fins 图里：

  /Odometry               --[OdometrySubscriber]--> nav/odom
  /cloud_registered_body  --[PointCloudSubscriber]--> finenav_terrain_map
  /goal_pose              --[PoseStampedSubscriber]--> finenav_astar_planner

  nav/odom + nav/goal        --> finenav_astar_planner   --> nav/global_path
  nav/odom + nav/global_path --> finenav_mppi_controller --> nav/cmd_vel
  nav/cmd_vel                --> [TwistPublisher] --> /cmd_vel

  finenav_mppi_controller --(服务)--> finenav_terrain_map
      get_rolling_grid_data / get_base_frame_id：MPPI 的 CostCritic 靠这两个
      服务向地图节点要「以机器人为中心的滚动栅格」，服务名必须在下面
      clients / servers 里成对出现，否则 MPPI 拿到的是默认构造的空地图。

用法（先起仿真，再起本脚本）：

    # 终端 1
    ros2 launch active_place_recognition_sim sim.launch.py rviz:=true
    # 终端 2（记得点 Gazebo 的播放按钮，暂停状态下什么都不发）
    cd ~/fins_navstack/launch
    python3 simulation.py

    # 终端 3：发目标点（也可以用 rviz 的 "2D Goal Pose"）
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
      "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}"

参数在 simulation_config/nav_sim.yaml（AstarPlanner / SimpleMap /
finenav_mppi_controller 三段）。
"""

import os

from fins import Agent, DefaultSource, Group, Node

# 可视化话题在 rviz 里怎么加：
#   /global_costmap  (Map)   膨胀后的全局代价图，看车体尺寸约束有没有生效
#   /global_plan     (Path)  A* 输出
#   /optimal_plan    (Path)  MPPI 预测轨迹
#   /local_costmap   (Map)   滚动局部栅格（由 /cloud_registered_body 实时生成）

# MPPI 与 terrain_map 之间的 fins 服务名（clients / servers 必须一致）
SVC_ROLLING_GRID = "terrain_map/get_rolling_grid_data"
SVC_BASE_FRAME = "terrain_map/get_base_frame_id"


def simulation_group():
    """构造整条链路。

    **必须在 `with DefaultSource():` 内部调用**（见文件末尾 __main__）。

    fins 的节点全名是 `{source}/{类名}@{version}`：source 在 `Node.__init__`
    里就定死了 —— 不显式传 `source=` 就取 `fins.core` 的模块级默认值
    `_CURRENT_DEFAULT_SOURCE = "common"`，只有进入 `DefaultSource()` 上下文
    才会改成自动探测到的**工作区名**（本仓库 = "fins_navstack"）。

    而插件里注册的 source 是 `fins build` 注入的编译宏 `PKG_SOURCE`
    （`build/<pkg>/CMakeLists.txt`: `add_compile_definitions(PKG_SOURCE="${FINS_META_SOURCE}")`），
    也就是工作区名。所以如果像以前那样把 `Node(...)` 写在模块级常量里，
    import 阶段就构造完了，source 永远是 "common"，Agent 会去找
    `common/OdometrySubscriber@default` 并报 `Node class not found`。
    """
    return Group([
        # ---------------------------------------------------------- ROS -> fins
        # 仿真提供的唯一里程计源：位姿 + 速度都在里面，不需要再从
        # /localization_pose 差分出一个假的 twist（差分会引入延迟和尖峰）
        Node(
            package="ros_bridge",
            name="OdometrySubscriber",
            parameters={
                "topic": "/Odometry",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            outputs={"msg": "nav/odom"},
        ),
        # 点云是 base_link 系的（/cloud_registered_body），terrain_map 会自己
        # 用 odom 的 yaw 把它转到地图系
        Node(
            package="ros_bridge",
            name="PointCloudSubscriber",
            parameters={
                "topic": "/cloud_registered_body",
                "history": "Keep Last",
                "depth": "5",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            outputs={"msg": "sim/cloud_body"},
        ),
        # 目标点：rviz 的 "2D Goal Pose" 默认发 /goal_pose。
        # 用 Transient Local 是为了目标点晚于连接时还能收到最后一帧
        Node(
            package="ros_bridge",
            name="PoseStampedSubscriber",
            parameters={
                "topic": "/goal_pose",
                "history": "Keep Last",
                "depth": "1",
                "reliability": "Reliable",
                "durability": "Transient Local",
            },
            outputs={"msg": "nav/goal"},
        ),
        # ---------------------------------------------------------- 感知 / 规划
        # 注意顺序：terrain_map 必须排在 MPPI 前面，让它的服务端先注册出来
        Node(
            package="finenav_terrain_map",
            name="SimpleMapNode",
            inputs={
                "point_cloud": "sim/cloud_body",
                "odom": "nav/odom",
            },
            outputs={"map": "nav/local_map"},
            servers={
                "get_rolling_grid_data": SVC_ROLLING_GRID,
                "get_base_frame_id": SVC_BASE_FRAME,
            },
        ),
        Node(
            package="finenav_astar_planner",
            name="AstarPlannerNode",
            inputs={
                "odom": "nav/odom",
                "goal": "nav/goal",
            },
            outputs={
                "path": "nav/global_path",
                "costmap": "nav/costmap",
            },
        ),
        # ---------------------------------------------------------- 控制
        Node(
            package="finenav_mppi_controller",
            name="MPPIControllerNode",
            inputs={
                "odom": "nav/odom",
                "reference_path": "nav/global_path",
            },
            outputs={
                "cmd_vel": "nav/cmd_vel",
                "optimal_path": "nav/optimal_path",
            },
            clients={
                "get_rolling_grid_data": SVC_ROLLING_GRID,
                "get_base_frame_id": SVC_BASE_FRAME,
            },
        ),
        # ---------------------------------------------------------- fins -> ROS
        Node(
            package="ros_bridge",
            name="TwistPublisher",
            parameters={
                "topic": "/cmd_vel",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "nav/cmd_vel"},
        ),
        Node(
            package="ros_bridge",
            name="PathPublisher",
            parameters={
                "topic": "/global_plan",
                "history": "Keep Last",
                "depth": "1",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "nav/global_path"},
        ),
        Node(
            package="ros_bridge",
            name="PathPublisher",
            parameters={
                "topic": "/optimal_plan",
                "history": "Keep Last",
                "depth": "1",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "nav/optimal_path"},
        ),
        Node(
            package="ros_bridge",
            name="OccupancyGridPublisher",
            parameters={
                "topic": "/global_costmap",
                "history": "Keep Last",
                "depth": "1",
                "reliability": "Reliable",
                "durability": "Transient Local",
            },
            inputs={"msg": "nav/costmap"},
        ),
        Node(
            package="ros_bridge",
            name="OccupancyGridPublisher",
            parameters={
                "topic": "/local_costmap",
                "history": "Keep Last",
                "depth": "1",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "nav/local_map"},
        ),
    ])


if __name__ == "__main__":
    DIR = os.path.dirname(os.path.abspath(__file__))

    with Agent(name="nav_sim") as agent:
        # 用绝对路径，避免必须 cd 到 launch/ 才能启动
        agent.add_config(os.path.join(DIR, "simulation_config", "nav_sim.yaml"))

        # 顺序很重要：simulation_group() 必须在 with 块里调用，
        # 这样 Node 才是在 source 已经确定的前提下构造的。
        with DefaultSource():
            agent.launch(simulation_group())

        # 想让 Agent 崩在 GDB 里并打印调用栈时再打开（需要 apt install gdb）：
        # agent.enable_debugging(full_backtrace=True)
        #
        # 注意：不要调用 agent.enable_performance_monitor()，
        # 当前 fins_py（/usr/local/lib/python3.10/dist-packages/fins）里没有这个方法，
        # 调用会抛 AttributeError，而此时 Agent 已经在跑，atexit 会把它一起带走。

        agent.spin()

# ---------------------------------------------------------------------------
# 关于 /localization_pose
#
# 仿真还会发 /localization_pose（PoseWithCovarianceStamped，和 /Odometry 的 pose
# 是同一个真值位姿）。这条链路不用它：/Odometry 已经把「位姿 + 速度」都给了，
# 再接一份位姿只会多一个可能和 odom 不同步的源。需要位姿协方差的下游（例如
# 传感器融合、或者想在 rviz 里单独看定位）时再往 simulation_group() 里加：
#
#     Node(
#         package="ros_bridge",
#         name="PoseWithCovarianceStampedSubscriber",
#         parameters={
#             "topic": "/localization_pose",
#             "history": "Keep Last",
#             "depth": "10",
#             "reliability": "Reliable",
#             "durability": "Volatile",
#         },
#         outputs={"msg": "sim/localization"},
#     ),
# ---------------------------------------------------------------------------
