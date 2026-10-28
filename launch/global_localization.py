import os
import sys
import subprocess
import signal

from fins import Node, Group, Agent, DefaultSource
from sensor import sensor_group
from fastlio import fastlio_group

MAP_SERVER_PROC = None

def global_localization_group():
    return Group([
        Node(
            package="global_localization",
            name="GlobalLocalization",
            inputs={
                "cloud": "lio/cloud",
                "$T_{odom}^{baselink}$": "tf/odom_to_base_link",
            },
            outputs={
                "global_map_viz": "global/map_viz",
                "aligned_cloud": "global/aligned_cloud",
                "$T_{map}^{odom}$": "tf/map_to_odom",
                "current_pose": "/current_pose",
                "initial_pose": "/initial_pose"
            },
        ),

        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            parameters={
                "from_frame_override": "map",
                "to_frame_override": "odom"
            },
            inputs={
                "transform": "tf/map_to_odom"
            },
        ),
        Node(
            package="ros_bridge",
            name="PoseWithCovarianceStampedPublisher",
            parameters={
                "topic": "/localization_pose",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={
                "msg": "/current_pose"
            },
        ),
        Node(
            package="ros_bridge",
            name="PoseWithCovarianceStampedPublisher",
            parameters={
                "topic": "/initialpose",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={
                "msg": "/initial_pose"
            },
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={
                "pcl_cloud": "global/map_viz",
            },
            outputs={
                "ros_cloud": "global/ros_map"
            },
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            parameters={
                "topic": "/global_map_viz",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Transient Local",
            },
            inputs={
                "msg": "global/ros_map"
            },
        ),

        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={
                "pcl_cloud": "global/aligned_cloud",
            },
            outputs={
                "ros_cloud": "global/ros_aligned"
            },
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            parameters={
                "topic": "/aligned_cloud",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={
                "msg": "global/ros_aligned"
            },
        ),
    ])

def _start_map_server():
    """启动 map_server 子进程."""
    global MAP_SERVER_PROC
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_server_script = os.path.join(script_dir, "map_server.py")
    map_dir = os.environ.get(
        "MAP_DIR", "/home/fins/Map/YunJing_Indoor/map"
    )
    topic = os.environ.get("MAP_TOPIC", "/map")

    MAP_SERVER_PROC = subprocess.Popen(
        [sys.executable, map_server_script, map_dir, topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    print(f"[global_localization] map_server 已启动 (pid={MAP_SERVER_PROC.pid})")


def _stop_map_server():
    """停止 map_server 子进程."""
    global MAP_SERVER_PROC
    if MAP_SERVER_PROC is not None:
        print(f"[global_localization] 正在停止 map_server (pid={MAP_SERVER_PROC.pid})...")
        MAP_SERVER_PROC.send_signal(signal.SIGINT)
        try:
            MAP_SERVER_PROC.wait(timeout=5)
        except subprocess.TimeoutExpired:
            MAP_SERVER_PROC.kill()
            MAP_SERVER_PROC.wait()
        MAP_SERVER_PROC = None


if __name__ == "__main__":
    _start_map_server()
    try:
        with Agent(name="global_localization") as agent:
            agent.add_config("config/fastlio_mid360.yaml")
            agent.add_config("config/global_localization.yaml")
            agent.enable_timeline_monitor()
            # agent.enable_debugging(full_backtrace=True)
            with DefaultSource():
                agent.launch(
                    sensor_group(),
                    fastlio_group(),
                    global_localization_group()
                )
            agent.spin()
    finally:
        _stop_map_server()
