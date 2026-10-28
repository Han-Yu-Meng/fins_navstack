from fins import Node, Group
import os

def sensor_group():
    DIR = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(DIR, "config", "MID360_config.json")

    return Group([
        Node(
            package="livox_driver2",
            name="LivoxDriverNode",
            parameters={
                "config_path": config_path,
                "publish_freq": 10,
                "frame_id": "livox_frame"
            },
            outputs={
                "imu": "sensor/imu",
                "lidar": "sensor/lidar",
                "lidar_standard": "sensor/lidar_standard"
            },
        ),
        Node(
            package="ros_bridge",
            name="ImuPublisher",
            parameters={
                "topic": "/livox/imu",
                "history": "Keep Last",
                "depth": "200",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "sensor/imu"},
        ),
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
        Node(
            package="ros_bridge",
            name="TransformRPY",
            parameters={
                "tx": 0.057, "ty": 0.083, "tz": 0.31,
                "roll": 180, "pitch": -132, "yaw": 0,
                "from_frame": "base_link",
                "to_frame": "base_lidar"
            },
            outputs={
                "transform": "tf/base_link_to_base_lidar"
            },
        ),
        Node(
            package="ros_bridge",
            name="StaticBroadcaster",
            inputs={
                "transform": "tf/base_link_to_base_lidar"
            },
        ),
    ])
