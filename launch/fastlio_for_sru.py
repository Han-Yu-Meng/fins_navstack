from fins import Node, Group, Agent, DefaultSource
from sensor import sensor_group
from executor import executor_group

def fastlio_group():
    return Group([
        Node(
            package="FAST_LIO",
            name="FastLIO",
            inputs={
                "imu": "sensor/imu",
                "lidar": "sensor/lidar",
                "$T_{base}^{lidar}$": "tf/base_link_to_base_lidar"
            },
            outputs={
                "cloud": "lio/cloud",
                "cloud_body": "lio/cloud_body",
                "path": "lio/path",
                "odometry": "lio/odometry",
                "$T_{odom}^{base}$": "tf/odom_to_base_link"
            },
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={"pcl_cloud": "lio/cloud"},
            outputs={"ros_cloud": "lio/ros_cloud"}
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            parameters={
                "topic": "/cloud_registered",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "lio/ros_cloud"}
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={"pcl_cloud": "lio/cloud_body"},
            outputs={"ros_cloud": "lio/ros_cloud_body"}
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            parameters={
                "topic": "/cloud_registered_body",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "lio/ros_cloud_body"}
        ),
        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            parameters={
                "from_frame_override": "odom",
                "to_frame_override": "base_link"
            },
            inputs={"transform": "tf/odom_to_base_link"}
        ),
        Node(
            package="ros_bridge",
            name="OdometryPublisher",
            parameters={
                "topic": "/dlio/odom_node/odom",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
            inputs={"msg": "lio/odometry"}
        )
    ])

if __name__ == "__main__":
    with Agent(name="fastlio") as agent:
        agent.add_config("config/fastlio_mid360.yaml")
        with DefaultSource():
            agent.launch(
                sensor_group(),
                fastlio_group(),
                executor_group()
            )
        agent.enable_debugging(full_backtrace=True)
        agent.spin()
