#!/usr/bin/env python3
"""
Map Server — 监听地图文件变化并以 transient_local QoS 发布到 /map 话题.

用法:
    python3 map_server.py [map_dir] [topic_name]

默认值:
    map_dir   = /home/fins/Map/YunJing_Indoor/map
    topic_name = /map
"""

import os
import sys
import math
import time
import yaml
import struct
import hashlib
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

try:
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    _has_explicit_qos = True
except ImportError:
    _has_explicit_qos = False


def read_pgm(pgm_path: str):
    """读取 PGM (P5 灰度图) 文件，返回 (width, height, data_bytes)."""
    with open(pgm_path, "rb") as f:
        header = f.readline().strip()
        if header not in (b"P5", b"P2"):
            raise ValueError(f"不支持的 PGM 格式: {header}")

        # 跳过注释行
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()

        width, height = map(int, line.split())
        max_val_line = f.readline().strip()
        while max_val_line.startswith(b"#"):
            max_val_line = f.readline().strip()
        max_val = int(max_val_line)

        data = f.read()

    return width, height, max_val, data


def load_map(map_dir: str):
    """
    从目录加载地图，返回两个字典:
        info:   {width, height, resolution, origin_x, origin_y, negate, free_thresh, occupied_thresh}
        image_hash: pgm 文件的 sha256
    """
    yaml_path = os.path.join(map_dir, "map.yaml")
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    image_rel = meta["image"]
    pgm_path = os.path.join(map_dir, image_rel)

    width, height, max_val, raw = read_pgm(pgm_path)
    image_hash = hashlib.sha256(raw).hexdigest()

    # 将原始字节转为 OccupancyGrid 的 data (0-100)
    # PGM: 黑(0)=占用, 白(max_val)=空闲
    # OccupancyGrid: 0=空闲, 100=占用, -1=未知
    occupancy = bytearray(width * height)
    if max_val == 0:
        max_val = 1  # 防止除零

    negate = int(meta.get("negate", 0))
    free_thresh = int(meta.get("free_thresh", 0.196) * 100)
    occupied_thresh = int(meta.get("occupied_thresh", 0.65) * 100)

    for y in range(height):
        # PGM row 0 = top of image, but OccupancyGrid row 0 = bottom of map.
        # Flip vertically so the image top maps to the highest world-y.
        dest_y = height - 1 - y
        for x in range(width):
            src_idx = y * width + x
            dst_idx = dest_y * width + x
            # P5: 每个像素 1 字节 (max_val <= 255), 2 字节 (max_val > 255)
            if max_val <= 255:
                p = raw[src_idx]
            else:
                p = struct.unpack_from(">H", raw, src_idx * 2)[0]

            # 归一化到 [0, 100], 黑→0, 白→100
            pct = int(p / max_val * 100.0)

            # 默认: 黑(0)=占用(100), 白(100)=空闲(0)
            # negate=1 时反过来
            if negate:
                scaled = pct          # 黑→0(空闲), 白→100(占用)
            else:
                scaled = 100 - pct    # 黑→100(占用), 白→0(空闲)

            if scaled <= free_thresh:
                occupancy[dst_idx] = 0       # 自由
            elif scaled >= occupied_thresh:
                occupancy[dst_idx] = 100     # 占用
            else:
                occupancy[dst_idx] = -1      # 未知

    origin = meta.get("origin", [0.0, 0.0, 0.0])
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0

    return {
        "width": width,
        "height": height,
        "resolution": float(meta["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "origin_yaw": origin_yaw,
        "negate": negate,
        "free_thresh": float(meta.get("free_thresh", 0.196)),
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
    }, image_hash, bytes(occupancy)


class MapServerNode(Node):
    def __init__(self, map_dir: str, topic: str = "/map", watch_interval: float = 1.0):
        super().__init__("map_server")

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._publisher = self.create_publisher(OccupancyGrid, topic, qos)
        self._map_dir = map_dir
        self._watch_interval = watch_interval
        self._last_image_hash: str | None = None
        self._last_yaml_hash: str | None = None
        self._frame_id = "map"

        self.get_logger().info(f"MapServer 启动, 目录={map_dir}, 话题={topic}")
        self.get_logger().info(f"QoS: transient_local + reliable + keep_last(1)")

        self._publish_map()
        self._timer = self.create_timer(watch_interval, self._check_and_republish)

    def _check_and_republish(self):
        """定时检查文件是否变化, 有变化则重新发布."""
        yaml_path = os.path.join(self._map_dir, "map.yaml")

        if not os.path.exists(yaml_path):
            return

        try:
            with open(yaml_path, "rb") as f:
                yaml_hash = hashlib.sha256(f.read()).hexdigest()
        except Exception:
            return

        if yaml_hash != self._last_yaml_hash:
            self.get_logger().info("检测到 map.yaml 变化, 重新发布")
            self._publish_map()
            return

        # 也检查 pgm 是否被替换 (yaml 不变但 pgm 内容变了)
        try:
            with open(yaml_path, "r") as f:
                meta = yaml.safe_load(f)
            pgm_path = os.path.join(self._map_dir, meta["image"])
            with open(pgm_path, "rb") as f:
                pgm_hash = hashlib.sha256(f.read()).hexdigest()
        except Exception:
            return

        if pgm_hash != self._last_image_hash:
            self.get_logger().info("检测到 PGM 文件变化, 重新发布")
            self._publish_map()

    def _publish_map(self):
        try:
            info, image_hash, occupancy = load_map(self._map_dir)
        except Exception as e:
            self.get_logger().error(f"加载地图失败: {e}")
            return

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.info.resolution = info["resolution"]
        msg.info.width = info["width"]
        msg.info.height = info["height"]
        msg.info.origin = Pose()
        msg.info.origin.position.x = info["origin_x"]
        msg.info.origin.position.y = info["origin_y"]
        msg.info.origin.position.z = 0.0
        # yaw → quaternion
        half_yaw = info["origin_yaw"] * 0.5
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = math.sin(half_yaw)
        msg.info.origin.orientation.w = math.cos(half_yaw)

        msg.data = list(occupancy)

        self._publisher.publish(msg)

        # 也记录 yaml 哈希
        yaml_path = os.path.join(self._map_dir, "map.yaml")
        with open(yaml_path, "rb") as f:
            self._last_yaml_hash = hashlib.sha256(f.read()).hexdigest()
        self._last_image_hash = image_hash

        self.get_logger().info(
            f"地图已发布 — {info['width']}x{info['height']}, "
            f"分辨率={info['resolution']:.3f} m/pix"
        )


def main():
    rclpy.init(args=sys.argv)

    map_dir = sys.argv[1] if len(sys.argv) > 1 else "/home/fins/Map/YunJing_Indoor/map"
    topic = sys.argv[2] if len(sys.argv) > 2 else "/map"

    node = MapServerNode(map_dir=map_dir, topic=topic, watch_interval=1.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
