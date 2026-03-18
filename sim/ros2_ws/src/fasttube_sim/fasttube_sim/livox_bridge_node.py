#!/usr/bin/env python3
"""
ROS 2 node: Livox HAP bridge for FastTube.

Subscribes to a PointCloud2 topic (Gazebo LiDAR output) and
re-publishes the data as Livox HAP UDP packets to localhost:57000,
which is the port the Rust detection app listens on.

If you don't have ROS 2 installed, use the standalone sim/bridge.py instead.
"""

import socket
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


LIVOX_UDP_HOST = '127.0.0.1'
LIVOX_UDP_PORT = 57000


def make_livox_packet(points_mm: list) -> bytes:
    """Build a Livox HAP Cartesian-high UDP packet (same as bridge.py)."""
    n = len(points_mm)
    ts = int(time.time() * 1e9) & 0xFFFF_FFFF_FFFF_FFFF
    hdr  = struct.pack('<B',  0x01)
    hdr += struct.pack('<H',  36 + n * 14)
    hdr += struct.pack('<H',  0)
    hdr += struct.pack('<H',  n)
    hdr += struct.pack('<H',  0)
    hdr += struct.pack('<B',  0)
    hdr += struct.pack('<B',  0x01)
    hdr += struct.pack('<B',  0)
    hdr += b'\x00' * 12
    hdr += struct.pack('<I',  0)
    hdr += struct.pack('<Q',  ts)
    body = b''.join(
        struct.pack('<iiibb', x, y, z, min(255, max(0, r)), 0)
        for x, y, z, r in points_mm
    )
    return hdr + body


class LivoxBridgeNode(Node):
    def __init__(self):
        super().__init__('livox_bridge')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (LIVOX_UDP_HOST, LIVOX_UDP_PORT)

        self.declare_parameter('lidar_topic', '/lidar/points')
        topic = self.get_parameter('lidar_topic').value

        self.sub = self.create_subscription(
            PointCloud2, topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'Livox bridge: subscribing to {topic}, '
            f'forwarding to {LIVOX_UDP_HOST}:{LIVOX_UDP_PORT}')

    def cloud_callback(self, msg: PointCloud2):
        pts_raw = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))

        if not pts_raw:
            return

        # Convert ROS frame (x=fwd, y=left, z=up) →
        # Livox sensor frame (x=right, y=fwd, z=up)
        #   sensor_x = -ros_y
        #   sensor_y =  ros_x
        #   sensor_z =  ros_z
        pts_mm = []
        for (rx, ry, rz, intensity) in pts_raw:
            sx = int(-ry * 1000)
            sy = int( rx * 1000)
            sz = int( rz * 1000)
            r  = int(min(255, max(0, intensity)))
            pts_mm.append((sx, sy, sz, r))

        pkt = make_livox_packet(pts_mm)
        self.sock.sendto(pkt, self.target)
        self.get_logger().debug(f'Forwarded {len(pts_mm)} points')


def main(args=None):
    rclpy.init(args=args)
    node = LivoxBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
