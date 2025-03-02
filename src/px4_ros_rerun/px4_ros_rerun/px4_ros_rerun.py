#!/usr/bin/env python3

import os

import numpy as np
import rclpy
import rerun as rr
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


class PX4RerunNode(Node):
    def __init__(self):
        super().__init__("px4_rerun_node")

        # Configure QoS profile for subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_pos_callback,
            qos_profile,
        )

        self.att_sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.att_callback, qos_profile
        )

        # Init Rerun
        rr.init("px4-ros-rerun")
        web_port = int(os.getenv("RR_WEBVIEWER_PORT", 9090))
        rr.serve_web(web_port=web_port)

    def local_pos_callback(self, msg):
        pass

    def att_callback(self, msg):
        print(msg.q)


def main(args=None):
    rclpy.init(args=args)
    node = PX4RerunNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
