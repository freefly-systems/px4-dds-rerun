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
            durability=DurabilityPolicy.VOLATILE,
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

        # Local NED frame
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_DOWN, static=True)
        rr.log(
            "world/earth-ned",
            rr.Arrows3D(
                vectors=[[1, 0, 0], [0, 1, 0], [0, 0, -1]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            ),
            static=True,
        )

        # Ground plane
        rr.log(
            "world/ground",
            rr.Boxes3D(
                mins=[-100, -100, -1],
                sizes=[200, 200, 1],
                colors=[255, 255, 255],
                fill_mode="solid",
            ),
            static=True,
        )

        self.last_callback_time = self.get_clock().now()

    def local_pos_callback(self, msg: VehicleLocalPosition):
        if self.limit_fps():
            return
        rr.log("world/body-frd", rr.components.Translation3D(xyz=[msg.x, msg.y, msg.z]))

    def att_callback(self, msg: VehicleAttitude):
        if self.limit_fps():
            return
        rr.log("world/body-frd", rr.RotationQuat(xyzw=list(msg.q)))

    def limit_fps(self, fps: int = 60) -> bool:
        now = self.get_clock().now()
        if (now - self.last_callback_time).nanoseconds / 1e9 < (1.0 / fps):
            return 1
        self.last_callback_time = now
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = PX4RerunNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
