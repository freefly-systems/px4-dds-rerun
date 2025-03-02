#!/usr/bin/env python3

import numpy as np
import rclpy
import rerun as rr
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


class RerunViz(Node):
    def __init__(self):
        super().__init__("px4_ros_rerun_node")

        rr.init("px4_ros_rerun")

        # For storing the current position and quaternion:
        self.pos = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

        # Subscribe to local position
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_pos_callback,
            10,
        )

        # Subscribe to attitude quaternion
        self.att_sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.att_callback, 10
        )

        # Periodic timer to log data to the Rerun viewer
        self.frame_id = 0
        self.timer = self.create_timer(0.1, self.update_rerun)

    def local_pos_callback(self, msg):
        # Store the local NED position
        self.pos[0] = msg.x
        self.pos[1] = msg.y
        self.pos[2] = msg.z

    def att_callback(self, msg):
        # Store the vehicle's attitude quaternion
        # px4_msgs VehicleAttitude has msg.q[4] as [w, x, y, z]
        self.q = np.array(msg.q)

    def update_rerun(self):
        # Update frame index or time
        rr.set_time_sequence("frame_id", self.frame_id)
        self.frame_id += 1

        # Log the transform for the entire drone
        # This creates "drone" as a child node of "world"
        # so that any geometry under "drone/" will apply the same transform.
        rr.log_rigid3(
            "drone",
            parent="world",
            translation=self.pos.tolist(),  # [x, y, z]
            rotation_q=self.q.tolist(),  # [w, x, y, z]
        )

        # Draw 3 arrows to show body x, y, z axes.
        rr.log_arrow(
            "drone/x_axis", origin=[0, 0, 0], vector=[1, 0, 0], color=[255, 0, 0]
        )
        rr.log_arrow(
            "drone/y_axis", origin=[0, 0, 0], vector=[0, 1, 0], color=[0, 255, 0]
        )
        rr.log_arrow(
            "drone/z_axis", origin=[0, 0, 0], vector=[0, 0, 1], color=[0, 0, 255]
        )


def main(args=None):
    rclpy.init(args=args)
    node = DroneVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
