#!/usr/bin/env python3
"""Expose the real lift service to Isaac's standard-message ROS bridge."""

from __future__ import annotations

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64MultiArray

from lifting_msg_pkg.srv import LiftMotorSrv


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class SimLiftServiceBridge(Node):
    """Translate LiftMotorSrv into a process-private standard ROS message."""

    def __init__(self) -> None:
        super().__init__("ammr_sim_lift_service_bridge")
        self.publisher = self.create_publisher(
            Int64MultiArray,
            "/ammr/sim/internal/lift_command",
            reliable_qos(),
        )
        self.service = self.create_service(
            LiftMotorSrv,
            "/LiftingMotorService",
            self._handle_command,
        )

    def _handle_command(self, request, response):
        mode = int(request.mode)
        value = int(request.val)
        if mode not in {-4, -2}:
            response.state = -1
            self.get_logger().warning(f"Rejected unsupported lift mode {mode}")
            return response
        if mode == -2:
            value = 0
        self.publisher.publish(Int64MultiArray(data=[mode, value]))
        response.state = 0
        return response


def main() -> int:
    rclpy.init()
    node = SimLiftServiceBridge()
    node.get_logger().info(
        "Virtual lift service ready on /LiftingMotorService "
        f"(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')})"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
