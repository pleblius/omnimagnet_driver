#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from omnimagnet_interfaces.msg import Vector3, ErrorMessage, FinishedMessage
from omnimagnet_interfaces.srv import (
    SingleMagnetConstant,
    SingleMagnetRotation,
    MultiMagnetConstant,
    MultiMagnetRotation,
    SingleCurrentConstant,
    SingleCurrentRotation,
    MultiCurrentConstant,
    MultiCurrentRotation,
    DriverReset,
)


def vec(x, y, z):
    v = Vector3()
    v.x = float(x)
    v.y = float(y)
    v.z = float(z)
    return v


class DriverTester(Node):
    def __init__(self):
        super().__init__("omnimagnet_driver_tester")

        self.create_subscription(ErrorMessage, "driver_errors", self.error_cb, 10)
        self.create_subscription(FinishedMessage, "driver_finished", self.finished_cb, 10)

        self.smc = self.create_client(SingleMagnetConstant, "single_magnet_constant")
        self.smr = self.create_client(SingleMagnetRotation, "single_magnet_rotation")
        self.mmc = self.create_client(MultiMagnetConstant, "multi_magnet_constant")
        self.mmr = self.create_client(MultiMagnetRotation, "multi_magnet_rotation")
        self.scc = self.create_client(SingleCurrentConstant, "single_current_constant")
        self.scr = self.create_client(SingleCurrentRotation, "single_current_rotation")
        self.mcc = self.create_client(MultiCurrentConstant, "multi_current_constant")
        self.mcr = self.create_client(MultiCurrentRotation, "multi_current_rotation")

        self.reset = self.create_client(DriverReset, "reset_driver")

    def error_cb(self, msg):
        self.get_logger().error(f"DRIVER ERROR: shutdown={msg.shutdown}, {msg.error_desc}")

    def finished_cb(self, msg):
        self.get_logger().info(f"FINISHED: {msg.msg}")

    def wait_for_services(self):
        for client, name in [
            (self.smc, "single_magnet_constant"),
            (self.smr, "single_magnet_rotation"),
            (self.mmc, "multi_magnet_constant"),
            (self.mmr, "multi_magnet_rotation"),
            (self.scc, "single_current_constant"),
            (self.scr, "single_current_rotation"),
            (self.mcc, "multi_current_constant"),
            (self.mcr, "multi_current_rotation"),
            (self.reset, "reset_driver"),
        ]:
            self.get_logger().info(f"Waiting for {name}...")
            client.wait_for_service()

    def call(self, client, request, label):
        self.get_logger().info(f"\n=== {label} ===")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error(f"{label}: no response")
            return False

        if hasattr(response, "error") and response.error:
            self.get_logger().error(f"{label}: ERROR: {response.error_desc}")
            return False

        self.get_logger().info(f"{label}: OK")
        return True

    def reset_driver(self):
        req = DriverReset.Request()
        return self.call(self.reset, req, "Reset driver")

    def run_tests(self):
        self.wait_for_services()
        
        # 1. Reset first
        self.reset_driver()
        time.sleep(0.5)

        duration = 10.0
        vect = vec(1.0, 1.0, 1.0)

        # 2. SMC
        req = SingleMagnetConstant.Request()
        req.omnimagnet = 5
        req.dipole_vec = vect
        req.dipole_strength = 40.0
        req.duration = duration
        self.call(self.smc, req, "SMC")
        time.sleep(duration)

        # 3. SMR
        req = SingleMagnetRotation.Request()
        req.omnimagnet = 5
        req.rotation_vector = vect
        req.dipole_strength = 40.0
        req.rotation_freq = 5.0
        req.phase_offset = 0.0
        req.duration = duration
        self.call(self.smr, req, "SMR")
        time.sleep(duration)

        # 4. SCC
        req = SingleCurrentConstant.Request()
        req.omnimagnet = 5
        req.current_vec = vect
        req.current_strength = 15.0
        req.duration = duration
        self.call(self.scc, req, "SCC")
        time.sleep(duration)

        # 5. SCR
        req = SingleCurrentRotation.Request()
        req.omnimagnet = 5
        req.rotation_vector = vect
        req.rotation_freq = 5.0
        req.phase_offset = 0.0
        req.current_strength = 15.0
        req.duration = duration
        self.call(self.scr, req, "SCR")
        time.sleep(duration)

        # 6. MMC
        req = MultiMagnetConstant.Request()
        req.omnimagnets = [1, 2, 3, 4, 5]
        req.dipole_strengths = [40.0]
        req.dipole_vecs = [vect]
        req.duration = duration
        self.call(self.mmc, req, "MMC")
        time.sleep(duration)

        # 7. MMR
        req = MultiMagnetRotation.Request()
        req.omnimagnets = [1, 2, 3, 4, 5]
        req.dipole_strengths = [40.0]
        req.phase_offsets = [0.0]
        req.rotation_freqs = [5.0]
        req.duration = duration
        req.rotation_vectors = [vect]
        self.call(self.mmr, req, "MMR")
        time.sleep(duration)

        # 8. MCC
        req = MultiCurrentConstant.Request()
        req.omnimagnets = [1, 2, 3, 4, 5]
        req.current_strengths = [1.0]
        req.current_vecs = [vect]
        req.duration = duration
        self.call(self.mcc, req, "MCC")
        time.sleep(duration)

        # 9. MCR
        req = MultiCurrentRotation.Request()
        req.omnimagnets = [1, 2, 3, 4, 5]
        req.current_strengths = [1.0]
        req.rotation_vectors = [vect]
        req.rotation_freqs = [5.0]
        req.phase_offsets = [0.0]
        req.duration = duration
        self.call(self.mcr, req, "MCR")
        time.sleep(duration)

        # Final reset
        self.reset_driver()
        self.get_logger().info("Test sequence complete.")


def main():
    rclpy.init()
    node = DriverTester()

    try:
        node.run_tests()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()