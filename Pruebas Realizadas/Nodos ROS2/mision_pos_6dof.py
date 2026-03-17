#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

import numpy as np
import time

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

TAKEOFF_ALT = 2.0

# (duracion, x, y, z, roll, pitch, yaw, descripcion)
TEST_SEQUENCE = [
    # (duración, x, y, z, roll, pitch, yaw, descripción)

    (5.0,   0.0,   0.0, -2.0,  0.0, 0.0, 0.0, "Hover inicial"),

    (6.0,  20.0,   0.0, -2.0,  0.0, 0.0, 0.0, "Mover +X (20m)"),

    (6.0,  20.0,  20.0, -2.0,  0.0, 0.0, 0.0, "Mover diagonal +X +Y"),

    (6.0,   0.0,  20.0, -2.0,  0.0, 0.0, 0.0, "Mover -X"),

    (6.0,   0.0,   0.0, -2.0,  0.0, 0.0, 0.0, "Volver origen"),

    (6.0,  20.0,   0.0, -2.0,   np.radians(20), 0.0, 0.0, "Mover +X (20m)"),

    (6.0,  20.0,  20.0, -2.0,  0.0,  np.radians(20), 0.0, "Mover diagonal +X +Y"),

    (6.0,   0.0,  20.0, -2.0,  0.0, 0.0,  np.radians(20), "Mover -X"),

    (6.0,   0.0,   0.0, -2.0,  0.0,  np.radians(20),  np.radians(20), "Volver origen"),

    # Ahora metemos actitud desacoplada
    (6.0,   0.0,   0.0, -2.0,  np.radians(20), 0.0, 0.0, "Roll 20º en hover"),

    (6.0,   0.0,   0.0, -2.0,  0.0, np.radians(20), 0.0, "Pitch 20º en hover"),

    (6.0,   0.0,   0.0, -2.0,  0.0, 0.0, np.radians(45), "Yaw 45º"),
]


class OmniCommander(Node):

    OFFBOARD_HZ = 20.0

    STATE_PREFLIGHT = 0
    STATE_ARM_OFFBOARD = 1
    STATE_TAKEOFF = 2
    STATE_TEST = 3
    STATE_LAND = 4
    STATE_DONE = 5

    def __init__(self):
        super().__init__('omni_commander')

        self._state = self.STATE_PREFLIGHT
        self._state_start_time = self.get_clock().now()
        self._preflight_count = 0

        self._test_step = 0
        self._step_start_time = None

        self._z_origin = None

        self._local_pos = None
        self._vehicle_status = None

        # comandos
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        self._dt = 1.0 / self.OFFBOARD_HZ

        # pubs
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._pub_att = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', PX4_QOS)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # subs
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._local_pos_callback,
            PX4_QOS
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self._status_callback,
            PX4_QOS
        )

        self._timer = self.create_timer(self._dt, self._timer_callback)

        self.get_logger().info('OmniCommander iniciado')

    # ---------------- callbacks ----------------

    def _local_pos_callback(self, msg):
        self._local_pos = msg

    def _status_callback(self, msg):
        self._vehicle_status = msg

    # ---------------- main loop ----------------

    def _timer_callback(self):

        self._publish_offboard_mode()

        if self._state == self.STATE_PREFLIGHT:
            self._state_preflight()

        elif self._state == self.STATE_ARM_OFFBOARD:
            self._state_arm()

        elif self._state == self.STATE_TAKEOFF:
            self._state_takeoff()

        elif self._state == self.STATE_TEST:
            self._state_test()

        elif self._state == self.STATE_LAND:
            self._state_land()

        self._publish_trajectory()
        self._publish_attitude()

    # ---------------- states ----------------

    def _state_preflight(self):
        self._set_command(0, 0, 0, 0, 0, 0)
        self._preflight_count += 1

        if self._preflight_count > 40:
            self.get_logger().info("Activando Offboard + Arm")
            self._transition(self.STATE_ARM_OFFBOARD)

    def _state_arm(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(0.1)
        self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self._transition(self.STATE_TAKEOFF)

    def _state_takeoff(self):

        if self._local_pos is None:
            return

        if not np.isfinite(self._local_pos.z):
            return

        if self._z_origin is None:
            self._z_origin = self._local_pos.z
            self.get_logger().info(f"z_origin: {self._z_origin:.2f}")

        target_z = self._z_origin - TAKEOFF_ALT

        if not np.isfinite(self._local_pos.x) or not np.isfinite(self._local_pos.y):
            return

        self._set_command(
            self._local_pos.x,
            self._local_pos.y,
            target_z,
            0, 0, 0
        )

        if abs(self._local_pos.z - target_z) < 0.2:
            self._transition(self.STATE_TEST)

    def _state_test(self):

        if self._test_step >= len(TEST_SEQUENCE):
            self._transition(self.STATE_LAND)
            return

        duration, x, y, z, roll, pitch, yaw, desc = TEST_SEQUENCE[self._test_step]

        now = self.get_clock().now()

        if self._step_start_time is None:
            self._step_start_time = now
            self.get_logger().info(desc)

        elapsed = (now - self._step_start_time).nanoseconds * 1e-9

        self._set_command(x, y, z, roll, pitch, yaw)

        if elapsed > duration:
            self._test_step += 1
            self._step_start_time = None

    def _state_land(self):

        self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        if self._local_pos is None:
            return

        # cuando está cerca del suelo
        if abs(self._local_pos.z - self._z_origin) < 0.1:
            self.get_logger().info("En tierra → desarmando")

            self._cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,
                param2=21196.0
            )

            self._transition(self.STATE_DONE)


    # ---------------- helpers ----------------

    def _set_command(self, x, y, z, roll, pitch, yaw):
        self._x = float(x)
        self._y = float(y)
        self._z = float(z)
        self._roll = float(roll)
        self._pitch = float(pitch)
        self._yaw = float(yaw)

    def _transition(self, s):
        self._state = s
        self._state_start_time = self.get_clock().now()

    # ---------------- publishers ----------------

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._ts()
        msg.position = True
        msg.attitude = True
        self._pub_offboard.publish(msg)

    def _publish_trajectory(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self._ts()

        msg.position = [self._x, self._y, self._z]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3

        msg.yaw = float('nan')  # ⚠️ IMPORTANTE
        msg.yawspeed = float('nan')

        self._pub_traj.publish(msg)

    def _publish_attitude(self):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self._ts()

        q = self._euler_to_quat(self._roll, self._pitch, self._yaw)

        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.thrust_body = [0.0, 0.0, -1.0]

        self._pub_att.publish(msg)

    def _cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._ts()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)

    # ---------------- utils ----------------

    def _euler_to_quat(self, r, p, y):
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)

        return [
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        ]

    def _ts(self):
        return self.get_clock().now().nanoseconds // 1000


def main():
    rclpy.init()
    node = OmniCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
