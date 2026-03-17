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

# WAYPOINTS (posición, no velocidad)
TEST_SEQUENCE = [
    (5.0,   0.0,   0.0, -2.0, 0.0, 0.0, 0.0, "Hover"),
    (6.0,  20.0,   0.0, -2.0, 0.0, 0.0, 0.0, "Ir a X=20"),
    (6.0,  20.0,  20.0, -2.0, 0.0, 0.0, 0.0, "Ir a (20,20)"),
    (6.0,  0.0,  20.0, -2.0, 0.0, 0.0, 0.0, "Ir a (20,20)"),
    (6.0,  0.0,  0.0, -2.0, 0.0, 0.0, 0.0, "Ir a (20,20)"),
    (6.0,  20.0,   0.0, -2.0, np.radians(30), 0.0, 0.0, "Ir a X=20"),
    (6.0,  20.0,  20.0, -2.0, 0.0, np.radians(30), 0.0, "Ir a (20,20)"),
    (6.0,  0.0,  20.0, -2.0, 0.0, 0.0, np.radians(50), "Ir a (20,20)"),
    (6.0,  0.0,  0.0, -2.0, np.radians(20), np.radians(20), 0.0, "Ir a (20,20)"),
    (2.0,  0.0,  0.0, -2.0, 0.0, 0.0, 0.0, "Ir a (20,20)"),
]


class OmniCommander(Node):

    OFFBOARD_HZ = 20.0
    V_MAX = 5.7  # velocidad límite m/s

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

        self._local_pos = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()

        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw_accumulated = 0.0
        self._yaw_rate = 0.0

        self._dt = 1.0 / self.OFFBOARD_HZ

        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._pub_att = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', PX4_QOS)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

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

    def _local_pos_callback(self, msg):
        self._local_pos = msg

    def _status_callback(self, msg):
        self._vehicle_status = msg

    def _timer_callback(self):
        self._publish_offboard_mode()

        if self._state == self.STATE_PREFLIGHT:
            self._state_preflight()
        elif self._state == self.STATE_ARM_OFFBOARD:
            self._state_arm_offboard()
        elif self._state == self.STATE_TAKEOFF:
            self._state_takeoff()
        elif self._state == self.STATE_TEST:
            self._state_test()
        elif self._state == self.STATE_LAND:
            self._state_land()

        self._yaw_accumulated += self._yaw_rate * self._dt
        self._publish_trajectory_setpoint()
        self._publish_attitude_setpoint()

    def _state_preflight(self):
        self._preflight_count += 1
        if self._preflight_count > 40:
            self.get_logger().info('Armando...')
            self._transition_to(self.STATE_ARM_OFFBOARD)

    def _state_arm_offboard(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(0.1)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self._transition_to(self.STATE_TAKEOFF)

    def _state_takeoff(self):
        if self._z_origin is None:
            if np.isfinite(self._local_pos.z):
                self._z_origin = self._local_pos.z
                self._x = self._local_pos.x
                self._y = self._local_pos.y
                self._z = self._local_pos.z
                self.get_logger().info(f'z_origin: {self._z_origin}')

        target_z = self._z_origin - TAKEOFF_ALT

        self._z += np.clip(target_z - self._z, -self.V_MAX*self._dt, self.V_MAX*self._dt)

        if abs(self._local_pos.z - target_z) < 0.2:
            self._transition_to(self.STATE_TEST)

    def _state_test(self):
        if self._test_step >= len(TEST_SEQUENCE):
            self._transition_to(self.STATE_LAND)
            return

        duration, x_obj, y_obj, z_obj, roll, pitch, yaw_rate, desc = TEST_SEQUENCE[self._test_step]

        if self._step_start_time is None:
            self._step_start_time = self.get_clock().now()
            self.get_logger().info(desc)

        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

        max_step = self.V_MAX * self._dt

        self._x += np.clip(x_obj - self._x, -max_step, max_step)
        self._y += np.clip(y_obj - self._y, -max_step, max_step)
        self._z += np.clip(z_obj - self._z, -max_step, max_step)

        self._roll = roll
        self._pitch = pitch
        self._yaw_rate = yaw_rate

        if elapsed > duration:
            self._test_step += 1
            self._step_start_time = None

    def _state_land(self):
        if not hasattr(self, "_land_sent"):
            self.get_logger().info('LAND...')
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._land_sent = True

        low_speed = abs(self._local_pos.vz) < 0.1
        near_ground = abs(self._local_pos.z - self._z_origin) < 0.2

        if low_speed and near_ground:
            self.get_logger().info('Desarmando')
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self._transition_to(self.STATE_DONE)

    def _transition_to(self, new_state):
        self._state = new_state
        self._state_start_time = self.get_clock().now()

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._px4_timestamp()
        msg.position = True
        msg.attitude = True
        self._pub_offboard.publish(msg)

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self._px4_timestamp()
        msg.position = [self._x, self._y, self._z]
        msg.yaw = self._yaw_accumulated
        self._pub_traj.publish(msg)

    def _publish_attitude_setpoint(self):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self._px4_timestamp()

        q = self._euler_to_quat(self._roll, self._pitch, self._yaw_accumulated)

        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.yaw_sp_move_rate = float(self._yaw_rate)
        msg.thrust_body = [0.0, 0.0, -1.0]

        self._pub_att.publish(msg)


    def _euler_to_quat(self, roll, pitch, yaw):
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])


    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._px4_timestamp()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)

    def _px4_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OmniCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

