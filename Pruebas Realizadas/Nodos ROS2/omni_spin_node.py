#!/usr/bin/env python3
"""
omni_spin_node.py
=================
Despega a TAKEOFF_ALT metros y rota el dron sobre sí mismo (yaw rate)
manteniendo posición fija en XYZ.

Secuencia:
  1. Heartbeat offboard 2s
  2. Offboard + Arm
  3. Despegue a TAKEOFF_ALT metros
  4. Rotación continua sobre sí mismo a YAW_RATE rad/s
  5. Land + Disarm

Uso:
  python3 omni_spin_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

import numpy as np

# ---------------------------------------------------------------------------
# Configuración — ajusta estos valores
# ---------------------------------------------------------------------------
TAKEOFF_ALT  = 2.0          # metros sobre el punto de despegue
YAW_RATE     = 0 # rad/s — 45°/s = una vuelta completa en 8s
ROLL_RATE     = np.radians(45.0)  # rad/s — 45°/s = una vuelta completa en 8s
SPIN_DURATION = 16.0        # segundos girando (2 vueltas completas a 45°/s)
# ---------------------------------------------------------------------------

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OmniSpin(Node):

    VEL_MAX_Z   = 1.0
    OFFBOARD_HZ = 20.0

    STATE_PREFLIGHT    = 0
    STATE_ARM_OFFBOARD = 1
    STATE_TAKEOFF      = 2
    STATE_SPIN         = 3
    STATE_LAND         = 4
    STATE_DONE         = 5

    def __init__(self):
        super().__init__('omni_spin')

        self._state            = self.STATE_PREFLIGHT
        self._state_start_time = self.get_clock().now()
        self._preflight_count  = 0
        self._z_origin         = None
        self._yaw_accumulated  = 0.0
        self._dt               = 1.0 / self.OFFBOARD_HZ

        # Comando actual
        self._vx    = 0.0
        self._vy    = 0.0
        self._vz    = 0.0
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw_rate = 0.0

        self._local_pos = VehicleLocalPosition()

        # Publishers
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._pub_att = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', PX4_QOS)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # Subscribers
        self._sub_local_pos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            lambda msg: setattr(self, '_local_pos', msg), PX4_QOS)

        self._timer = self.create_timer(self._dt, self._timer_callback)
        self.get_logger().info(
            f'OmniSpin iniciado — despegue a {TAKEOFF_ALT}m, '
            f'rotación {np.degrees(YAW_RATE):.0f}°/s durante {SPIN_DURATION}s'
        )

    # ------------------------------------------------------------------
    # Timer principal
    # ------------------------------------------------------------------

    def _timer_callback(self):
        self._publish_offboard_mode()

        if self._state == self.STATE_PREFLIGHT:
            self._preflight_count += 1
            self._set(0, 0, 0, 0, 0, 0)
            if self._preflight_count >= int(2.0 * self.OFFBOARD_HZ):
                self.get_logger().info('Activando Offboard + Arm...')
                self._goto(self.STATE_ARM_OFFBOARD)

        elif self._state == self.STATE_ARM_OFFBOARD:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('Armado — subiendo...')
            self._goto(self.STATE_TAKEOFF)

        elif self._state == self.STATE_TAKEOFF:
            if self._z_origin is None:
                if np.isfinite(self._local_pos.z):
                    self._z_origin = self._local_pos.z
                    self.get_logger().info(f'z_origin={self._z_origin:.3f}')
                else:
                    self._set(0, 0, 0, 0, 0, 0)
                    self._publish_setpoints()
                    return

            current_alt = self._z_origin - self._local_pos.z

            if current_alt < TAKEOFF_ALT - 0.2:
                self._set(0, 0, -0.8, 0, 0, 0)
                self.get_logger().info(
                    f'Subiendo... {current_alt:.2f}/{TAKEOFF_ALT} m',
                    throttle_duration_sec=1.0)
            else:
                self.get_logger().info(f'Altitud alcanzada: {current_alt:.2f} m — iniciando rotación')
                self._set(0, 0, 0, 0, 0, 0)
                self._goto(self.STATE_SPIN)

        elif self._state == self.STATE_SPIN:
            elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9

            if elapsed < SPIN_DURATION:
                # Posición fija, solo yaw rate
                self._set(vx=0, vy=0, vz=0, roll=ROLL_RATE, pitch=0, yaw_rate=YAW_RATE)
                self.get_logger().info(
                    f'Girando... {elapsed:.1f}/{SPIN_DURATION}s '
                    f'yaw={np.degrees(self._yaw_accumulated):.1f}°',
                    throttle_duration_sec=1.0)
            else:
                self.get_logger().info('Rotación completada — aterrizando')
                self._set(0, 0, 0, 0, 0, 0)
                self._goto(self.STATE_LAND)

        elif self._state == self.STATE_LAND:
            self.get_logger().info('Enviando LAND', throttle_duration_sec=2.0)
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._set(0, 0, 0, 0, 0, 0)
            elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9
            if elapsed > 10.0:
                self.get_logger().info('Aterrizado — desarmando')
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0, param2=21196.0)
                self._goto(self.STATE_DONE)

        elif self._state == self.STATE_DONE:
            self._set(0, 0, 0, 0, 0, 0)

        self._publish_setpoints()

    # ------------------------------------------------------------------
    # Publicadores
    # ------------------------------------------------------------------

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._ts()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = True
        msg.body_rate    = False
        self._pub_offboard.publish(msg)

    def _publish_setpoints(self):
        # Acumular yaw
        self._yaw_accumulated = self._wrap(
            self._yaw_accumulated + self._yaw_rate * self._dt)

        # Trajectory setpoint — velocidad en world frame
        traj = TrajectorySetpoint()
        traj.timestamp    = self._ts()
        traj.position     = [float('nan'), float('nan'), float('nan')]
        traj.velocity     = [float(self._vx), float(self._vy), float(self._vz)]
        traj.acceleration = [float('nan'), float('nan'), float('nan')]
        traj.yaw          = float('nan')
        traj.yawspeed     = float('nan')
        self._pub_traj.publish(traj)

        # Attitude setpoint — actitud desacoplada
        att = VehicleAttitudeSetpoint()
        att.timestamp = self._ts()
        q = self._euler_to_quat(self._roll, self._pitch, self._yaw_accumulated)
        att.q_d              = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        att.yaw_sp_move_rate = float(self._yaw_rate)
        att.thrust_body      = [0.0, 0.0, -1.0]
        self._pub_att.publish(att)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0, **kwargs):
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set(self, vx, vy, vz, roll, pitch, yaw_rate):
        self._vx, self._vy, self._vz = float(vx), float(vy), float(vz)
        self._roll, self._pitch       = float(roll), float(pitch)
        self._yaw_rate                = float(yaw_rate)

    def _goto(self, state):
        self._state = state
        self._state_start_time = self.get_clock().now()

    def _euler_to_quat(self, roll, pitch, yaw):
        cr, sr = np.cos(roll  * 0.5), np.sin(roll  * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw   * 0.5), np.sin(yaw   * 0.5)
        return np.array([
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
        ])

    def _wrap(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def _ts(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OmniSpin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
