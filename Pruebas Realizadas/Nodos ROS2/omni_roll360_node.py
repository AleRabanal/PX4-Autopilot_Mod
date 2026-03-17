#!/usr/bin/env python3
"""
omni_roll360_node.py
====================
Despega a TAKEOFF_ALT metros y ejecuta vueltas completas en roll (360°)
manteniendo posición XYZ fija y yaw fijo.

REQUISITOS EN PX4 antes de usar:
  param set MPC_TILTMAX_AIR 180
  param set MPC_TILTMAX_LND 180
  Y en MulticopterPositionControl.cpp quitar el límite de tilt en modo omni.

Uso:
  python3 omni_roll360_node.py
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
)

import numpy as np

# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------
TAKEOFF_ALT   = 3.0               # metros — más alto para tener margen al invertirse
ROLL_RATE     = np.radians(20.0)  # rad/s — 30°/s = vuelta completa en 12s (suave)
NUM_ROTATIONS = 2                 # número de vueltas completas
SPIN_DURATION = (2 * np.pi / ROLL_RATE) * NUM_ROTATIONS  # calculado automáticamente
# ---------------------------------------------------------------------------

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OmniRoll360(Node):

    OFFBOARD_HZ = 20.0

    STATE_PREFLIGHT    = 0
    STATE_ARM_OFFBOARD = 1
    STATE_TAKEOFF      = 2
    STATE_SPIN         = 3
    STATE_RECOVER      = 4   # volver a actitud nivelada antes de land
    STATE_LAND         = 5
    STATE_DONE         = 6

    def __init__(self):
        super().__init__('omni_roll360')

        self._state            = self.STATE_PREFLIGHT
        self._state_start_time = self.get_clock().now()
        self._preflight_count  = 0
        self._z_origin         = None
        self._dt               = 1.0 / self.OFFBOARD_HZ

        # Actitud acumulada (roll se acumula continuamente)
        self._roll_accumulated = 0.0
        self._yaw_fixed        = 0.0   # yaw se fija al entrar en spin

        # Velocidades de traslación (siempre 0 durante spin)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0

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
        self._sub_pos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            lambda msg: setattr(self, '_local_pos', msg), PX4_QOS)

        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'OmniRoll360 — despegue {TAKEOFF_ALT}m, '
            f'roll {np.degrees(ROLL_RATE):.0f}°/s, '
            f'{NUM_ROTATIONS} vueltas ({SPIN_DURATION:.1f}s)'
        )

    # ------------------------------------------------------------------
    # Timer principal
    # ------------------------------------------------------------------

    def _tick(self):
        self._publish_offboard_mode()

        # --- PREFLIGHT ---
        if self._state == self.STATE_PREFLIGHT:
            self._preflight_count += 1
            self._vx = self._vy = self._vz = 0.0
            if self._preflight_count >= int(2.0 * self.OFFBOARD_HZ):
                self.get_logger().info('Activando Offboard + Arm...')
                self._goto(self.STATE_ARM_OFFBOARD)

        # --- ARM + OFFBOARD ---
        elif self._state == self.STATE_ARM_OFFBOARD:
            self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
            self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            self.get_logger().info('Armado — subiendo...')
            self._goto(self.STATE_TAKEOFF)

        # --- TAKEOFF ---
        elif self._state == self.STATE_TAKEOFF:
            if self._z_origin is None:
                if np.isfinite(self._local_pos.z):
                    self._z_origin = self._local_pos.z
                    self.get_logger().info(f'z_origin={self._z_origin:.3f}')
                else:
                    self._pub_setpoints(vz=0)
                    return

            alt = self._z_origin - self._local_pos.z

            if alt < TAKEOFF_ALT - 0.2:
                self._vz = -0.8
                self.get_logger().info(
                    f'Subiendo {alt:.2f}/{TAKEOFF_ALT} m',
                    throttle_duration_sec=1.0)
            else:
                self._vz = 0.0
                # Fijar yaw actual al entrar en spin
                self._yaw_fixed = self._local_pos.heading
                self._roll_accumulated = 0.0
                self.get_logger().info(
                    f'Altitud {alt:.2f}m — iniciando roll 360° '
                    f'({NUM_ROTATIONS} vueltas a {np.degrees(ROLL_RATE):.0f}°/s)'
                )
                self.get_logger().info(f'yaw_fixed al iniciar spin: {np.degrees(self._yaw_fixed):.1f}°')
                self._goto(self.STATE_SPIN)

        # --- SPIN (roll continuo) ---
        elif self._state == self.STATE_SPIN:
            elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9

            if elapsed < SPIN_DURATION:
                # Acumular roll sin límite (permite pasar por 180° e invertirse)
                self._roll_accumulated += ROLL_RATE * self._dt
                self._vx = self._vy = self._vz = 0.0

                vueltas = self._roll_accumulated / (2 * np.pi)
                self.get_logger().info(
                    f'Roll {np.degrees(self._roll_accumulated) % 360:.1f}°  '
                    f'({vueltas:.2f} vueltas)  t={elapsed:.1f}/{SPIN_DURATION:.1f}s',
                    throttle_duration_sec=0.5)
            else:
                self.get_logger().info('Vueltas completadas — nivelando actitud...')
                self._goto(self.STATE_RECOVER)

        # --- RECOVER (volver a nivelado suavemente) ---
        elif self._state == self.STATE_RECOVER:
            # Reducir roll acumulado hacia 0 a la misma velocidad
            if abs(self._roll_accumulated) > np.radians(2.0):
                # Normalizar a [-pi, pi] y reducir hacia 0
                roll_norm = (self._roll_accumulated + np.pi) % (2 * np.pi) - np.pi
                step = ROLL_RATE * self._dt
                if abs(roll_norm) > step:
                    self._roll_accumulated -= np.sign(roll_norm) * step
                else:
                    self._roll_accumulated = 0.0
                self.get_logger().info(
                    f'Nivelando roll: {np.degrees(self._roll_accumulated):.1f}°',
                    throttle_duration_sec=0.5)
            else:
                self._roll_accumulated = 0.0
                self.get_logger().info('Nivelado — aterrizando')
                self._goto(self.STATE_LAND)

        # --- LAND ---
        elif self._state == self.STATE_LAND:
            self.get_logger().info('Enviando LAND', throttle_duration_sec=2.0)
            self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._vx = self._vy = self._vz = 0.0
            elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9
            if elapsed > 10.0:
                self.get_logger().info('Aterrizado — desarmando')
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                          p1=0.0, p2=21196.0)
                self._goto(self.STATE_DONE)

        # --- DONE ---
        elif self._state == self.STATE_DONE:
            self._vx = self._vy = self._vz = 0.0
            self._roll_accumulated = 0.0

        self._pub_setpoints()

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

    def _pub_setpoints(self, vz=None):
        vz_send = vz if vz is not None else self._vz

        # Trajectory — velocidad en world frame
        traj = TrajectorySetpoint()
        traj.timestamp    = self._ts()
        traj.position     = [float('nan'), float('nan'), float('nan')]
        traj.velocity     = [float(self._vx), float(self._vy), float(vz_send)]
        traj.acceleration = [float('nan'), float('nan'), float('nan')]
        traj.yaw          = float('nan')
        traj.yawspeed     = float('nan')
        self._pub_traj.publish(traj)

        # Attitude — roll acumulado, pitch=0, yaw fijo
        att = VehicleAttitudeSetpoint()
        att.timestamp = self._ts()
        q = self._euler_to_quat(self._roll_accumulated, 0.0, self._yaw_fixed)
        att.q_d              = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        att.yaw_sp_move_rate = 0.0
        att.thrust_body      = [0.0, 0.0, -1.0]
        self._pub_att.publish(att)

    def _cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    def _euler_to_quat(self, roll, pitch, yaw):
        """Construye q = q_yaw * q_roll  (sin acoplamiento pitch espurio)"""
        # Quaternion de yaw puro
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        q_yaw = np.array([cy, 0.0, 0.0, sy])

        # Quaternion de roll puro (sobre eje X del body)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        q_roll = np.array([cr, sr, 0.0, 0.0])

        # Multiplicar: q = q_yaw * q_roll
        w1, x1, y1, z1 = q_yaw
        w2, x2, y2, z2 = q_roll
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    def _goto(self, state):
        self._state = state
        self._state_start_time = self.get_clock().now()

    def _ts(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OmniRoll360()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
