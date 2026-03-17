#!/usr/bin/env python3
"""
omni_commander_node.py
======================
Nodo ROS 2 para comandar un dron 6DOF (omni) con PX4 via uXRCE-DDS.

Secuencia automática:
  1. Publica offboard heartbeat durante 2s (PX4 lo requiere antes de aceptar el modo)
  2. Activa modo Offboard
  3. Arma el dron
  4. Despega a TAKEOFF_ALT metros (vz negativo en NED = subir)
  5. Espera a que alcance la altitud
  6. Ejecuta la secuencia de prueba de desacoplamiento
  7. Hover y aterriza

Uso:
  python3 omni_commander_node.py
"""

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

# QoS que PX4 espera para los topics /fmu/in/
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Altitud de despegue en metros (positivo = metros sobre el suelo)
TAKEOFF_ALT = 2.0

# Secuencia de prueba: (duracion_s, vx, vy, vz, roll_rad, pitch_rad, yaw_rate_rad_s, descripcion)
TEST_SEQUENCE = [
    (4.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  "Hover estático"),
    (4.0,  1.0,  0.0,  0.0,  0.0,   0.0,   0.0,  "Adelante vx=1 m/s, actitud nivelada"),
    (4.0,  1.0,  0.0,  0.0,  0.0,   0.0,   0.5,  "Adelante + yaw_rate: dirección de vuelo NO cambia"),
    (4.0,  1.0,  0.0,  0.0,  0.3,   0.0,   0.0,  "Adelante + roll 17°: chasis inclinado, sigue al norte"),
    (4.0,  0.0,  1.0,  0.0,  0.0,   0.0,   0.0,  "Lateral vy=1 m/s"),
    (4.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  "Hover — fin de secuencia"),
]


class OmniCommander(Node):

    # Límites
    VEL_MAX_XY   = 3.0
    VEL_MAX_Z    = 2.0
    ROLL_MAX     = np.radians(45.0)
    PITCH_MAX    = np.radians(45.0)
    YAW_RATE_MAX = np.radians(90.0)
    OFFBOARD_HZ  = 20.0

    # Estados internos del nodo
    STATE_PREFLIGHT   = 0   # enviando heartbeat, esperando
    STATE_ARM_OFFBOARD= 1   # activar offboard y armar
    STATE_TAKEOFF     = 2   # subiendo a TAKEOFF_ALT
    STATE_TEST        = 3   # ejecutando secuencia de prueba
    STATE_LAND        = 4   # aterrizando
    STATE_DONE        = 5

    def __init__(self):
        super().__init__('omni_commander')

        # --- Estado de la máquina de estados ---
        self._state            = self.STATE_PREFLIGHT
        self._state_start_time = self.get_clock().now()
        self._preflight_count  = 0          # ticks de heartbeat antes de armar
        self._test_step        = 0          # paso actual de TEST_SEQUENCE
        self._step_start_time  = None
        self._z_origin         = None       # z NED en el momento del despegue

        # --- Estado del vehículo (leído de PX4) ---
        self._local_pos        = VehicleLocalPosition()
        self._vehicle_status   = VehicleStatus()

        # --- Comando actual (traslación + actitud) ---
        self._vx   = 0.0
        self._vy   = 0.0
        self._vz   = 0.0
        self._roll = 0.0
        self._pitch= 0.0
        self._yaw_accumulated = 0.0
        self._yaw_rate = 0.0

        self._dt = 1.0 / self.OFFBOARD_HZ

        # --- Publishers ---
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._pub_att = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', PX4_QOS)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # --- Subscribers de estado del vehículo ---
        self._sub_local_pos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_callback, PX4_QOS)
        self._sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self._status_callback, PX4_QOS)

        # --- Subscriber de comandos externos (opcional, para control manual posterior) ---
        self._sub_cmd = self.create_subscription(
            Twist, '/omni/cmd_vel', self._cmd_callback, 10)

        # --- Timer principal ---
        self._timer = self.create_timer(self._dt, self._timer_callback)

        self.get_logger().info('OmniCommander iniciado — comenzando secuencia automática')

    # ------------------------------------------------------------------
    # Callbacks de estado
    # ------------------------------------------------------------------

    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _status_callback(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _cmd_callback(self, msg: Twist):
        """Permite control externo manual una vez en STATE_TEST."""
        if self._state == self.STATE_TEST:
            self._vx    = msg.linear.x
            self._vy    = msg.linear.y
            self._vz    = msg.linear.z
            self._roll  = np.clip(msg.angular.x, -self.ROLL_MAX,  self.ROLL_MAX)
            self._pitch = np.clip(msg.angular.y, -self.PITCH_MAX, self.PITCH_MAX)
            self._yaw_rate = np.clip(msg.angular.z, -self.YAW_RATE_MAX, self.YAW_RATE_MAX)

    # ------------------------------------------------------------------
    # Timer principal — máquina de estados
    # ------------------------------------------------------------------

    def _timer_callback(self):
        # Siempre publicar heartbeat offboard (PX4 lo necesita continuamente)
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

        elif self._state == self.STATE_DONE:
            # Hover suave hasta que el operador apague el nodo
            self._set_command(0, 0, 0, 0, 0, 0)

        # Publicar setpoints actuales
        self._yaw_accumulated = self._wrap_angle(
            self._yaw_accumulated + self._yaw_rate * self._dt)
        self._publish_trajectory_setpoint()
        self._publish_attitude_setpoint()

    # ------------------------------------------------------------------
    # Estados
    # ------------------------------------------------------------------

    def _state_preflight(self):
        """
        Enviar heartbeat durante ~2s antes de intentar armar.
        PX4 requiere recibir offboard_control_mode antes de aceptar el modo offboard.
        """
        self._set_command(0, 0, 0, 0, 0, 0)
        self._preflight_count += 1

        if self._preflight_count >= int(2.0 * self.OFFBOARD_HZ):
            self.get_logger().info('Heartbeat establecido — activando Offboard y armando...')
            self._transition_to(self.STATE_ARM_OFFBOARD)

    def _state_arm_offboard(self):
        """Enviar comando de modo offboard y armar."""
        # Activar modo offboard (MAV_CMD_DO_SET_MODE, modo 6 = offboard en PX4)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        time.sleep(0.1)
        # Armar (MAV_CMD_COMPONENT_ARM_DISARM, param1=1 = arm)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        self.get_logger().info('Comandos Offboard + Arm enviados — esperando despegue...')
        self._transition_to(self.STATE_TAKEOFF)

    def _state_takeoff(self):
        """
        Subir hasta TAKEOFF_ALT metros sobre el punto de despegue.
        Guarda z_origin la primera vez para medir altitud relativa,
        evitando el problema de que z=0 en NED no coincide con el suelo real.
        """
        # Guardar z de origen la primera vez (cuando ya hay datos válidos)
        if self._z_origin is None:
            if np.isfinite(self._local_pos.z):
                self._z_origin = self._local_pos.z
                self.get_logger().info(f'z_origin fijado en: {self._z_origin:.2f} m (NED)')
            else:
                self._set_command(0, 0, 0, 0, 0, 0)
                return

        # Altitud relativa al suelo de despegue (positivo = hemos subido)
        current_alt = self._z_origin - self._local_pos.z

        if current_alt < TAKEOFF_ALT - 0.2:
            # Subir a velocidad constante
            self._set_command(vx=0, vy=0, vz=-0.8, roll=0, pitch=0, yaw_rate=0)
            self.get_logger().info(
                f'Subiendo... altitud relativa: {current_alt:.2f} m / objetivo: {TAKEOFF_ALT} m',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(f'Altitud alcanzada: {current_alt:.2f} m — iniciando prueba')
            self._set_command(0, 0, 0, 0, 0, 0)
            self._test_step = 0
            self._step_start_time = self.get_clock().now()
            self._transition_to(self.STATE_TEST)

    def _state_test(self):
        """Ejecutar la secuencia TEST_SEQUENCE paso a paso."""
        if self._test_step >= len(TEST_SEQUENCE):
            self.get_logger().info('Secuencia completada — aterrizando')
            self._transition_to(self.STATE_LAND)
            return

        duration, vx, vy, vz, roll, pitch, yaw_rate, desc = TEST_SEQUENCE[self._test_step]

        # Primer tick del paso: loguear descripción
        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if elapsed < 0.05:
            self.get_logger().info(f'[Paso {self._test_step + 1}/{len(TEST_SEQUENCE)}] {desc}')

        self._set_command(vx, vy, vz, roll, pitch, yaw_rate)

        if elapsed >= duration:
            self._test_step += 1
            self._step_start_time = self.get_clock().now()

    def _state_land(self):
        """Usar el comando de land nativo de PX4."""
        self.get_logger().info('Enviando comando LAND', throttle_duration_sec=2.0)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._set_command(0, 0, 0, 0, 0, 0)
        elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9
        if elapsed > 10.0:
            self.get_logger().info('Aterrizaje completado — desarmando')
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,
                param2=21196.0)   # magic number para forzar desarme en PX4
            self._transition_to(self.STATE_DONE)

    # ------------------------------------------------------------------
    # Helpers de estado
    # ------------------------------------------------------------------

    def _set_command(self, vx, vy, vz, roll, pitch, yaw_rate):
        self._vx       = float(vx)
        self._vy       = float(vy)
        self._vz       = float(vz)
        self._roll     = float(roll)
        self._pitch    = float(pitch)
        self._yaw_rate = float(yaw_rate)

    def _transition_to(self, new_state):
        self._state = new_state
        self._state_start_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Publicadores
    # ------------------------------------------------------------------

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._px4_timestamp()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = True
        msg.body_rate    = False
        self._pub_offboard.publish(msg)

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp    = self._px4_timestamp()
        msg.position     = [float('nan'), float('nan'), float('nan')]
        msg.velocity     = [
            float(np.clip(self._vx, -self.VEL_MAX_XY, self.VEL_MAX_XY)),
            float(np.clip(self._vy, -self.VEL_MAX_XY, self.VEL_MAX_XY)),
            float(np.clip(self._vz, -self.VEL_MAX_Z,  self.VEL_MAX_Z)),
        ]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw          = float('nan')
        msg.yawspeed     = float('nan')
        self._pub_traj.publish(msg)

    def _publish_attitude_setpoint(self):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self._px4_timestamp()

        roll  = np.clip(self._roll,  -self.ROLL_MAX,  self.ROLL_MAX)
        pitch = np.clip(self._pitch, -self.PITCH_MAX, self.PITCH_MAX)
        yaw   = self._yaw_accumulated

        q = self._euler_to_quat(roll, pitch, yaw)
        msg.q_d              = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.yaw_sp_move_rate = float(np.clip(self._yaw_rate, -self.YAW_RATE_MAX, self.YAW_RATE_MAX))
        msg.thrust_body      = [0.0, 0.0, -1.0]
        self._pub_att.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                  param3=0.0, param4=0.0,
                                  param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._px4_timestamp()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.param3           = float(param3)
        msg.param4           = float(param4)
        msg.param5           = float(param5)
        msg.param6           = float(param6)
        msg.param7           = float(param7)
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
        """Euler ZYX → quaternion [w, x, y, z] (convención PX4)."""
        cr, sr = np.cos(roll  * 0.5), np.sin(roll  * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw   * 0.5), np.sin(yaw   * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return np.array([w, x, y, z])

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _px4_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OmniCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
