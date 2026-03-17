import rclpy
from rclpy.node import Node
import math

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleOdometry

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy


class OmniController(Node):

    def __init__(self):

        super().__init__('omni_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)

        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint,
            '/fmu/in/vehicle_thrust_setpoint',
            10)

        self.torque_pub = self.create_publisher(
            VehicleTorqueSetpoint,
            '/fmu/in/vehicle_torque_setpoint',
            10)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        self.counter = 0
        self.current_yaw = 0.0
        self.yaw_ref = None
        self.current_z = 0.0

        self.prev_yaw = 0.0
        self.prev_time = None
        self.vz = 0.0

        self.timer = self.create_timer(0.02, self.loop)

    # ---------------------------------------

    def odom_callback(self, msg):

        q = msg.q

        siny = 2.0 * (q[0]*q[3] + q[1]*q[2])
        cosy = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3])

        self.current_yaw = math.atan2(siny, cosy)

        if self.yaw_ref is None:
            self.yaw_ref = self.current_yaw

        self.current_z = msg.position[2]
        self.vz = msg.velocity[2]

    # ---------------------------------------

    def send_command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.cmd_pub.publish(msg)

    # ---------------------------------------

    def loop(self):

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard = OffboardControlMode()

        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.thrust_and_torque = True

        offboard.timestamp = timestamp

        self.offboard_pub.publish(offboard)

        if self.counter == 20:
            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        if self.counter == 40:
            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        thrust = VehicleThrustSetpoint()
        torque = VehicleTorqueSetpoint()

        thrust.timestamp = timestamp
        torque.timestamp = timestamp

        # ---------------- YAW CONTROL ----------------

        yaw_error = 0.0

        if self.yaw_ref is not None:

            yaw_error = self.current_yaw - self.yaw_ref

            yaw_error = math.atan2(
                math.sin(yaw_error),
                math.cos(yaw_error)
            )

        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is None:
            dt = 0.02
        else:
            dt = current_time - self.prev_time

        yaw_rate = (self.current_yaw - self.prev_yaw) / dt

        Kp_yaw = 0.25
        Kd_yaw = 0.08

        yaw_torque = -Kp_yaw * yaw_error - Kd_yaw * yaw_rate

        yaw_torque = max(-0.08, min(0.08, yaw_torque))

        self.prev_yaw = self.current_yaw
        self.prev_time = current_time

        # ---------------- ALTITUDE CONTROL ----------------

        hover = -0.215
        target_z = -4.0

        error_z = target_z - self.current_z

        Kp_z = 0.12
        Kd_z = 0.08

        thrust_z = hover + Kp_z * error_z - Kd_z * self.vz

        thrust_z = max(-0.23, min(-0.19, thrust_z))

        # ---------------- DEMO ----------------

        if self.counter < 300:

            thrust.xyz = [0.0, 0.0, -0.22]

        elif self.counter < 500:

            thrust.xyz = [0.0, 0.0, thrust_z]

        elif self.counter < 1500:

            forward_world = 0.005

            fx = forward_world * math.cos(self.current_yaw)
            fy = forward_world * math.sin(self.current_yaw)

            lateral_sq = fx*fx + fy*fy
            vertical = -math.sqrt(max(0.0, thrust_z*thrust_z - lateral_sq))

            thrust.xyz = [fx, fy, vertical]

        else:

            thrust.xyz = [0.0, 0.0, thrust_z]

        torque.xyz = [0.0, 0.0, yaw_torque]

        self.thrust_pub.publish(thrust)
        self.torque_pub.publish(torque)

        self.counter += 1


def main():

    rclpy.init()

    node = OmniController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
