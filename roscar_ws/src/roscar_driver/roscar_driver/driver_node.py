"""ROScar1 driver node: bridges ROS2 topics to the YB-ERF01-V3.0 motor board.

Subscribes to /cmd_vel and forwards velocity commands to the board via
Rosmaster_Lib. Publishes odometry, IMU, and battery data from the board's
sensor readings.

The YB-ERF01-V3.0 (STM32) handles mecanum inverse kinematics and PID motor
control internally. This node only needs to:
  1. Forward cmd_vel -> set_car_motion(vx, vy, wz)
  2. Read sensor data -> publish ROS2 topics
  3. Integrate velocities -> publish odometry + TF
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from roscar_driver.mecanum_kinematics import MecanumOdometry


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (rad) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RoscarDriverNode(Node):
    """Main driver node for the ROScar1 mecanum robot."""

    def __init__(self):
        super().__init__('roscar_driver')

        # -- Declare parameters --
        self.declare_parameter('serial_port', '/dev/roscar_board')
        self.declare_parameter('car_type', 1)  # 1 = X3 mecanum
        self.declare_parameter('serial_delay', 0.002)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('max_linear_x', 1.0)   # m/s
        self.declare_parameter('max_linear_y', 1.0)   # m/s
        self.declare_parameter('max_angular_z', 5.0)   # rad/s
        self.declare_parameter('pub_rate', 20.0)       # Hz

        # -- Read parameters --
        serial_port = self.get_parameter('serial_port').value
        car_type = self.get_parameter('car_type').value
        serial_delay = self.get_parameter('serial_delay').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self._max_vx = self.get_parameter('max_linear_x').value
        self._max_vy = self.get_parameter('max_linear_y').value
        self._max_wz = self.get_parameter('max_angular_z').value
        pub_rate = self.get_parameter('pub_rate').value

        # -- Initialize hardware --
        self._bot = None
        try:
            from Rosmaster_Lib import Rosmaster
            self._bot = Rosmaster(
                car_type=car_type,
                com=serial_port,
                delay=serial_delay,
                debug=False,
            )
            self._bot.create_receive_threading()
            self._bot.set_auto_report_state(enable=True, forever=False)
            self.get_logger().info(
                f'Connected to YB-ERF01-V3.0 on {serial_port} '
                f'(car_type={car_type})'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to connect to motor board: {e}. '
                f'Running in dry-run mode (no hardware).'
            )

        # -- Odometry integrator --
        self._odom = MecanumOdometry()

        # -- TF broadcaster --
        self._tf_broadcaster = TransformBroadcaster(self)

        # -- Publishers --
        self._pub_odom = self.create_publisher(Odometry, 'odom_raw', 10)
        self._pub_vel = self.create_publisher(Twist, 'vel_raw', 10)
        self._pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self._pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self._pub_battery = self.create_publisher(Float32, 'battery_voltage', 10)

        # -- Subscribers --
        self._sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )

        # -- Timer for publishing sensor data --
        timer_period = 1.0 / pub_rate
        self._pub_timer = self.create_timer(timer_period, self._publish_sensor_data)

        # -- Watchdog: stop motors if no cmd_vel for 500ms --
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info('ROScar1 driver node started')

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _cmd_vel_callback(self, msg: Twist):
        """Forward velocity commands to the motor board.

        The board is mounted 180-deg rotated (camera/lidar on its "rear") and
        motor L/R ports are swapped.  Net effect: negate all three components
        (vx, vy, wz) at the hardware boundary.
        """
        self._last_cmd_time = self.get_clock().now()

        vx = self._clamp(msg.linear.x, self._max_vx)
        vy = self._clamp(msg.linear.y, self._max_vy)
        wz = self._clamp(msg.angular.z, self._max_wz)

        if self._bot is not None:
            self._bot.set_car_motion(-vx, -vy, -wz)

    def _watchdog_callback(self):
        """Stop motors if no cmd_vel received recently."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5 and self._bot is not None:
            self._bot.set_car_motion(0.0, 0.0, 0.0)

    def _publish_sensor_data(self):
        """Read sensors from the board and publish ROS2 topics."""
        if self._bot is None:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        time_sec = now.nanoseconds / 1e9

        # --- Velocity / Odometry ---
        # Board axes are rotated from robot frame and motor ports are
        # swapped left/right: negate all velocity components
        try:
            vx, vy, vz = self._bot.get_motion_data()
            vx, vy, vz = -vx, -vy, -vz
        except Exception:
            vx, vy, vz = 0.0, 0.0, 0.0

        # Publish raw velocity
        vel_msg = Twist()
        vel_msg.linear.x = float(vx)
        vel_msg.linear.y = float(vy)
        vel_msg.angular.z = float(vz)
        self._pub_vel.publish(vel_msg)

        # Integrate odometry
        x, y, theta = self._odom.update(vx, vy, vz, time_sec)
        q = yaw_to_quaternion(theta)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = float(vx)
        odom_msg.twist.twist.linear.y = float(vy)
        odom_msg.twist.twist.angular.z = float(vz)
        # Covariance: diagonal values for [x, y, z, roll, pitch, yaw]
        # These are initial estimates; tune based on real performance
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[14] = 1e6   # z (unused, large uncertainty)
        odom_msg.pose.covariance[21] = 1e6   # roll
        odom_msg.pose.covariance[28] = 1e6   # pitch
        odom_msg.pose.covariance[35] = 0.03  # yaw
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[7] = 0.01  # vy
        odom_msg.twist.covariance[14] = 1e6  # vz
        odom_msg.twist.covariance[21] = 1e6  # wx
        odom_msg.twist.covariance[28] = 1e6  # wy
        odom_msg.twist.covariance[35] = 0.03 # wz
        self._pub_odom.publish(odom_msg)

        # Broadcast odom -> base_footprint TF
        if self._publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

        # --- IMU ---
        # Board axes are 180-deg rotated in yaw: negate x, y for both.
        # Accel z: Rosmaster_Lib reports gravity as negative; ROS expects
        # +9.81 when flat, so negate az too.
        # Gyro z: 180-deg yaw rotation does NOT change gz direction, so
        # keep gz as-is (it must agree with wheel encoder angular velocity).
        try:
            ax, ay, az = self._bot.get_accelerometer_data()
            gx, gy, gz = self._bot.get_gyroscope_data()
            ax, ay, az = -ax, -ay, -az
            gx, gy = -gx, -gy
        except Exception:
            ax, ay, az = 0.0, 0.0, 0.0
            gx, gy, gz = 0.0, 0.0, 0.0

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._imu_frame
        # Orientation not provided by raw data (use IMU filter downstream)
        imu_msg.orientation_covariance[0] = -1.0  # indicates no orientation
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.linear_acceleration_covariance[0] = 0.04
        imu_msg.linear_acceleration_covariance[4] = 0.04
        imu_msg.linear_acceleration_covariance[8] = 0.04
        self._pub_imu.publish(imu_msg)

        # --- Magnetometer ---
        # Board axes are 180° rotated: negate x, y components; z unchanged
        try:
            mx, my, mz = self._bot.get_magnetometer_data()
            mx, my = -mx, -my
        except Exception:
            mx, my, mz = 0.0, 0.0, 0.0

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self._imu_frame
        mag_msg.magnetic_field.x = float(mx)
        mag_msg.magnetic_field.y = float(my)
        mag_msg.magnetic_field.z = float(mz)
        self._pub_mag.publish(mag_msg)

        # --- Battery ---
        try:
            voltage = self._bot.get_battery_voltage()
        except Exception:
            voltage = 0.0

        bat_msg = Float32()
        bat_msg.data = float(voltage)
        self._pub_battery.publish(bat_msg)

    def destroy_node(self):
        """Clean shutdown: stop motors before exiting."""
        self.get_logger().info('Shutting down, stopping motors...')
        if self._bot is not None:
            self._bot.set_car_motion(0.0, 0.0, 0.0)
            self._bot.reset_car_state()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoscarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
