import sys
import os
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_dir, 'mavsim/mavsim_python'))

from pyproj import Proj, transform
import numpy as np

import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import Airspeed
from rosflight_msgs.msg import Barometer
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import GNSS
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

import parameters.planner_parameters as PLAN
from controllers.autopilot import Autopilot
from estimators.observer_full import Observer
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_sensors import MsgSensors


def ecef_to_ned_matrix(ecef):
    # Define the projection for ECEF: EPSG:4978
    # and WGS84 LatLong: EPSG:4326
    ecef_proj = Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    latlong_proj = Proj(proj='latlong', ellps='WGS84', datum='WGS84')

    # Convert from ECEF to Geodetic (Latitude, Longitude, Altitude)
    lon, lat, _ = transform(ecef_proj, latlong_proj, ecef[0], ecef[1], ecef[2], radians=True)

    # Calculate transformation matrix from ECEF to NED
    matrix = np.array([
        [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
        [-np.sin(lon), np.cos(lon), 0],
        [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]
    ])
    return matrix


class MavSimBridge(Node):
    def __init__(self):
        super().__init__('mavsim_bridge')
        control_timer_period = 1 / 100.0
        self.sensors = MsgSensors()
        self.initial_baro = None
        self.initial_ecef = None
        self.ecef_ned_matrix = None

        # initialize elements of the architecture
        self.autopilot = Autopilot(control_timer_period)
        self.observer = Observer(control_timer_period)
        self.path_follower = PathFollower()
        self.path_manager = PathManager()

        # waypoint definition
        self.waypoints = MsgWaypoints()
        self.waypoints.type = 'fillet'
        Va = PLAN.Va0
        self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
        self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
        self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
        self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)

        # Create publisher for delta commands
        self.delta_pub = self.create_publisher(Command, 'command', 1)

        # Create sensor subscriptions
        self.airspeed_sub = self.create_subscription(Airspeed, '/airspeed', self.airspeed_callback, 1)
        self.barometer_sub = self.create_subscription(Barometer, '/barometer', self.barometer_callback, 1)
        self.gnss_sub = self.create_subscription(GNSS, '/gnss', self.gnss_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.mag_sub = self.create_subscription(MagneticField, '/magnetometer', self.mag_callback, 1)

        # Create timer for control loop
        self.timer = self.create_timer(control_timer_period, self.timer_callback)

    def airspeed_callback(self, msg):
        # msg is in Pa
        # Uses max since mavsim is not prepared to handle negative values
        self.sensors.diff_pressure = max(msg.differential_pressure, 0.0)

    def barometer_callback(self, msg):
        # msg is in Pa
        if self.initial_baro is None:
            self.initial_baro = msg.pressure
        self.sensors.abs_pressure = msg.pressure - self.initial_baro

    def gnss_callback(self, msg):
        # msg is in m and m/s
        if self.initial_ecef is None:
            self.initial_ecef = msg.position
            self.ecef_ned_matrix = ecef_to_ned_matrix(self.initial_ecef)
        ned = np.dot(self.ecef_ned_matrix, msg.position - self.initial_ecef)
        ned_vel = np.dot(self.ecef_ned_matrix, msg.velocity)
        self.sensors.gps_n = ned[0]
        self.sensors.gps_e = ned[1]
        self.sensors.gps_h = -ned[2]
        self.sensors.Vg = np.linalg.norm(ned_vel[0:2])
        self.sensors.gps_course = np.arctan2(ned_vel[1], ned_vel[0])

    def imu_callback(self, msg):
        # msg is in rad/s and m/s^2
        self.sensors.gyro_x = msg.angular_velocity.x
        self.sensors.gyro_y = msg.angular_velocity.y
        self.sensors.gyro_z = msg.angular_velocity.z
        self.sensors.accel_x = msg.linear_acceleration.x
        self.sensors.accel_y = msg.linear_acceleration.y
        self.sensors.accel_z = msg.linear_acceleration.z

    def mag_callback(self, msg):
        # msg is in Tesla
        self.sensors.mag_x = msg.magnetic_field.x
        self.sensors.mag_y = msg.magnetic_field.y
        self.sensors.mag_z = msg.magnetic_field.z

    def timer_callback(self):
        # Get next set of commands
        estimated_state = self.observer.update(self.sensors)
        path = self.path_manager.update(self.waypoints, estimated_state, PLAN.R_min)
        autopilot_commands = self.path_follower.update(path, estimated_state)
        delta, _ = self.autopilot.update(autopilot_commands, estimated_state)

        # Publish delta commands
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = Command.MODE_PASS_THROUGH
        msg.x = delta.aileron
        msg.y = delta.elevator
        msg.z = delta.rudder
        msg.f = delta.throttle
        self.delta_pub.publish(msg)


def main():
    rclpy.init()
    mavsim_bridge = MavSimBridge()
    rclpy.spin(mavsim_bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
