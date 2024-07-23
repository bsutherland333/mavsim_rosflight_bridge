import sys
import os
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_dir, 'mavsim/mavsim_python'))

import rclpy
from rclpy.node import Node
import numpy as np

from rosflight_msgs.msg import Command

import parameters.planner_parameters as PLAN
from controllers.autopilot import Autopilot
from estimators.observer import Observer
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_sensors import MsgSensors


class MavSimBridge(Node):
    def __init__(self):
        super().__init__('mavsim_bridge')
        control_timer_period = 1 / 100.0
        self.sensors = MsgSensors()

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

        # Create timer for control loop
        self.timer = self.create_timer(control_timer_period, self.timer_callback)

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
