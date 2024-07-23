# mavsim_rosflight_bridge

ROS2 bridge for running the [mavsim](https://github.com/randybeard/mavsim_public) autonomy stack with the ROSflight firmware and simulation.

This repo includes the mavsim (non-public version) repository as a git submodule that is built and installed as a ROS package.
A ROS node (mavsim_bridge) then runs mavsim internally while exchanging sensor data and actuator commands with ROSflight over the ROS network.

To use this bridge, set up a ROS2 workspace with [rosflight_ros_pkgs](https://github.com/rosflight/rosflight_ros_pkgs) in the workspace. Run the setup_mavsim.sh script from the root directory of this repository. Build both packages, and run mavsim_bridge with `ros2 run mavsim_bridge mavsim_bridge` alongside rosflight_io.

To adapt this bridge to different autopilots (like PX4), you would first need to make sure the other autopilot is setup to send sensor data and recieve actuator commands over the ROS network, with something like mavros or microROS.
Then, modify the sensor subscriptions and command publisher in mavsim_bridge/mavsim_bridge.py to recieve data and send commands to whatever topics the autopilot is using.

On my Ryzen 7 7840u laptop, I was able to get the mavsim bridge running at over 400 hz with the full state estimator thanks to python and ROS pre-compiling python packages into .pyc files, which runs much faster than regular .py files.
