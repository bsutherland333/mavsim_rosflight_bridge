# Script to setup mavsim_bridge with mavsim source files. This needs to be run from the repository root directory.

# Clone mavsim repository
git submodule update --init

# Create a __init__.py file in every mavsim_python directory, necessary for python (and therefore ROS) to recognize
# the directories as packages
touch mavsim_bridge/mavsim/__init__.py
find mavsim_bridge/mavsim/mavsim_python -type d -exec touch {}/__init__.py \;
