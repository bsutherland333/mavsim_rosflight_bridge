# Script to setup mavsim_bridge with mavsim source files

# Clone mavsim repository
git submodule update --init

# Copy mavsim source files to mavsim_bridge
cp -r mavsim/mavsim_python mavsim_bridge/mavsim_python

# Create a __init__.py file in every mavsim directory
find mavsim_bridge -type d -exec touch {}/__init__.py \;
