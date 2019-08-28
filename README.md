# TESSE_interface

Provides a Python interface to the TESSE Unity environment.

Use python 2.7 (to ease usage with ROS)

## Commands
Use the following commands when having the Unity window in focus:

- Shift+T: disable keyboard input
- w,a,s,d: control agent using forces
- x: stops motion of agent.
- r: respawns agent.
- 'left ctrl + left shift + g': enter spawn point capture mode for next respawn: press 'r' until you get to a nice location, then enter capture mode so that Unity restarts where you left. Note that in this mode, it'll capture a new point every second while moving around. You can stop capturing using 'left ctrl + left shift + g'.
- ESC: to quit game.

## Setup

To use this interface, clone then setup the `tesse` package.
```bash
git clone git@github.mit.edu:TESS/TESSE_interface.git
cd TESSE_interface/python
python setup.py develop
```

### Setup for ROS
To use this interface in ROS, you will need to import the package in the ROS folder into your catkin workspace.

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Clone repo
cd src
git clone git@github.mit.edu:TESS/TESSE_interface.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge wstool merge TESSE_interface/ROS/tesse_ros_bridge/install/tesse_ros_bridge.rosinstall 
wstool update

# Source TESSE non-ROS code
cd TESSE_interface/python
python setup.py develop
cd ../..

# Compile code
catkin build

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Refresh environment
source ~/.bashrc
```

## Usage

To run the ROS node:
```bash
roslaunch tesse_ros_bridge tesse_ros_bridge.launch
```

See (python_demonstration.ipynb)[python_demonstration.ipynb] for example usage of the python package.

You can also find Ben's test script at 

### Plotting

You can use rviz for general visualization, we provide a configuration file:
```bash
rviz -r ./ROS/tesse_ros_bridge/rviz/tesse_semantic_mesh.rviz
```

You can also plot IMU and ground-truth odometry from the simulator using `rqt_multiplot` and the config file we provide:
```bash
rqt_multiplot --multiplot-config:=rqt_multiplot/tesse.xml
```

### Disclaimer

Distribution authorized to U.S. Government agencies and their contractors. Other requests for this document shall be referred to the MIT Lincoln Laboratory Technology Office.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2019 Massachusetts Institute of Technology.

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
