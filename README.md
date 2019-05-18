# TESSE_interface

Provides a Python interface to the TESSE Unity environment.

Use python 2.7 (to ease usage with ROS)

## Commands
Use the following commands when having the Unity window in focus:

- Shift+T: disable keyboard input
- w,a,s,d: control camera using forces

## Setup

To use this interface, clone then setup the `tesse` package.
```
git clone git@github.mit.edu:TESS/TESSE_interface.git
cd TESSE_interface/python
python setup.py develop
```

To use this interface in ROS, you will need to import the package in the ROS folder into your catkin workspace.
If you have a catkin workspace already set up, just clone this repo in the `src` folder of the catkin worksapce, then:

```
cd TESSE_interface/python
python setup.py develop
catkin build
```

## Usage

See (python_demonstration.ipynb)[python_demonstration.ipynb] for example usage of the python package.

You can also find Ben's test script at 



### Disclaimer

Distribution authorized to U.S. Government agencies and their contractors. Other requests for this document shall be referred to the MIT Lincoln Laboratory Technology Office.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2019 Massachusetts Institute of Technology.

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
