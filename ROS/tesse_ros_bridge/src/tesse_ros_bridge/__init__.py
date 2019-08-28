#!/usr/bin/env python
import numpy as np

# Module-scoped transformation matrices:
# TODO(marcus): naming conventions!!!
enu_T_unity = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0],
                        [0,0,0,1]])

brh_T_blh = np.array([[1,0,0,0],
                      [0,-1,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

gravity_enu = [0.0, 0.0, -9.81] # in 'world' frame
