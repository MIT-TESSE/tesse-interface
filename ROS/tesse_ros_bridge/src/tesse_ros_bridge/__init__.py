#!/usr/bin/env python
import numpy as np

# Module-scoped transformation matrices:
# TODO(marcus): naming conventions!!!
enu_T_unity = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0],
                        [0,0,0,1]])

unity_T_enu = np.transpose(enu_T_unity)

brh_T_blh = np.array([[1,0,0,0],
                      [0,-1,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

blh_T_brh = np.transpose(brh_T_blh)

gravity_enu = [0.0, 0.0, -9.81] # in 'world' frame
