#!/usr/bin/env python
import numpy as np

# Module-scoped transformation matrices:
unity_T_enu = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0],
                        [0,0,0,1]])

lh_T_rh = np.array([[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])

gravity_vector = [0.0, 0.0, -9.81] # in 'world' frame
