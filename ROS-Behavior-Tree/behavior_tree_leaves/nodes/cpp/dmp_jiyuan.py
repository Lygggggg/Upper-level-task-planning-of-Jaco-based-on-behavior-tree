# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pydmps.dmp_discrete


def DMP_generation(path,start_point,end_point):

    data = np.loadtxt(path).T
    DMP = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500)

    DMP.imitate_path(y_des=data)
    DMP.reset_state()
    DMP.y0 = start_point
    DMP.goal = end_point

    track, _, _ = DMP.rollout(tau=0.8)
    track = track.flatten()
    track = list(track)
    print(type(track))
    return track



#    x = track[i,j]
#    return x




