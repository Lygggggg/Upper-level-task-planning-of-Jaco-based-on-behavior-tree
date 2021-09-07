#!/usr/bin/env python
import numpy as np
np.seterr(divide='ignore') # these warnings are usually harmless for this code
from matplotlib import pyplot as plt
import copy, os

import pyhsmm
from pyhsmm.util.text import progprint_xrange

SAVE_FIGURES = False

###############
#  load origin_data  #
###############

data = np.loadtxt('/home/lni/Ros_ws/catkin_ws/src/hdp_hsmm/data/jiyuan.txt')

data = data[:, 0:3]

#########################
#  posterior inference  #
#########################

# Set the weak limit truncation level
Nmax = 10

# and some hyperparameters
obs_dim = data.shape[1]
obs_hypparams = {'mu_0': np.zeros(obs_dim),
                 'sigma_0': np.eye(obs_dim),
                 'kappa_0': 0.1,
                 'nu_0': obs_dim+3}
dur_hypparams = {'alpha_0': 8,
                 'beta_0': 5}

obs_distns = [pyhsmm.distributions.Gaussian(**obs_hypparams) for state in range(Nmax)]
dur_distns = [pyhsmm.distributions.PoissonDuration(**dur_hypparams) for state in range(Nmax)]

posteriormodel = pyhsmm.models.WeakLimitHDPHSMM(
    alpha=6, gamma=6,  # these can matter; see concentration-resampling.py
    init_state_concentration=6,
#    init_state_distn = 'uniform',# pretty inconsequential
    obs_distns=obs_distns,
    dur_distns=dur_distns)

posteriormodel.add_data(data)  # duration truncation speeds things up when it's possible

models = []
lik = []
for idx in progprint_xrange(150):
    posteriormodel.resample_model()
    lik.append(posteriormodel.log_likelihood())
    if (idx+1) % 10 == 0:
        models.append(copy.deepcopy(posteriormodel))
plt.figure()
plt.plot(lik)
plt.title('lik')

plt.figure()
#plt.subplot(3, 1, 1)
#posteriormodel.plot_observations()
#plt.title('Observations')
#plt.subplot(3, 1, 2)
posteriormodel.plot_stateseq(posteriormodel.states_list[0])
plt.title('State Seq')


#plt.subplot(3, 1, 3)
#posteriormodel.plot_durations()
#plt.title('Durations')

plt.show()
