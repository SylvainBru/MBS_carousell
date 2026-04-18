#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

import numpy as np


# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

import neri

# %%===========================================================================
# Project loading mbsdata
# =============================================================================
#mbs_data = Robotran.MbsData('../dataR/Merry_go_round.mbs')
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
mbs_data = Robotran.MbsData(mbs_path)

topology = neri.define_topology()
neri.check_topology_lengths(topology)  #Pour vérifier qu'on a bien le bon nombre de corps

#Condition initial (l'indice 0 contient le nombre de joints )
q0 = mbs_data.q0[1:]
qd0 = mbs_data.qd0[1:]


#omega, omega_c_dot, alpha_c, beta_c, O_M, A_M, R = neri.forward_kinematics(q0, qd0, topology, mbs_data)

#print(topology["inbody"])
#print(topology["phi"])
#print(topology["m"])


# # %%===========================================================================
# # Partitionning
# # =============================================================================
# mbs_data.process = 1
# mbs_part = Robotran.MbsPart(mbs_data)
# mbs_part.set_options(rowperm=1, verbose=1)
# mbs_part.run()

# # %%===========================================================================
# # Direct Dynamics
# # =============================================================================
# mbs_data.process = 3
# mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# mbs_dirdyn.set_options(dt0=1e-3, tf=5, save2file=1)
# results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')
