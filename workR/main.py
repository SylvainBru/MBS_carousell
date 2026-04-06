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

topology = neri.extract_topology_from_mbs(mbs_path)

print(topology)




# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')
