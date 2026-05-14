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
neri.validate_topology(topology, "topology")
'''topology = neri.define_topology_from_mbs(mbs_path)
neri.validate_topology(topology, "topology from mbs")'''
print("Simulation started")

# Partitioning
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

u = np.array(mbs_data.qu[1:mbs_data.nqu + 1], dtype=int)
c = np.array(mbs_data.qc[1:mbs_data.nqc + 1], dtype=int)

q0 = np.array(mbs_data.q0, dtype=float)
qd0 = np.array(mbs_data.qd0, dtype=float)

y0 = np.concatenate((q0[u], qd0[u]))


t0 = 0.0
tf = 1.0
dt = 2e-3

motor_state = {"t0_reach": None}

t_vec, y_vec = neri.rk4_integrate(
    neri.rhs_neri,
    t0, tf, y0, dt, 
    mbs_data, topology, u, c, motor_state
)

nu = len(u)

idx_q1 = np.where(u == 1)[0][0]
qd1_vec = y_vec[:, nu + idx_q1]
qd1_deg = qd1_vec * 180.0 / np.pi

print("\n--- simulation summary ---")
print("Any NaN in y_vec?", np.isnan(y_vec).any())
print("max abs y =", np.max(np.abs(y_vec)))
print("motor t0_reach =", motor_state["t0_reach"])
print("max qd1 rad/s =", np.max(qd1_vec))
print("max qd1 deg/s =", np.max(qd1_deg))
print("final qd1 rad/s =", qd1_vec[-1])
print("final qd1 deg/s =", qd1_deg[-1])

try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError("Unable to load matplotlib, plotting results unavailable.")

plt.figure()
plt.plot(t_vec, qd1_deg, label="Main pole angular velocity")
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [deg/s]")
plt.title("Main pole angular velocity around vertical axis")
plt.grid(True)
plt.legend()
plt.show()

q_u_vec = y_vec[:, :nu]
qd_u_vec = y_vec[:, nu:]

plt.figure()
for idx in range(nu):
    plt.plot(t_vec, q_u_vec[:, idx], label=f"q{u[idx]}")

plt.xlabel("Time [s]")
plt.ylabel("q_u [rad or m]")
plt.title("NERi short integration: independent coordinates")
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
for joint in [4, 5, 6, 11, 13, 16, 19]:
    idx = np.where(u == joint)[0][0]
    plt.plot(t_vec, q_u_vec[:, idx], label=f"q{joint}")

plt.xlabel("Time [s]")
plt.ylabel("q_u [rad or m]")
plt.title("NERi short integration: selected independent coordinates")
plt.grid(True)
plt.legend()
plt.show()
