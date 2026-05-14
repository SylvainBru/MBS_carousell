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


# loading the data file
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
mbs_data = Robotran.MbsData(mbs_path)


# partitioning
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# direct dynamic
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=2e-3, tf=3, save2file=1)
results = mbs_dirdyn.run()



# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')


t0 = mbs_dirdyn.get_options('t0')
tf = mbs_dirdyn.get_options('tf')

joint_pole  = mbs_data.joint_id['R3_Pole']
joint_pend1 = mbs_data.joint_id['R2_arm_pend1']
joint_pend2 = mbs_data.joint_id['arm_pend2']
joint_pend3 = mbs_data.joint_id['arm_pend3']
joint_pend4 = mbs_data.joint_id['arm_pend4']
joint_r1    = mbs_data.joint_id['R1_Pole']
joint_r2    = mbs_data.joint_id['R2_Pole']

time = results.qd[:, 0]

_plot_dir = os.path.join(script_dir, "dirdyn_plot")
os.makedirs(_plot_dir, exist_ok=True)

_comp_dir = os.path.join(script_dir, '..', '..', 'comparaison_neri_vs_dirdyn')
os.makedirs(_comp_dir, exist_ok=True)
np.save(os.path.join(_comp_dir, 'dirdyn_results.npy'),
        np.column_stack((time, np.rad2deg(results.qd[:, joint_pole]))))

def save(fig, name):
    fig.savefig(os.path.join(_plot_dir, name + ".png"), dpi=150)
    plt.close(fig)

# ── Plot 1 : vitesse angulaire pôle principal ─────────────────────────
fig, axis = plt.subplots(figsize=(9, 5))
axis.plot(time, np.rad2deg(results.qd[:, joint_pole]), label='Main pole angular velocity')
axis.grid(True)
axis.set_xlim(t0, tf)
axis.set_xlabel('Time (s)')
axis.set_ylabel('Angular velocity [deg/s]')
fig.suptitle('Robotran')
axis.legend()
fig.tight_layout()
save(fig, "01_pole_velocity")

# ── Plot 2 : angles des pendules ──────────────────────────────────────
fig, axis = plt.subplots(figsize=(9, 5))
axis.plot(time, np.rad2deg(results.q[:, joint_pend1]), label='Pendule 1')
axis.plot(time, np.rad2deg(results.q[:, joint_pend2]), label='Pendule 2 (rouillé)')
axis.plot(time, np.rad2deg(results.q[:, joint_pend3]), label='Pendule 3')
axis.plot(time, np.rad2deg(results.q[:, joint_pend4]), label='Pendule 4')
axis.grid(True)
axis.set_xlim(t0, tf)
axis.set_xlabel('Time (s)')
axis.set_ylabel('Angle [deg]')
fig.suptitle('Robotran')
axis.legend()
fig.tight_layout()
save(fig, "02_pendulum_angles")

# ── Plot 3 : vitesses angulaires des pendules ─────────────────────────
fig, axis = plt.subplots(figsize=(9, 5))
axis.plot(time, np.rad2deg(results.qd[:, joint_pend1]), label='Pendule 1')
axis.plot(time, np.rad2deg(results.qd[:, joint_pend2]), label='Pendule 2 (rouillé)')
axis.plot(time, np.rad2deg(results.qd[:, joint_pend3]), label='Pendule 3')
axis.plot(time, np.rad2deg(results.qd[:, joint_pend4]), label='Pendule 4')
axis.grid(True)
axis.set_xlim(t0, tf)
axis.set_xlabel('Time (s)')
axis.set_ylabel('Angular velocity [deg/s]')
fig.suptitle('Robotran')
axis.legend()
fig.tight_layout()
save(fig, "03_pendulum_velocities")

# ── Plot 4 : joints commandés (inclinaison pôle) ──────────────────────
fig, axis = plt.subplots(figsize=(9, 5))
axis.plot(time, np.rad2deg(results.q[:, joint_r1]), label='q2 — R1_Pole')
axis.plot(time, np.rad2deg(results.q[:, joint_r2]), label='q3 — R2_Pole')
axis.grid(True)
axis.set_xlim(t0, tf)
axis.set_xlabel('Time (s)')
axis.set_ylabel('Angle [deg]')
fig.suptitle('Robotran')
axis.legend()
fig.tight_layout()
save(fig, "04_driven_joints")

plt.show()
