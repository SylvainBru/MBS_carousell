# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

import numpy as np

def user_DrivenJoints(mbs_data, tsim):
    """Set the values of the driven joints directly in the MbsData structure.

    The position, velocity and acceleration of the driven joints must be set in
    the attributes mbs_data.q, mbs_data.qd and mbs_data.qdd .

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """

    # Example: joint 5 under constant acceleration with non-zero initial
    #          coordinate (mbs_data.q0) and velocity (mbs_data.qd0).
    # mbs_data.qdd[5] = 2
    # mbs_data.qd[5]  = mbs_data.qd0[5] + mbs_data.qdd[5]*tsim
    # mbs_data.q[5]   = mbs_data.q0[5]  + mbs_data.qd0[5]*tsim + 0.5 * mbs_data.qdd[5]*tsim*tsim

    A = (2.5*2*np.pi)/360  # amplitude of the oscillation (rad)
    w = 0.4*np.pi  # pulsation of the oscillation (rad/s)
    phi1 = np.pi/2  # phase of the oscillation (rad)
    phi2 = 0  # phase of the oscillation (rad)

    theta1 = mbs_data.joint_id['Pole1']  # name of the joint to be driven
    theta2 = mbs_data.joint_id['Pole2']  # name of the joint to be driven

    mbs_data.q[theta1] = A*(1 - np.cos(w*tsim + phi1))
    mbs_data.qd[theta1] = A*w*np.sin(w*tsim + phi1)
    mbs_data.qdd[theta1] = A*w*w*np.cos(w*tsim + phi1)

    mbs_data.q[theta2] = A*(1 - np.cos(w*tsim + phi2))
    mbs_data.qd[theta2] = A*w*np.sin(w*tsim + phi2)
    mbs_data.qdd[theta2] = A*w*w*np.cos(w*tsim + phi2)

    return
