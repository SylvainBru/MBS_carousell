# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

import numpy as np

def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    #mbs_data.Qq[1:] = 0.


    # Example: damping in joint number 5
    # D = 0.5 # N/(m/s)
    # mbs_data.Qq[5] = -D * mbs_data.qd[5]

    mbs_data.Qq[1:] = 0.

    #print(mbs_data.Qq[1:])

    # Index du joint de rotation principale du pôle
    joint_pole = mbs_data.joint_id['Pole3']  # nom à adapter

    # Vitesse angulaire actuelle du pôle
    omega_pole = mbs_data.qd[joint_pole]

    # Paramètres du contrôleur
    omega_target = 0.8  # rad/s
    A = 500.0           # Nm
    omega_cos = 2 * np.pi  # rad/s

    # Initialisation t0 au premier appel
    if not hasattr(mbs_data, 'target_reached'):
        mbs_data.target_reached = False
        mbs_data.t0 = 0.0

    # Détection du premier passage à 0.8 rad/s
    if not mbs_data.target_reached and abs(omega_pole) >= omega_target:
        mbs_data.target_reached = True
        mbs_data.t0 = tsim

    # Calcul du couple selon le scénario
    if not mbs_data.target_reached:
        # Phase 1 : jamais atteint 0.8 rad/s
        T = 1000.0

    elif tsim < mbs_data.t0 + 0.5:
        # Phase 2 : transition douce
        phi = -2 * np.pi * mbs_data.t0
        T = A * (1 + np.cos(omega_cos * tsim + phi))

    else:
        # Phase 3 : moteur coupé
        T = 0.0

    #print(T)

    mbs_data.Qq[joint_pole] = T




    D_normal = 100.0    # Ns/m — pendules 1, 3, 4
    D_rusted = 20000.0  # Ns/m — pendule 2 (rouillé)

    hinge_pend1 = mbs_data.joint_id['arm_pend1']
    hinge_pend2 = mbs_data.joint_id['arm_pend2']
    hinge_pend3 = mbs_data.joint_id['arm_pend3']
    hinge_pend4 = mbs_data.joint_id['arm_pend4']

    mbs_data.Qq[hinge_pend1] += -D_normal * mbs_data.qd[hinge_pend1]
    mbs_data.Qq[hinge_pend2] += -D_rusted * mbs_data.qd[hinge_pend2]
    mbs_data.Qq[hinge_pend3] += -D_normal * mbs_data.qd[hinge_pend3]
    mbs_data.Qq[hinge_pend4] += -D_normal * mbs_data.qd[hinge_pend4]




    #Amortissement visqueux Cardan pendule-nacelle

    D_cardan = 6.0

    cardan_joints = [
        'R_cardan1a', 'R_cardan1b',  # nacelle 1
        'R_cardan2a', 'R_cardan2b',  # nacelle 2
        'R_cardan3a', 'R_cardan3b',  # nacelle 3
        'R_cardan4a', 'R_cardan4b',  # nacelle 4
    ]  

    for jname in cardan_joints:
        j = mbs_data.joint_id[jname]
        mbs_data.Qq[j] += -D_cardan * mbs_data.qd[j]

    return
