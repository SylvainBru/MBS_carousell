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
    joint_pole = mbs_data.joint_id['R3_Pole']  # nom à adapter

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

    

    # --- Constantes ---
    # Amortissement de base des charnières (donné dans l'énoncé)
    D_hinge_normal = 100.0  # [cite: 152]
    D_hinge_rusted = 20000.0 # [cite: 152]

    # Équivalents calculés pour le vérin (ressort-amortisseur)
    D_rot_eq = 175.0  # Nm.s/rad
    K_rot_eq = 125.0  # Nm/rad

    # Le ressort réel est étiré de ~8.5m à l'arrêt, créant un couple permanent
    # Valeur calculée pour l'équilibre à 75°
    T_precharge = -2145.0 # Nm 

    # Angle initial en radians (75°)
    q_init = 75.0 * np.pi / 180.0 # [cite: 175, 186]

    # IDs des articulations (déjà définis dans ton code)
    hinge_pend1 = mbs_data.joint_id['R2_arm_pend1']
    hinge_pend2 = mbs_data.joint_id['R2_arm_pend2']
    hinge_pend3 = mbs_data.joint_id['arm_pend3']
    hinge_pend4 = mbs_data.joint_id['arm_pend4']

    hinges = [hinge_pend1, hinge_pend2, hinge_pend3, hinge_pend4]

    for i, h_id in enumerate(hinges):
        # 1. Sélection de l'amortissement de la charnière (spécifique au pendule 2)
        d_hinge = D_hinge_normal #D_hinge_rusted if i == 1 else D_hinge_normal
        
        # 2. Calcul du couple total
        # Terme 1 : Amortissement (Charnière + Équivalent vérin) * vitesse
        # Terme 2 : Rappel élastique équivalent * écart à la position initiale
        # Terme 3 : Le couple de précharge constant (très important !)
        
        torque_damping = -(d_hinge + D_rot_eq) * mbs_data.qd[h_id]
        torque_stiffness = -K_rot_eq * (mbs_data.q[h_id] - q_init)

        if(h_id in [ hinge_pend2, hinge_pend4]): 
            T_precharge = - T_precharge
        
        mbs_data.Qq[h_id] += torque_damping + torque_stiffness + T_precharge




    #Amortissement visqueux Cardan pendule-nacelle

    D_cardan = 6.0

    cardan_joints = [
        'R2_cardan1', 'R1_cardan1',  # nacelle 1
        'R_cardan2a', 'R_cardan2b',  # nacelle 2
        'R_cardan3a', 'R_cardan3b',  # nacelle 3
        'R_cardan4a', 'R_cardan4b',  # nacelle 4
    ]  

    for jname in cardan_joints:
        j = mbs_data.joint_id[jname]
        mbs_data.Qq[j] += -D_cardan * mbs_data.qd[j]

    return
