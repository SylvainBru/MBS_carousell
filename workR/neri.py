import numpy as np 
import MBsysPy as Robotran
import xml.etree.ElementTree as ET


######################################################
####              Extracte topology             ######
######################################################


def define_topology():
    """
    Definition de la topologie sur base du pad robotran nécessaires au formalisme NER (inbody, phi(joint rotation), psy (joint translation), d_hi, masse, l_ci, Inertie).
    Les vecteurs sont structurés pour un accès via var[i] = [x, y, z].
    L'index est le corps (fictif ou non associé au valeur)
    Le premier corps est la base (index 0) (Pas de parent)
    """
    inbody_name_list = ["base", "R3_Pole", "R1_Pole", "R2_Pole", "R2_arm_pend1", "R2_cardan1a", "R1_cardan1", "R2_arm_pend2", "T1_effort_normal", "T3_effort_tranchant", "R2_effort_flechissant"    ]
    inbody_list = np.array([None, 0, 1, 2, 3, 4, 5, 3, 7, 8, 9, 10, 11, 12, 3, 14, 15, 3, 17, 18])

    phi_list = [
    np.array([0.0, 0.0, 0.0]),  # 0: base
    np.array([0, 0, 1]),           # 1: R3_Pole 
    np.array([1, 0, 0]),           # 2: R1_Pole
    np.array([0, 1, 0]),           # 3: R2_Pole 
    np.array([0, 1, 0]),           # 4: R2_arm_pend1
    np.array([0, 1, 0]),           # 5: R2_cardan1 
    np.array([1, 0, 0]),           # 6: R1_cardan1 
    np.array([0, 0, 0]),           # 7: R2_arm_pend2 
    np.array([0, 0, 0]),           # 8: T1_effort_normal 
    np.array([0, 0, 0]),           # 9: T3_effort_tranchant 
    np.array([0, 1, 0]),           # 10: R2_effort_flechissant
    np.array([1, 0, 0]),           # 11: arm_pend4
    np.array([1, 0, 0]),           # 12: R_cardan4a 
    np.array([0, 1, 0]),           # 13: R_cardan4b 
    np.array([0, 1, 0]),           # 14: arm_pend3
    np.array([0, 1, 0]),           # 15: R_cardan3a 
    np.array([1, 0, 0]),           # 16: R_cardan3b 
    np.array([1, 0, 0]),           # 17: arm_pend2 
    np.array([1, 0, 0]),           # 18: R_cardan2a 
    np.array([0, 1, 0])           # 19: R_cardan2b 
    ]


    psi_list = [
    np.array([0.0, 0.0, 0.0]),                           # 0: base
    np.array([0, 0, 0]),           # 1: R3_Pole (no translation)
    np.array([0, 0, 0]),           # 2: R1_Pole (no translation)
    np.array([0, 0, 0]),           # 3: R2_Pole (no translation)
    np.array([0, 0, 0]),           # 4: R2_arm_pend1 (no translation)
    np.array([0, 0, 0]),           # 5: R2_cardan1 (no translation)
    np.array([0, 0, 0]),           # 6: R1_cardan1 (no translation)
    np.array([1, 0, 0]),           # 7: R2_arm_pend2 (T1 translation along X)
    np.array([1, 0, 0]),           # 8: T1_effort_normal (translation along X)
    np.array([0, 0, 1]),           # 9: T3_effort_tranchant (translation along Z)
    np.array([0, 0, 0]),           # 10: R2_effort_flechissant (no translation)
    np.array([0, 0, 0]),           # 11: arm_pend4 (no translation)
    np.array([0, 0, 0]),           # 12: R_cardan4a (no translation)
    np.array([0, 0, 0]),           # 13: R_cardan4b (no translation)
    np.array([0, 0, 0]),           # 14: arm_pend3 (no translation)
    np.array([0, 0, 0]),           # 15: R_cardan3a (no translation)
    np.array([0, 0, 0]),           # 16: R_cardan3b (no translation)
    np.array([0, 0, 0]),           # 17: arm_pend2 (no translation)
    np.array([0, 0, 0]),           # 18: R_cardan2a (no translation)
    np.array([0, 0, 0])            # 19: R_cardan2b (no translation)
    ]

    d_hi_list = [
    np.array([0.0, 0.0, 0.0]),                           # 0: base
    np.array([0, 0, 0]),           # 1: R3_Pole (no translation)
    np.array([0, 0, 0]),           # 2: R1_Pole (no translation)
    np.array([1, 0, 4.5]),           # 3: R2_Pole (no translation)
    np.array([0, 0, -3]),           # 4: R2_arm_pend1 (no translation)
    np.array([0, 0, 0]),           # 5: R2_cardan1 (no translation)
    np.array([0, 0, -1]),           # 6: R1_cardan1 (no translation)
    np.array([0, -1, 0]),           # 7: R2_arm_pend2 (T1 translation along X)
    np.array([0, 0, 0]),           # 8: T1_effort_normal (translation along X)
    np.array([0, 0, 0]),           # 9: T3_effort_tranchant (translation along Z)
    np.array([0, 0, 0]),           # 10: R2_effort_flechissant (no translation)
    np.array([0, 0, -3]),           # 11: arm_pend4 (no translation)
    np.array([0, 0, 0]),           # 12: R_cardan4a (no translation)
    np.array([0, 0, -1]),           # 13: R_cardan4b (no translation)
    np.array([0, 0, -3]),           # 14: arm_pend3 (no translation)
    np.array([0, 0, 0]),           # 15: R_cardan3a (no translation)
    np.array([0, 0, -1]),           # 16: R_cardan3b (no translation)
    np.array([0, 0, -3]),           # 17: arm_pend2 (no translation)
    np.array([0, 0, 0]),           # 18: R_cardan2a (no translation)
    np.array([0, 0, -1])            # 19: R_cardan2b (no translation)
    ]

    z_list = [
    np.array([0.0, 0.0, 0.0]),                           # 0: base
    np.array([0, 0, 0]),           # 1: R3_Pole (no translation)
    np.array([0, 0, 0]),           # 2: R1_Pole (no translation)
    np.array([0, 0, 0]),           # 3: R2_Pole (no translation)
    np.array([0, 0, 0]),           # 4: R2_arm_pend1 (no translation)
    np.array([0, 0, 0]),           # 5: R2_cardan1 (no translation)
    np.array([0, 0, 0]),           # 6: R1_cardan1 (no translation)
    np.array([0, 0, 0]),           # 7: R2_arm_pend2 (T1 translation along X)
    np.array([0, 0, 0]),           # 8: T1_effort_normal (translation along X)
    np.array([0, 0, 0]),           # 9: T3_effort_tranchant (translation along Z)
    np.array([0, 0, 0]),           # 10: R2_effort_flechissant (no translation)
    np.array([0, 0, 0]),           # 11: arm_pend4 (no translation)
    np.array([0, 0, 0]),           # 12: R_cardan4a (no translation)
    np.array([0, 0, 0]),           # 13: R_cardan4b (no translation)
    np.array([0, 0, 0]),           # 14: arm_pend3 (no translation)
    np.array([0, 0, 0]),           # 15: R_cardan3a (no translation)
    np.array([0, 0, 0]),           # 16: R_cardan3b (no translation)
    np.array([0, 0, 0]),           # 17: arm_pend2 (no translation)
    np.array([0, 0, 0]),           # 18: R_cardan2a (no translation)
    np.array([0, 0, 0])            # 19: R_cardan2b (no translation)
    ]
    
    m_list = [
    0.0,                            # 0: base
    0.0,                          # 1: R3_Pole (mass of Pole)
    0.0,                          # 2: R1_Pole
    600.0,                          # 3: R2_Pole
    44.0,                           # 4: R2_arm_pend1 (mass of Pendule1)
    0.0,                          # 5: R2_cardan1 (mass of nacelle1)
    123.3,                          # 6: R1_cardan1
    30.0,                           # 7: R2_arm_pend2 (mass of arm_part1)
    0.0,                            # 8: T1_effort_normal (mass of arm_part2)
    0.0,                            # 9: T3_effort_tranchant
    0.0,                            # 10: R2_effort_flechissant
    44.0,                           # 11: arm_pend4 (mass of Pendule4)
    0,                          # 12: R_cardan4a (mass of nacelle4)
    123.3,                          # 13: R_cardan4b
    44.0,                           # 14: arm_pend3 (mass of Pendule3)
    0,                          # 15: R_cardan3a (mass of nacelle3)
    123.3,                          # 16: R_cardan3b
    44.0,                           # 17: arm_pend2 (mass of Pendule2)
    0,                          # 18: R_cardan2a (mass of nacelle2)
    123.3                           # 19: R_cardan2b
    ]


    d_ii_list = [
    np.array([0.0, 0.0, 0.0]),     # 0: base
    np.array([0.0, 0.0, 0.0]),     # 1: R3_Pole (COM of Pole)
    np.array([0.0, 0.0, 0.0]),     # 2: R1_Pole
    np.array([0.0, 0.0, 2.0]),     # 3: R2_Pole
    np.array([0.0, 0.0, -1.5]),    # 4: R2_arm_pend1 (COM of Pendule1)
    np.array([0.0, 0.0, 0.0]),    # 5: R2_cardan1 (COM of nacelle1)
    np.array([0.0, 0.0, -1.0]),    # 6: R1_cardan1
    np.array([-0.5, 0.0, 0.0]),    # 7: R2_arm_pend2 (COM of arm_part1)
    np.array([0.0, 0.0, 0.0]),     # 8: T1_effort_normal (COM of arm_part2)
    np.array([0.0, 0.0, 0.0]),     # 9: T3_effort_tranchant
    np.array([0.0, 0.0, 0.0]),     # 10: R2_effort_flechissant
    np.array([0.0, 0.0, -1.5]),    # 11: arm_pend4 (COM of Pendule4)
    np.array([0.0, 0.0, 0.0]),    # 12: R_cardan4a (COM of nacelle4)
    np.array([0.0, 0.0, -1.0]),    # 13: R_cardan4b
    np.array([0.0, 0.0, -1.5]),    # 14: arm_pend3 (COM of Pendule3)
    np.array([0.0, 0.0, 0.0]),    # 15: R_cardan3a (COM of nacelle3)
    np.array([0.0, 0.0, -1.0]),    # 16: R_cardan3b
    np.array([0.0, 0.0, -1.5]),    # 17: arm_pend2 (COM of Pendule2)
    np.array([0.0, 0.0, 0.0]),    # 18: R_cardan2a (COM of nacelle2)
    np.array([0.0, 0.0, -1.0])     # 19: R_cardan2b
    ]


    I_list = [
    np.zeros((3, 3)),               # 0: base
    np.zeros((3, 3)),           # 1: R3_Pole (Inertia of Pole)
    np.zeros((3, 3)),           # 2: R1_Pole
    np.diag([458.8, 458.8, 16.0]),  # 3: R2_Pole
    np.diag([15.9, 15.9, 0.091]),   # 4: R2_arm_pend1 (Inertia of Pendule1)
    np.zeros((3, 3)),           # 5: R2_cardan1 (Inertia of nacelle1)
    np.diag([15.4, 15.4, 61.7]),    # 6: R1_cardan1
    np.diag([1.0, 1.0, 1.0]),       # 7: R2_arm_pend2 (Inertia of arm_part1)
    np.zeros((3, 3)),               # 8: T1_effort_normal (Inertia of arm_part2)
    np.zeros((3, 3)),               # 9: T3_effort_tranchant
    np.zeros((3, 3)),               # 10: R2_effort_flechissant
    np.diag([15.9, 15.9, 0.091]),   # 11: arm_pend4 (Inertia of Pendule4)
    np.zeros((3, 3)),           # 12: R_cardan4a (Inertia of nacelle4)
    np.diag([15.4, 15.4, 61.7]),    # 13: R_cardan4b
    np.diag([15.9, 15.9, 0.091]),   # 14: arm_pend3 (Inertia of Pendule3)
    np.zeros((3, 3)),           # 15: R_cardan3a (Inertia of nacelle3)
    np.diag([15.4, 15.4, 61.7]),    # 16: R_cardan3b
    np.diag([15.9, 15.9, 0.091]),   # 17: arm_pend2 (Inertia of Pendule2)
    np.zeros((3, 3)),           # 18: R_cardan2a (Inertia of nacelle2)
    np.diag([15.4, 15.4, 61.7])     # 19: R_cardan2b
    ]





    topology = {
        # Topologie
        "inbody": np.array(inbody_list, dtype=object),             # Taille: (n+1,)
        "phi":    np.array(phi_list, dtype=float),                # Taille: (n+1, 3) --> phi[i] = [x,y,z]
        "psi":    np.array(psi_list, dtype=float),                # Taille: (n+1, 3) --> psi[i] = [x,y,z]
        "d_hi":   np.array(d_hi_list, dtype=float),               # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]
        "z":      np.array(z_list, dtype=float),                  # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]

        # Dynamique
        "m":      np.array(m_list, dtype=float),                  # Taille: (n+1,)     --> m[i] = scalaire
        "d_ii":   np.array(d_ii_list, dtype=float),               # Taille: (n+1, 3) --> d_ii[i] = [x,y,z]
        "I":      np.array(I_list, dtype=float)                   # Taille: (n+1, 3, 3)--> I[i] = matrice 3x3
    }
    
    return topology
    
def validate_topology(topology, name="topology"):
    """
    Print useful checks for manual topology.
    """
    m = topology["m"]
    d_ii = topology["d_ii"]
    I = topology["I"]
    joint_names = topology.get("joint_names", None)

    print(f"\n--- validate {name} ---")

    total_mass = 0.0

    for i in range(1, len(m)):
        mi = float(m[i])
        Ii = np.array(I[i], dtype=float)
        di = np.array(d_ii[i], dtype=float)

        has_inertia = np.linalg.norm(Ii) > 1e-12
        has_com = np.linalg.norm(di) > 1e-12

        label = f"q{i}"
        if joint_names is not None and i < len(joint_names):
            label += f" {joint_names[i]}"

        if abs(mi) > 1e-12 or has_inertia or has_com:
            print(
                label,
                "m =", mi,
                "d_ii =", di,
                "I_diag =", np.diag(Ii),
            )

        if abs(mi) < 1e-12 and has_inertia:
            print("  WARNING: inertia on massless joint")

        if abs(mi) < 1e-12 and has_com:
            print("  WARNING: COM offset on massless joint")

        total_mass += mi

    print("total mass =", total_mass)





######################################################
####              Fonction utile                ######
######################################################

def tilde(v): 
    """
    Return: n x n anti-symétrique matrice associé au vecteur v
    """
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0]
    ])


def rotation_matrix(phi, psi, q):
    """
    Calcule la matrice de rotation autour d'un axe local (x, y ou z).

    Si phi = [ 0, 0, 0] et psi = [ 0, 0, 0] alors return Identité 
    Si phi != [0 , 0, 0] alors il y a une rotation 
    """
    phi = np.array(phi, dtype=float)
    psi = np.array(psi, dtype=float)
    q = float(q)
        
    c = np.cos(q)
    s = np.sin(q)

    if np.allclose(phi, np.zeros(3)): 
        return np.eye(3)
    else:  
        if phi[0] == 1.0:   # Autour de X
            return np.array([[1, 0, 0], 
                            [0, c, -s], 
                            [0, s, c]])
        elif phi[1] == 1.0: # Autour de Y
            return np.array([[c, 0, -s], 
                            [0, 1, 0], 
                            [s, 0, c]])
        elif phi[2] == 1.0: # Autour de Z
            return np.array([[c, -s, 0], 
                            [s, c, 0], 
                            [0, 0, 1]])


######################################################
####             NERi Formalism                 ######
######################################################

def forward_kinematics(q, qd, topology, mbs_data: Robotran.MbsData, debug=False): 
    """
        Etape 1: Parcourt l'arbre de la base vers les feuilles
        Calcule les vitesses et accélérations dans chaque corps. 

    Returns
    -------
    omega       : absolute angular velocity
    omega_c_dot : convective angular acceleration part
    alpha_c     : convective linear acceleration part
    beta_c      : omega_tilde_dot + omega_tilde^2 equivalent
    O_M, A_M    : recursive matrices for mass matrix construction
    R           : relative rotation matrices
    """

    N_body = mbs_data.njoint 

    inbody = topology["inbody"]
    phi    = topology["phi"]
    psi    = topology["psi"]
    d_hi   = topology["d_hi"]

    #Allocations mémoires (taille n+1 pour inclure la base à l'index 0)
    omega       = np.zeros((N_body + 1, 3))
    omega_c_dot = np.zeros((N_body + 1, 3))
    alpha_c     = np.zeros((N_body + 1, 3))
    beta_c      = np.zeros((N_body + 1, 3,3))
    R           = np.zeros((N_body + 1, 3, 3)) # Matrices de rotation R^{i,h}

    R_abs = np.zeros((N_body + 1, 3, 3))
    p_abs = np.zeros((N_body + 1, 3))
    #print(R)

    O_M = np.zeros((N_body + 1, N_body + 1, 3))  # matrice de taille N * N ou chaque termes est un vecteur de taille 3
    A_M = np.zeros((N_body + 1, N_body + 1, 3))

    #Condition initial

    #alpha_c[0]  = mbs_data.g[1:4] # [0, 0, -9.81]

    #les conditions initial mis à 0 sont fait par le np.zeros

    #Note: Attention q[i] pour la matrice de rotation si c'est une translation R = I
    # base
    R[0] = np.eye(3)
    alpha_c[0] = -np.array(mbs_data.g[1:4], dtype=float)
    z = np.zeros((N_body + 1, 3))

    R_abs[0] = np.eye(3)
    p_abs[0] = np.zeros(3)

    for i in range(1, N_body + 1):
        h = int(inbody[i])

        # relative rotation of joint i
        R_ih  = rotation_matrix(phi[i], psi[i], q[i])
        R[i] = R_ih 

        z[i] = psi[i] * q[i]
        R_abs[i] = R_abs[h] @ R_ih.T
        p_abs[i] = p_abs[h] + R_abs[h] @ (d_hi[i] + z[i])

        # angular velocity recursion
        omega[i] = R_ih @ omega[h] + phi[i] * qd[i]
        
        # convective angular acceleration recursion
        omega_c_dot[i] = R_ih @ omega_c_dot[h] + tilde(omega[i]) @ (phi[i] * qd[i])

        # beta_c = ω~dot + ω~ω~
        beta_c[i] = tilde(omega_c_dot[i]) + tilde(omega[i]) @ tilde(omega[i])

        # convective linear acceleration recursion
        alpha_c[i] = (
            R_ih @ (alpha_c[h] + beta_c[h] @ (z[h] + d_hi[i]))
            + 2.0 * tilde(omega[i]) @ (psi[i] * qd[i])
        )
        

        for k in range(1, i + 1):
            delta_ki = 1.0 if k == i else 0.0
            O_M[i, k] = R_ih @ O_M[h, k] + delta_ki * phi[i]
            A_M[i, k] = R_ih @ (A_M[h, k] + tilde(O_M[h, k]) @ (z[h] + d_hi[i])) + delta_ki * psi[i]
    #print("O_M[10,:,:] =\n", O_M[10])
    #print("A_M[10,:,:] =\n", A_M[10])  
    #print(O_M[4])
    #print(A_M[4])
    if debug:
        print("Ancestors of 10 should be:")
        h = 10
        while h != 0:
           print(h, "parent =", inbody[h])
           h = int(inbody[h])

        print("\nNonzero O_M rows for body 10:")
        for k in range(1, N_body + 1):
            if not np.allclose(O_M[10, k], np.zeros(3)):
               print("k =", k, "O_M[10,k] =", O_M[10, k])

        print("\nNonzero A_M rows for body 10:")
        for k in range(1, N_body + 1):
            if not np.allclose(A_M[10, k], np.zeros(3)):
               print("k =", k, "A_M[10,k] =", A_M[10, k])

    return omega, omega_c_dot, alpha_c, beta_c,z, O_M, A_M, R, R_abs, p_abs

def backward_dynamics(topology, mbs_data,
                      omega, omega_c_dot, alpha_c, beta_c, z, R,O_M, A_M,
                      F_ext=None, L_ext=None, debug=False):
    """
    Etape 2: Parcourt de l'arbre des feuilles vers la base 
    Calcule les forces et couples aux articulations, et projette pour obtenir Q. 
    - computes c-part via W_c, F_c, L_c, Q
    - computes mass matrix via W_M, F_M, L_M, M
    
    """
    Nbody = mbs_data.njoint

    inbody = topology["inbody"]
    phi    = topology["phi"]
    psi    = topology["psi"]
    d_hi   = topology["d_hi"]
    d_ii   = topology["d_ii"]
    m      = topology["m"]
    I      = topology["I"]

    # external loads, default = zero
    if F_ext is None:
        F_ext = np.zeros((Nbody + 1, 3))
    if L_ext is None:
        L_ext = np.zeros((Nbody + 1, 3))

    # outputs
    W_c = np.zeros((Nbody + 1, 3))
    F_c  = np.zeros((Nbody + 1, 3))   # joint force transmitted to parent
    L_c = np.zeros((Nbody + 1, 3))   # joint torque transmitted to parent
    Q = np.zeros(Nbody + 1)        # generalized force


    W_M = np.zeros((Nbody + 1, Nbody + 1, 3))
    F_M = np.zeros((Nbody + 1, Nbody + 1, 3))
    L_M = np.zeros((Nbody + 1, Nbody + 1, 3))
    M   = np.zeros((Nbody + 1, Nbody + 1))

    for i in range(Nbody, 0, -1):

        d_z_ii  = z[i] + d_ii[i]
        W_c[i] = m[i] * (alpha_c[i] + beta_c[i] @ d_z_ii) - F_ext[i]
        F_c[i] = W_c[i].copy() 


        for j in range(1, Nbody + 1): #summation over the children of body i.
            if inbody[j] == i:
                # R[j] = R^{j,i}(q^j), i.e. child frame j -> parent frame i
                R_ji = R[j]
                F_c[i] += R_ji.T @ F_c[j]

        L_c[i] = (
            tilde(d_z_ii) @ W_c[i]
            - L_ext[i]
            + I[i] @ omega_c_dot[i]
            + tilde(omega[i]) @ (I[i] @ omega[i])
        )

        for j in range(1, Nbody + 1):
            if inbody[j] == i:
                d_z_ij = z[i] + d_hi[j]
                R_ji = R[j]
                F_child_in_i = R_ji.T @ F_c[j]
                L_c[i] += R_ji.T @ L_c[j] + tilde(d_z_ij) @ F_child_in_i

        Q[i] = F_c[i] @ psi[i] + L_c[i] @ phi[i]



        # --------------------------------
        # M-part; sym
        # --------------------------------
        for k in range(1, i + 1):

            W_M[i, k] = m[i] * (A_M[i, k] + tilde(O_M[i, k]) @ d_z_ii)
            F_M[i, k] = W_M[i, k].copy()

            for j in range(1, Nbody + 1):
                if inbody[j] == i:
                    R_ji = R[j]
                    F_M[i, k] += R_ji.T @ F_M[j, k]


            L_M[i, k] = tilde(d_z_ii) @ W_M[i, k] + I[i] @ O_M[i, k]

            for j in range(1, Nbody + 1):
                if inbody[j] == i:
                    d_z_ij = z[i] + d_hi[j]
                    R_ji = R[j]
                    F_child_in_i = R_ji.T @ F_M[j, k]
                    L_M[i, k] += R_ji.T @ L_M[j, k] + tilde(d_z_ij) @ F_child_in_i


            M[i, k] = F_M[i, k] @ psi[i] + L_M[i, k] @ phi[i]
    M = M + np.tril(M, -1).T
            
    if debug:
        print("Test backward")
        for k in range(1, 11):
            if not np.allclose(W_M[10, k], 0):
                print("k =", k, "W_M[10,k] =", W_M[10, k])

        for k in range(1, 11):
            if not np.allclose(F_M[10, k], 0):
                print("k =", k, "F_M[10,k] =", F_M[10, k])

        for k in range(1, 11):
            if not np.allclose(L_M[10, k], 0):
                print("k =", k, "L_M[10,k] =", L_M[10, k])

        print("Test ")
        print("phi[10] =", phi[10])
        print("psi[10] =", psi[10])

        for k in range(1, 11):
            val_force = F_M[10, k] @ psi[10]
            val_torque = L_M[10, k] @ phi[10]
            val_total = val_force + val_torque
            print(f"k={k}: force_proj={val_force:.6f}, torque_proj={val_torque:.6f}, total={val_total:.6f}")
    
        print("Final M =\n", M)
        print("M[10,1] =", M[10,1])
        print("M[10,2] =", M[10,2])
        print("M[10,3] =", M[10,3])
        print("M[10,7] =", M[10,7])
        print("M[10,8] =", M[10,8])
        print("M[10,9] =", M[10,9])
        print("M[10,10] =", M[10,10])

    return W_c, F_c, L_c, Q, W_M, F_M, L_M, M
   



def  ner_generique(q, qd, qdd, mbs_data,topology, F_ext=None, L_ext=None): 
    """
    Appelle la cinématique et puis la dynamique 
    
    Pour l'instant:
    - M est la matrice de masse généralisée
    - Q est le vecteur généralisé obtenu par la partie convective/gravitée
    """

    omega, omega_c_dot, alpha_c, beta_c, z, O_M, A_M, R, R_abs, p_abs = forward_kinematics(
        q, qd, topology, mbs_data, debug=False
    )

    W_c, F_c, L_c, Q, W_M, F_M, L_M, M = backward_dynamics(
        topology, mbs_data,
        omega, omega_c_dot, alpha_c, beta_c, z, R, O_M, A_M,
        F_ext=F_ext, L_ext=L_ext, debug=False
    )
    
    return M, Q


def compute_qdd_u(q, qd, mbs_data, topology, u, c=None, qdd_c=None,
                  t_current=0.0, motor_state=None,
                  use_carousel_forces=True):
    """
    Berechnet die Beschleunigungen der unabhängigen Koordinaten.

    q, qd:
        volle Robotran-Vektoren mit Indexierung 1..N
    u:
        Indizes der unabhängigen Koordinaten
    c:
        Indizes der kommandierten Koordinaten
    qdd_c:
        Beschleunigungen der kommandierten Koordinaten.
        Für den ersten Schritt None oder 0.
    
    Rückgabe:
        qdd_u : Beschleunigungen der unabhängigen Koordinaten
        Muu   : reduzierter Massematrixblock
        Qu    : reduzierter rechter Vektor vor Korrektur durch qdd_c
    """
    qdd_dummy = np.zeros_like(q)
    #F_ext, L_ext = compute_spring_damper_cartesian_forces(q, qd, mbs_data, topology)
    M, Q = ner_generique(q, qd, qdd_dummy, mbs_data, topology)


    Muu = M[np.ix_(u, u)]
    Qu = Q[u]
    cond_Muu = np.linalg.cond(Muu)

    if cond_Muu > 1e6:
        print("\nWARNING: Muu badly conditioned")
        print("t =", t_current)
        print("cond(Muu) =", cond_Muu)
        print("q[u] =")
        print(q[u])
        print("qd[u] =")
        print(qd[u])
    rhs = Qu.copy()

    # --------------------------------------------------
    # External/generalized torque from the assignment
    # First test: constant motor torque on main pole rotation q1
    # --------------------------------------------------

    Q_ext = np.zeros_like(Q)
    # Motor torque on q1 = main vertical pole rotation
    Q_ext[1] = 0.0     
    #Q_ext[1] = main_motor_torque(t_current, qd[1], motor_state)

    # Viscous damping in arm-pendulum hinges
    # damping torque: T_damp = -d * qd
    if use_carousel_forces:
        Q_ext[1] = main_motor_torque(t_current, qd[1], motor_state)

        Q_ext[4]  += -100.0   * qd[4]
        Q_ext[11] += -100.0   * qd[11]
        Q_ext[14] += -100.0   * qd[14]
        Q_ext[17] += -20000.0 * qd[17]

        for idx in [5, 6, 12, 13, 15, 16, 18, 19]:
            Q_ext[idx] += -6.0 * qd[idx]

    # Linear spring-damper between arm and pendulum, via virtual work
    #Q_spring = compute_spring_damper_Q_virtual_work(q, qd, mbs_data, topology, u)
    #Q_ext += Q_spring

    rhs = rhs + Q_ext[u]


    # --------------------------------------------------
    # Correction for commanded accelerations qdd_c
    # --------------------------------------------------

    if c is not None and qdd_c is not None and len(c) > 0:
        Muc = M[np.ix_(u, c)]
        rhs = rhs - Muc @ qdd_c

    qdd_u = np.linalg.solve(Muu, rhs)
    

    if np.max(np.abs(qdd_u)) > 1e4:
        print("\nWARNING: huge qdd_u")
        print("t =", t_current)
        print("max abs qdd_u =", np.max(np.abs(qdd_u)))
        print("qdd_u =")
        print(qdd_u)
        print("q[u] =")
        print(q[u])
        print("qd[u] =")
        print(qd[u])


    return qdd_u, Muu, Qu


def diagnose_q1_mass_contributions(q, qd, mbs_data, topology, t_label=""):
    """
    Diagnostic: recompute Qu[1] while keeping only one body's mass/inertia active.
    This identifies which body contributes most to Q1.
    """
    qdd_dummy = np.zeros_like(q)

    M_full, Q_full = ner_generique(q, qd, qdd_dummy, mbs_data, topology)
    print("\n--- Q1 mass contribution diagnostic", t_label, "---")
    print("Full Qu[1] =", Q_full[1])

    m_orig = topology["m"].copy()
    I_orig = topology["I"].copy()
    d_orig = topology["d_ii"].copy()

    contributors = []

    for i in range(1, mbs_data.njoint + 1):
        if abs(m_orig[i]) < 1e-12 and np.linalg.norm(I_orig[i]) < 1e-12:
            continue

        topology["m"][:] = 0.0
        topology["I"][:] = 0.0
        topology["d_ii"][:] = 0.0

        topology["m"][i] = m_orig[i]
        topology["I"][i] = I_orig[i]
        topology["d_ii"][i] = d_orig[i]

        _, Q_i = ner_generique(q, qd, qdd_dummy, mbs_data, topology)

        contributors.append((i, Q_i[1]))

    topology["m"][:] = m_orig
    topology["I"][:] = I_orig
    topology["d_ii"][:] = d_orig

    contributors.sort(key=lambda x: abs(x[1]), reverse=True)

    for i, val in contributors:
        print(f"body/joint q{i}: contribution to Q1 = {val}")


def rhs_neri(t, y, mbs_data, topology, u, c=None, motor_state=None):
    """
    Rechte Seite der Differentialgleichung für den RK4-Integrator.
    yd = f(t, y)

    y enthält nur die unabhängigen Koordinaten:
        y = [q_u, qd_u]

    Rückgabe:
        yd = [qd_u, qdd_u]
    """

    nu = len(u)

    q_u = y[:nu]
    qd_u = y[nu:]

    # Volle Robotran-Vektoren rekonstruieren
    q = np.array(mbs_data.q, dtype=float).copy()
    qd = np.array(mbs_data.qd, dtype=float).copy()

    # Unabhängige Koordinaten aus dem Integratorzustand einsetzen
    q[u] = q_u
    qd[u] = qd_u

    # Kommandierte Koordinaten aus der Aufgabenstellung einsetzen
    if c is not None and len(c) > 0:
        #qc, qdc, qddc = commanded_motion(t, c)
        qc = np.zeros(len(c))
        qdc = np.zeros(len(c))
        qddc = np.zeros(len(c))

        q[c] = qc
        qd[c] = qdc
    else:
        qddc = None
    # Beschleunigungen der unabhängigen Koordinaten berechnen
    if 1.6995 < t < 1.7005:
        diagnose_q1_mass_contributions(q, qd, mbs_data, topology, t_label=f"t={t}")
    qdd_u, _, _ = compute_qdd_u(q, qd, mbs_data, topology, u, c, qdd_c=qddc, 
                                t_current=t, motor_state=motor_state, use_carousel_forces=True)

    yd = np.zeros_like(y)

    # y = [q_u, qd_u]
    # yd = [qd_u, qdd_u]
    yd[:nu] = qd_u
    yd[nu:] = qdd_u

    return yd

def rk4_step(fun, t, y, dt, *args):
    """
    Ein einzelner Runge-Kutta-4 Schritt.

    fun muss die Form haben:
        yd = fun(t, y, *args)
    """

    k1 = fun(t, y, *args)
    k2 = fun(t + 0.5 * dt, y + 0.5 * dt * k1, *args)
    k3 = fun(t + 0.5 * dt, y + 0.5 * dt * k2, *args)
    k4 = fun(t + dt, y + dt * k3, *args)

    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

def rk4_integrate(fun, t0, tf, y0, dt, *args):
    """
    Einfache RK4-Zeitintegration.

    Returns
    -------
    t_vec : array, shape (nt,)
    y_vec : array, shape (nt, len(y0))
    """

    n_steps = int(np.floor((tf - t0) / dt)) + 1

    t_vec = np.zeros(n_steps)
    y_vec = np.zeros((n_steps, len(y0)))

    t_vec[0] = t0
    y_vec[0, :] = y0

    t = t0
    y = y0.copy()

    for k in range(1, n_steps):
        y = rk4_step(fun, t, y, dt, *args)
        t = t + dt

        t_vec[k] = t
        y_vec[k, :] = y

        #if np.isnan(y).any():
        #    raise RuntimeError(f"NaN detected during RK4 integration at step {k}, t={t}")
        if np.max(np.abs(y)) > 500.0:
            print("\nState became too large")
            print("step =", k)
            print("t =", t)
            print("max abs y =", np.max(np.abs(y)))

            imax = np.argmax(np.abs(y))
            print("index of max in y =", imax)
            print("value =", y[imax])

            nu = len(y) // 2
            if imax < nu:
                print("This is a q_u coordinate, local index =", imax)
            else:
                print("This is a qd_u coordinate, local index =", imax - nu)

            print("y =")
            print(y)

            raise RuntimeError(f"State too large during RK4 integration at step {k}, t={t}")
        if np.isnan(y).any():
            print("\nNaN detected during RK4 integration")
            print("step =", k)
            print("t =", t)
            print("previous y max abs =", np.max(np.abs(y_vec[k-1, :])))
            print("current y max abs =", np.max(np.abs(y)))
            print("previous y =")
            print(y_vec[k-1, :])
            print("current y =")
            print(y)
            raise RuntimeError(f"NaN detected during RK4 integration at step {k}, t={t}")

    return t_vec, y_vec


def commanded_motion(t, c):
    """
    Driven motions from the assignment.

    q2 = theta1(t): first tilt angle around I1
    q3 = theta2(t): second tilt angle around I2

    Other commanded coordinates are kept at zero for now.
    """

    A = 2.5 * 2.0 * np.pi / 360.0
    omega = 0.4 * np.pi
    phi1 = 0.5 * np.pi
    phi2 = 0.0

    qc = np.zeros(len(c))
    qdc = np.zeros(len(c))
    qddc = np.zeros(len(c))

    for j, idx in enumerate(c):
        if idx == 2:
            phi = phi1
            qc[j] = A * (1.0 - np.cos(omega * t + phi))
            qdc[j] = A * omega * np.sin(omega * t + phi)
            qddc[j] = A * omega**2 * np.cos(omega * t + phi)

        elif idx == 3:
            phi = phi2
            qc[j] = A * (1.0 - np.cos(omega * t + phi))
            qdc[j] = A * omega * np.sin(omega * t + phi)
            qddc[j] = A * omega**2 * np.cos(omega * t + phi)

        else:
            qc[j] = 0.0
            qdc[j] = 0.0
            qddc[j] = 0.0

    return qc, qdc, qddc

"""
def main_motor_torque(t, qd1, motor_state):
    
    #Motor torque from assignment.
    #T = 1000 Nm until qd1 first reaches 0.8 rad/s.
    #Then T = 500 * (1 + cos(2*pi*(t - t0))) for 0.5 s.
    #Then T = 0.
    

    threshold = 0.8

    if motor_state is None:
        return 1000.0

    if motor_state["t0_reach"] is None:
        if qd1 < threshold:
            return 1000.0
        else:
            motor_state["t0_reach"] = t
            print("\nMotor threshold reached")
            print("t0_reach =", t)
            print("qd1 =", qd1)
        
    t0 = motor_state["t0_reach"]

    if t < t0 + 0.5:
        return 500.0 * (1.0 + np.cos(2.0 * np.pi * (t - t0)))

    return 0.0
"""

def main_motor_torque(t, qd1, motor_state):
    threshold = 0.8

    if motor_state is None:
        torque = 1000.0
        return torque

    if motor_state["t0_reach"] is None:
        if qd1 < threshold:
            torque = 1000.0
        else:
            motor_state["t0_reach"] = t
            print("\nMotor threshold reached")
            print("t0_reach =", t)
            print("qd1 =", qd1)
            torque = 0.0
    else:
        torque = 0.0

    # print only every ~0.1 s
    if abs((t * 10) - round(t * 10)) < 1e-6:
        print("motor debug: t =", round(t, 3), "qd1 =", qd1, "T =", torque)

    return torque


def define_cartpendulum_reference_topology(mbs_data):
    N = mbs_data.njoint

    joint_names = ["base"] + [None] * N

    j_cart = mbs_data.joint_id["Cart_T2"]
    j_pend = mbs_data.joint_id["Pendulum_R1"]

    joint_names[j_cart] = "Cart_T2"
    joint_names[j_pend] = "Pendulum_R1"

    if N >= 3:
        joint_names[3] = "Joint_2"

    inbody = np.empty(N + 1, dtype=object)
    phi = np.zeros((N + 1, 3))
    psi = np.zeros((N + 1, 3))
    d_hi = np.zeros((N + 1, 3))
    m = np.zeros(N + 1)
    d_ii = np.zeros((N + 1, 3))
    I = np.zeros((N + 1, 3, 3))

    inbody[0] = None

    inbody[j_cart] = 0
    psi[j_cart] = np.array([0.0, 1.0, 0.0])

    inbody[j_pend] = j_cart
    phi[j_pend] = np.array([1.0, 0.0, 0.0])
    d_hi[j_pend] = np.zeros(3)

    if N >= 3:
        inbody[3] = j_pend
        phi[3] = np.array([1.0, 0.0, 0.0])
        d_hi[3] = np.zeros(3)
        m[3] = 0.0
        d_ii[3] = np.zeros(3)
        I[3] = np.zeros((3, 3))

    m_cart = mbs_data.m[mbs_data.body_id["Cart"]]
    m_pend = mbs_data.m[mbs_data.body_id["Pendulum_Mass"]]

    Lp_id = mbs_data.points_id["Pendulum_String"]["Lp"]
    Lp = mbs_data.dpt[3, Lp_id]

    m[j_cart] = m_cart
    d_ii[j_cart] = np.zeros(3)

    m[j_pend] = m_pend
    d_ii[j_pend] = np.array([0.0, 0.0, -Lp / 2.0])
    I[j_pend][0, 0] = m_pend * Lp**2 / 12.0

    return {
        "joint_names": joint_names,
        "inbody": inbody,
        "phi": phi,
        "psi": psi,
        "d_hi": d_hi,
        "m": m,
        "d_ii": d_ii,
        "I": I,
    }

