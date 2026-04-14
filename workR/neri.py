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
    None,                           # 0: base
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
    np.array([0, 1, 0]),           # 19: R_cardan2b 
    ]


    psi_list = [
    None,                           # 0: base
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
    np.array([0, 0, 0]),           # 19: R_cardan2b (no translation)
    ]

    d_hi_list = [
    None,                           # 0: base
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
    np.array([0, 0, -1]),           # 19: R_cardan2b (no translation)
    ]

    z_list = [
    None,                           # 0: base
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
    np.array([0, 0, 0]),           # 19: R_cardan2b (no translation)
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
    123.3,                          # 12: R_cardan4a (mass of nacelle4)
    123.3,                          # 13: R_cardan4b
    44.0,                           # 14: arm_pend3 (mass of Pendule3)
    123.3,                          # 15: R_cardan3a (mass of nacelle3)
    123.3,                          # 16: R_cardan3b
    44.0,                           # 17: arm_pend2 (mass of Pendule2)
    123.3,                          # 18: R_cardan2a (mass of nacelle2)
    123.3,                          # 19: R_cardan2b
    ]


    d_ii_list = [
    np.array([0.0, 0.0, 0.0]),     # 0: base
    np.array([0.0, 0.0, 2.0]),     # 1: R3_Pole (COM of Pole)
    np.array([0.0, 0.0, 2.0]),     # 2: R1_Pole
    np.array([0.0, 0.0, 2.0]),     # 3: R2_Pole
    np.array([0.0, 0.0, -1.5]),    # 4: R2_arm_pend1 (COM of Pendule1)
    np.array([0.0, 0.0, -1.0]),    # 5: R2_cardan1 (COM of nacelle1)
    np.array([0.0, 0.0, -1.0]),    # 6: R1_cardan1
    np.array([-0.5, 0.0, 0.0]),    # 7: R2_arm_pend2 (COM of arm_part1)
    np.array([0.0, 0.0, 0.0]),     # 8: T1_effort_normal (COM of arm_part2)
    np.array([0.0, 0.0, 0.0]),     # 9: T3_effort_tranchant
    np.array([0.0, 0.0, 0.0]),     # 10: R2_effort_flechissant
    np.array([0.0, 0.0, -1.5]),    # 11: arm_pend4 (COM of Pendule4)
    np.array([0.0, 0.0, -1.0]),    # 12: R_cardan4a (COM of nacelle4)
    np.array([0.0, 0.0, -1.0]),    # 13: R_cardan4b
    np.array([0.0, 0.0, -1.5]),    # 14: arm_pend3 (COM of Pendule3)
    np.array([0.0, 0.0, -1.0]),    # 15: R_cardan3a (COM of nacelle3)
    np.array([0.0, 0.0, -1.0]),    # 16: R_cardan3b
    np.array([0.0, 0.0, -1.5]),    # 17: arm_pend2 (COM of Pendule2)
    np.array([0.0, 0.0, -1.0]),    # 18: R_cardan2a (COM of nacelle2)
    np.array([0.0, 0.0, -1.0]),    # 19: R_cardan2b
    ]


    I_list = [
    np.zeros((3, 3)),               # 0: base
    np.diag([458.8, 458.8, 16.0]),  # 1: R3_Pole (Inertia of Pole)
    np.diag([458.8, 458.8, 16.0]),  # 2: R1_Pole
    np.diag([458.8, 458.8, 16.0]),  # 3: R2_Pole
    np.diag([15.9, 15.9, 0.091]),   # 4: R2_arm_pend1 (Inertia of Pendule1)
    np.diag([15.4, 15.4, 61.7]),    # 5: R2_cardan1 (Inertia of nacelle1)
    np.diag([15.4, 15.4, 61.7]),    # 6: R1_cardan1
    np.diag([1.0, 1.0, 1.0]),       # 7: R2_arm_pend2 (Inertia of arm_part1)
    np.zeros((3, 3)),               # 8: T1_effort_normal (Inertia of arm_part2)
    np.zeros((3, 3)),               # 9: T3_effort_tranchant
    np.zeros((3, 3)),               # 10: R2_effort_flechissant
    np.diag([15.9, 15.9, 0.091]),   # 11: arm_pend4 (Inertia of Pendule4)
    np.diag([15.4, 15.4, 61.7]),    # 12: R_cardan4a (Inertia of nacelle4)
    np.diag([15.4, 15.4, 61.7]),    # 13: R_cardan4b
    np.diag([15.9, 15.9, 0.091]),   # 14: arm_pend3 (Inertia of Pendule3)
    np.diag([15.4, 15.4, 61.7]),    # 15: R_cardan3a (Inertia of nacelle3)
    np.diag([15.4, 15.4, 61.7]),    # 16: R_cardan3b
    np.diag([15.9, 15.9, 0.091]),   # 17: arm_pend2 (Inertia of Pendule2)
    np.diag([15.4, 15.4, 61.7]),    # 18: R_cardan2a (Inertia of nacelle2)
    np.diag([15.4, 15.4, 61.7]),    # 19: R_cardan2b
    ]





    topology = {
        # Topologie
        "inbody": np.array(inbody_list),             # Taille: (n+1,)
        "phi":    np.array(phi_list),                # Taille: (n+1, 3) --> phi[i] = [x,y,z]
        "psi":    np.array(psi_list),                # Taille: (n+1, 3) --> psi[i] = [x,y,z]
        "d_hi":   np.array(d_hi_list),               # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]
        # Dynamique
        "m":      np.array(m_list),                  # Taille: (n+1,)     --> m[i] = scalaire
        "d_ii":   np.array(d_ii_list),               # Taille: (n+1, 3) --> d_ii[i] = [x,y,z]
        "I":      np.array(I_list)                   # Taille: (n+1, 3, 3)--> I[i] = matrice 3x3
    }
    
    return topology


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


def rotation_matrix(axis, angle):
    """
    Calcule la matrice de rotation autour d'un axe local (x, y ou z).
    """
    if np.allclose(axis, 0): # Si c'est une translation, pas de rotation
        return np.eye(3)
        
    c = np.cos(angle)
    s = np.sin(angle)
    
    if axis[0] == 1.0:   # Autour de X
        return np.array([[1, 0, 0], 
                         [0, c, -s], 
                         [0, s, c]])
    elif axis[1] == 1.0: # Autour de Y
        return np.array([[c, 0, s], 
                         [0, 1, 0], 
                         [-s, 0, c]])
    elif axis[2] == 1.0: # Autour de Z
        return np.array([[c, -s, 0], 
                         [s, c, 0], 
                         [0, 0, 1]])
    else:
        return np.eye(3)

######################################################
####             NERi Formalism                 ######
######################################################

def forward_kinematics(q, qd, topology, mbs_data: Robotran.MbsData): 
    """
        Etape 1: Parcourt l'arbre de la base vers les feuilles
        Calcule les vitesses et accélérations dans chaque corps. 
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
    beta_c      = np.zeros((N_body + 1, 3))
    R           = np.zeros((N_body + 1, 3, 3)) # Matrices de rotation R^{i,h}

    O_M = np.zeros((N_body + 1, N_body + 1, 3))
    A_M = np.zeros((N_body + 1, N_body + 1, 3))

    for i in range(1, N_body): 
        h = inbody[i]



    return omega, omega_c_dot, alpha_c, beta_c, O_M, A_M, R


def backward_dynamics(q, qd, qdd, mbs_data, omega, omega_d, alpha): 
    """
    Etape 2: Parcourt de l'arbre des feuilles vers la base 
    Calcule les forces et couples aux articulations, et projette pour obtenir Q. 
    
    """
    Q = 0.0
    return Q

def  ner_generique(q, qd, qdd, mbs_data): 
    """
    Appelle la cinématique et puis la dynamique 
    Renvoie le vecteur des efforts articulaires Q pour un état cinématique donné. 
    """

    omega, omega_d, alpha = forward_kinematics(q, qd, qdd, mbs_data)
    Q = backward_dynamics(q, qd, mbs_data, omega, omega_d, alpha)
    return Q

    