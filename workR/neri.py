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
    np.array([None, None, None]),  # 0: base
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
    np.array([None, None, None]),                           # 0: base
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
    np.array([None, None, None]),                           # 0: base
    np.array([0, 0, 0]),            # 1: R3_Pole (no translation)
    np.array([0, 0, 0]),            # 2: R1_Pole (no translation)
    np.array([1, 0, 4.5]),          # 3: R2_Pole (no translation)
    np.array([0, 0, -3]),           # 4: R2_arm_pend1 (no translation)
    np.array([0, 0, 0]),            # 5: R2_cardan1 (no translation)
    np.array([0, 0, -1]),           # 6: R1_cardan1 (no translation)
    np.array([0, -1, 0]),           # 7: R2_arm_pend2 (T1 translation along X)
    np.array([0, 0, 0]),            # 8: T1_effort_normal (translation along X)
    np.array([0, 0, 0]),            # 9: T3_effort_tranchant (translation along Z)
    np.array([0, 0, 0]),            # 10: R2_effort_flechissant (no translation)
    np.array([0, 0, -3]),           # 11: arm_pend4 (no translation)
    np.array([0, 0, 0]),            # 12: R_cardan4a (no translation)
    np.array([0, 0, -1]),           # 13: R_cardan4b (no translation)
    np.array([0, 0, -3]),           # 14: arm_pend3 (no translation)
    np.array([0, 0, 0]),            # 15: R_cardan3a (no translation)
    np.array([0, 0, -1]),           # 16: R_cardan3b (no translation)
    np.array([0, 0, -3]),           # 17: arm_pend2 (no translation)
    np.array([0, 0, 0]),            # 18: R_cardan2a (no translation)
    np.array([0, 0, -1])            # 19: R_cardan2b (no translation)
    ]

    z_list = [
    np.array([None, None, None]),                           # 0: base
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
    0.0,                            # 1: R3_Pole (mass of Pole)
    0.0,                            # 2: R1_Pole
    600.0,                          # 3: R2_Pole
    44.0,                           # 4: R2_arm_pend1 (mass of Pendule1)
    0.0,                            # 5: R2_cardan1 (mass of nacelle1)
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
    123.3                           # 19: R_cardan2b
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
    np.array([0.0, 0.0, -1.0])     # 19: R_cardan2b
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
    np.diag([15.4, 15.4, 61.7])     # 19: R_cardan2b
    ]

    id_ext_effort = np.array([6, 13, 16, 19]) #Id des corps ayant une force/couple externe 
    id_int_effort = np.array([]) #Id des articulations avec des effort interne 
    id_dependant_variable = np.array([]) 
    id_driven_varialble = np.array([2, 3, 8, 9, 10])



    topology = {
        # Topologie
        "inbody": np.array(inbody_list),             # Taille: (n+1,)
        "phi":    np.array(phi_list),                # Taille: (n+1, 3) --> phi[i] = [x,y,z]
        "psi":    np.array(psi_list),                # Taille: (n+1, 3) --> psi[i] = [x,y,z]
        "d_hi":   np.array(d_hi_list),               # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]
        "z":      np.array(z_list),                  # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]

        # Dynamique
        "m":      np.array(m_list),                  # Taille: (n+1,)     --> m[i] = scalaire
        "d_ii":   np.array(d_ii_list),               # Taille: (n+1, 3) --> d_ii[i] = [x,y,z]
        "I":      np.array(I_list),                   # Taille: (n+1, 3, 3)--> I[i] = matrice 3x3

        # id_corp particulier
        "id_ext_effort": id_ext_effort,
        "id_int_effort": id_int_effort,
        "id_dependant": id_dependant_variable,
        "id_driven": id_driven_varialble
    }
    
    return topology

def check_topology_lengths(topology):
    """
    Vérifie si toutes les listes/tableaux de la topologie ont la même taille.
    
    Args:
        topology (dict): Le dictionnaire contenant la topologie du robot.
        
    Returns:
        bool: True si toutes les tailles sont identiques, False sinon.
    """
    # Dictionnaire pour stocker la taille de chaque clé
    lengths = {}
    
    for key, value in topology.items():
        # Comme ce sont des arrays numpy, len() ou .shape[0] donne la taille de la première dimension
        lengths[key] = len(value) 
        
    # On convertit les valeurs en 'set' pour ne garder que les tailles uniques
    unique_lengths = set(lengths.values())
    
    # S'il n'y a qu'une seule taille unique, c'est que toutes les listes ont la même taille
    if len(unique_lengths) == 1:
        taille = list(unique_lengths)[0]
        print(f"Succès : Toutes les listes de la topologie ont la même taille ({taille} éléments).")
        return True
    else:
        print("Erreur : Les listes de la topologie n'ont pas toutes la même taille !")
        # Affichage du détail pour identifier facilement l'erreur
        for key, length in lengths.items():
            print(f" - '{key}' : {length} éléments")
        return False



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
    
        
    c = np.cos(q)
    s = np.sin(q)

    if phi is None or np.array_equal(phi, np.zeros(3)): 
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

def forward_kinematics(q, qd, qdd, topology, gravity, mbs_data: Robotran.MbsData): 
    """
        Etape 1: Parcourt l'arbre de la base vers les feuilles
        Calcule les vitesses et accélérations dans chaque corps. 
    """

    N_body = mbs_data.njoint 

    inbody = topology["inbody"]
    phi    = topology["phi"]
    psi    = topology["psi"]
    d_hi   = topology["d_hi"]
    z      = topology["z"]

    #Allocations mémoires (taille n+1 pour inclure la base à l'index 0)
    omega      = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    omega_dot = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    alpha     = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    beta      = np.zeros((N_body + 1, 3, 3)) #Liste de matrice 3x3

    #Pour si jamais on appelle une mauvaise matrice alors c'est l'identité I
    R = np.empty((N_body + 1, N_body + 1, 3, 3))

    for i in range(N_body + 1):
        for j in range(N_body + 1):
            R[i, j] = np.eye(3)

    #Condition initial

    alpha[0] = - gravity

    #les autres conditions initial mis à 0 sont fait par le np.zeros

    #Note: Attention q[i] pour la matrice de rotation si c'est une translation R = I

    for i in range(1, N_body + 1): 
        h = inbody[i]
        R_ih = rotation_matrix(phi[i], psi[i], q[i])
        R[i][h] = R_ih
        

        omega[i] = R_ih @ omega[h] + phi[i] * qd[i]
        omega_i_tilted = tilde(omega[i])

        omega_dot[i] = R_ih @ omega_dot[h] + omega_i_tilted @ phi[i] * qd[i] + phi[i] * qdd[i]
        omega_i_c_dot_tilted = tilde(omega_dot[i])

        beta[i] = omega_i_c_dot_tilted + omega_i_tilted @ omega_i_tilted
        alpha[i] = R_ih @ (alpha[h] + beta[h] @ (z[h] + d_hi[i])) + 2 * omega_i_tilted @ psi[i] * qd[i] + psi[i] * qdd[i]

    return omega, omega_dot, alpha, beta, R

def backward_dynamics(omega, omega_dot, alpha, beta, R, F_ext, L_ext, topology, mbs_data: Robotran.MbsData): 
    """
    Etape 2: Parcourt de l'arbre des feuilles vers la base 
    Calcule les forces et couples aux articulations, et projette pour obtenir Q. 
    
    """

    N_body  = mbs_data.njoint

  #  F_ext = np.zeros((N_body + 1, 3))   #Doit être une variable en entrée normalement 
  #  L_ext = np.zeros((N_body + 1, 3))   #Doit être une variable en entrée normalement 


    z       = topology["z"]
    m       = topology["m"]
    d_ii    = topology["d_ii"]
    d_ij    = topology["d_ij"]
    I       = topology["I"] 
    psi     = topology["psi"]
    phi     = topology["phi"]
    inbody  = topology["inbody"]


    #Allocation de la mémoire 
    W = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    F = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    L = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1

    Q = np.zeros((N_body + 1, 3))

    sum_F_children = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1
    sum_L_children = np.zeros((N_body + 1, 3)) #liste de vecteur 3x1

    for i in range(N_body, 0, -1):  # La boucle s'arrète à i = 1 et on parcourt à l'envers (de la feuille vers la base)
        W[i] = m[i] * (alpha[i] + beta[i] @ (z[i] + d_ii[i])) - F_ext[i]
        F[i] = sum_F_children[i] + W[i]
        L[i] = sum_L_children[i] + tilde(z[i] + d_ii[i]) @ W[i] - L_ext[i] + I[i] @ omega_dot[i] + tilde(omega[i]) @ I[i] @ omega[i]


        h = inbody[i]
        if h > 0: # On s'assure que le parent n'est pas la base (indice 0)
            # L'enfant 'i' pousse sa force vers le parent 'h'
            R_hi = R[i][h].T #transposer car dans l'autre sens
            sum_F_children[h] += R_hi @ F[i]
            
            # L'enfant 'i' pousse son moment vers le parent 'h' en incluant le bras de levier
            # Attention : d_ij[i] doit être le vecteur allant du parent h vers l'articulation de l'enfant i
            sum_L_children[h] += R_hi @ L[i] + tilde(z[i] + d_ij[i]) @ R_hi @ F[i]
        
        Q[i] = np.dot(F[i], psi[i]) + np.dot(L[i], phi[i])

    return Q

def  ner_generique(q, qd, qdd, F_ext, L_ext, topology, gravity, mbs_data: Robotran.MbsData): 
    """
    Appelle la cinématique et puis la dynamique 
    Renvoie le vecteur des efforts articulaires Q pour un état cinématique donné. 
    """

    omega, omega_dot, alpha, beta, R = forward_kinematics(q, qd, qdd, topology, gravity, mbs_data)
    Q = backward_dynamics(omega, omega_dot, alpha, beta, R,  F_ext, L_ext, topology, mbs_data)
    return Q

######################################################
####             Compute M and c                ######
######################################################

def compute_M(q, topology, mbs_data: Robotran.MbsData ): 
    """
    Calcule la matrice de Masse M(q)
    Tricks: on annule la gravité, les vitesse et on évalue la dynamique inverse 
    en mettant un '1' successivement sur chaque degré de liberté de qdd
    
    """
    N_body = mbs_data.njoint
    M = np.zeros((N_body + 1, N_body + 1))

    qd_zero = np.zeros(N_body + 1)

    #Toutes les forces à 0
    gravity_zero = np.zeros(3)
    F_ext_zero   = np.zeros((N_body + 1, 3))
    L_ext_zero   = np.zeros((N_body + 1, 3))

    for j in range(1, N_body + 1):

        qdd_unit = np.zeros(N_body + 1)
        qdd_unit[j] = 1.0; 
    
        Q_col = ner_generique(q, qd_zero, qdd_unit, F_ext_zero, L_ext_zero, topology, gravity_zero, mbs_data)

        M[:, j] = Q_col

    return M 

def compute_c(q, qd, F_ext, L_ext, topology, mbs_data: Robotran.MbsData):
    """
    Calcule le vecteur des termes de Coriolis, centrifuge et de gravité
    Tricks: On mets les accélérations (qdd) à 0
    
    """

    N_body = mbs_data.njoint

    qdd_zero = np.zeros(N_body + 1)
    gravity = mbs_data.g[1:4]

    c = ner_generique(q, qd, qdd_zero, F_ext, L_ext, topology, gravity, mbs_data)
    
    return c


######################################################
####            Partitioning                    ######
######################################################

#################################################################
####  compute force (exernal and joint) + Driven mouvement  #####
#################################################################

def compute_external_effort(t, q, topology, mbs_data: Robotran.MbsData):

    N_body = mbs_data.njoint
    id_ext_effort = topology["id_ext_effort"]

    F_ext = np.zeros((N_body + 1, 3))
    L_ext = np.zeros((N_body + 1, 3))


    #Calcule de la force du vent sur les nacelles (elle est appliquer selon x et sur le centre de masse)
    A = 300.0 
    omega = 6 * np.pi 
    F_wind_magnitude = A * (1 - np.cos(omega * t))

    F_wind_inertial = np.array([F_wind_magnitude, 0.0, 0.0])


    for i in id_ext_effort: 
        F_ext[i] = F_wind_inertial  #Une rotation à prendre en compte 
        L_ext[i] = np.zerros(3)  #La force est appliqué sur le centre de masse 

    return F_ext, L_ext

def compute_internal_effort(t, q, topology, mbs_data: Robotran.MbsData):
    N_body = mbs_data.njoint
    Q = np.zeros((N_body + 1, 3))
    return Q

def compute_driven_motion(t, q, qd, qdd, topology, mbs_data: Robotran.MbsData): 
    id_driven = topology["id_driven_varialble"]  #[2, 3, 8, 9, 10] => 2, 3 sont les moteurs du bras et 8, 9, 10 sont les efforts dans la poutres

    id_R1_pole = id_driven[0]
    id_R2_pole = id_driven[1]

    id_T1_effort_normal = id_driven[2]
    id_T3_effort_tranchant = id_driven[3]
    id_R2_effort_flechissant = id_driven[4]

    #Mouvement des moteurs du pole
    A = (2.5*2*np.pi)/360  # amplitude of the oscillation (rad)
    w = 0.4*np.pi  # pulsation of the oscillation (rad/s)
    phi1 = np.pi/2  # phase of the oscillation (rad)
    phi2 = 0  # phase of the oscillation (rad)

    q[id_R1_pole] = A*(1 - np.cos(w*t + phi1))
    qd[id_R1_pole] = A*w*np.sin(w*t + phi1)
    qdd[id_R1_pole] = A*w*w*np.cos(w*t + phi1)

    q[id_R2_pole] = A*(1 - np.cos(w*t + phi2))
    qd[id_R2_pole] = A*w*np.sin(w*t + phi2)
    qdd[id_R2_pole] = A*w*w*np.cos(w*t + phi2)


    #Effort dans la poutre
    q[id_T1_effort_normal] = 0.0
    qd[id_T1_effort_normal] = 0.0
    qdd[id_T1_effort_normal] = 0.0

    q[id_T3_effort_tranchant] = 0.0
    qd[id_T3_effort_tranchant] = 0.0
    qdd[id_T3_effort_tranchant] = 0.0

    q[id_R2_effort_flechissant] = 0.0
    qd[id_R2_effort_flechissant] = 0.0
    qdd[id_R2_effort_flechissant] = 0.0



    return q, qd, qdd


######################################################
####             compute derivative             ######
######################################################

def compute_derivatives(t, y, topology, mbs_data: Robotran.MbsData):
    """
    Compute the derivatives yd for a given state y of the system.
    The derivatives are computed at the given time t with
    the parameters values in the given data structure.
    
    It is assumed that the state vector y contains the following states:
      y = [q1, q2, qd1, qd2] with:
         - q1: the cart position
         - q2: the pendulum angular position 
         - qd1: the cart velocity
         - qd2: the pendulum angular velocity 

    :param  t: the time instant when to compute the derivatives.
    :param  y: the numpy array containing the states 
    :return: yd a numpy array containing the states derivatives  yd = [qd1, qd2,qd ...,  qdd1, qdd2, qdd ...]
    :param mbs_data: the mbs_data object containing the parameters of the model
    """
    N_body = mbs_data.njoint
    q_active = y[:N_body]
    qd_active = y[N_body:]

    q = np.zeros(N_body + 1)
    qd = np.zeros(N_body + 1)
    q[1:] = q_active
    qd[1:] = qd_active

    #Calcule des variables commandée

    q, qd, qdd = compute_driven_motion(t, q, qd, qdd, topology, mbs_data)

    F_ext, L_ext = compute_external_effort(t, q, topology, mbs_data)
    

    # Calcul des matrices génériques M et c via tes fonctions !
    M_full = compute_M(q, topology, mbs_data)
    c_full = compute_c(q, qd, F_ext, L_ext, topology, mbs_data)

    # On extrait les sous-matrices actives (on ignore la ligne/colonne 0 de la base)
    M_active = M_full[1:, 1:]
    c_active = c_full[1:]

    #Ajout des forces/couples dans les articulations 
    Q_active = np.zeros(N_body)

    Q = compute_internal_effort(t, q, topology, mbs_data)

    #Partitionnement entre les variables (indépendante et driven)

    #Resolution du systeme 
    RHS = Q_active - c_active
    qdd_active = np.linalg.solve(M_active, RHS)

    #Construction du vecteur dérivé yd = [qd, qdd]
    yd = np.zeros(2 * N_body)
    yd[:N_body] = qd_active
    yd[N_body:] = qdd_active

    return yd