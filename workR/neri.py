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
    inbody_list = np.array([None, 0, 1, 2, 3, 4, 5, 3,  ])
    phi_list    = np.array([None, 
                           [],
                           ])
    psi_list    = np.array([None, 
                         ])
    d_hi_list   = np.array([None,
                          ])
    z_list      = np.array([None,
                       ])





    topology = {
        # Topologie
        "inbody": np.array(inbody_list),             # Taille: (n+1,)
        "phi":    np.array(phi_list),                # Taille: (n+1, 3) --> phi[i] = [x,y,z]
        "psi":    np.array(psi_list),                # Taille: (n+1, 3) --> psi[i] = [x,y,z]
        "d_hi":   np.array(d_hi_list),               # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]
        # Dynamique
        "m":      np.array(m_list),                  # Taille: (n+1,)     --> m[i] = scalaire
        "l_ci":   np.array(l_ci_list),               # Taille: (n+1, 3) --> l_ci[i] = [x,y,z]
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

    