import numpy as np 
import MBsysPy as Robotran
import xml.etree.ElementTree as ET


######################################################
####              Extracte topology             ######
######################################################

def extract_topology_from_mbs(filepath):
    """
    Lit un fichier .mbs de Robotran et extrait automatiquement
    les matrices topologiques nécessaires au formalisme NER.
    """
    # Chargement et parsing du fichier XML
    tree = ET.parse(filepath)
    root = tree.getroot()
    
    # -------------------------------------------------------------
    # INITIALISATION (Pour la base du système = index 0)
    # -------------------------------------------------------------
    # Dictionnaire pour retrouver l'index (0, 1, 2...) d'un corps à partir de son nom
    body_indices = {'base': 0}
    
    # Dictionnaire pour stocker les coordonnées [x, y, z] des points d'ancrage.
    # Clé : (nom_du_corps, nom_du_point)
    points_ancrage = {('base', 'origin'): [0.0, 0.0, 0.0]}
    
    # Listes qui deviendront nos tableaux numpy à la fin
    inbody_list = [0]
    phi_list    = [[0.0, 0.0, 0.0]]
    psi_list    = [[0.0, 0.0, 0.0]]
    d_hi_list   = [[0.0, 0.0, 0.0]]
    
    # -------------------------------------------------------------
    # LECTURE DE L'ARBRE DES CORPS
    # -------------------------------------------------------------
    bodytree = root.find('bodytree')
    if bodytree is None:
        raise ValueError("Erreur : Balise <bodytree> introuvable dans le fichier MBS.")
        
    # On parcourt chaque balise <body>. 'enumerate(..., start=1)' assigne l'index i (1, 2, 3...)
    for i, body in enumerate(bodytree.findall('body'), start=1):
        
        # 1. Nom et indexation du corps
        bodyname = body.find('bodyname').text
        body_indices[bodyname] = i
        
        # Par défaut, chaque corps possède une "origin" en [0, 0, 0] dans son repère local
        points_ancrage[(bodyname, 'origin')] = [0.0, 0.0, 0.0]
        
        # 2. Sauvegarde des points d'ancrage spécifiques du corps (ex: "Lp")
        for pt in body.findall('point'):
            pt_name = pt.find('pointname').text
            coords = pt.find('coordinates')
            x = float(coords.find('x').text)
            y = float(coords.find('y').text)
            z = float(coords.find('z').text)
            points_ancrage[(bodyname, pt_name)] = [x, y, z]
            
        # 3. Lien de parenté (pour 'inbody' et 'd_hi')
        parent = body.find('parent')
        parent_name = parent.find('bodyname').text
        parent_point = parent.find('pointname').text
        
        # On ajoute l'index du parent
        inbody_list.append(body_indices[parent_name])
        # On va chercher où ce point parent était situé pour avoir le vecteur d_hi
        d_hi_list.append(points_ancrage[(parent_name, parent_point)])
        
        # 4. Axes de l'articulation (pour 'phi' et 'psi')
        joint = body.find('joint')
        jtype = joint.find('type').text
        
        phi = [0.0, 0.0, 0.0]
        psi = [0.0, 0.0, 0.0]
        
        # Décodage de la nomenclature classique Robotran
        if jtype == 'T1':   psi = [1.0, 0.0, 0.0]
        elif jtype == 'T2': psi = [0.0, 1.0, 0.0]
        elif jtype == 'T3': psi = [0.0, 0.0, 1.0]
        elif jtype == 'R1': phi = [1.0, 0.0, 0.0]
        elif jtype == 'R2': phi = [0.0, 1.0, 0.0]
        elif jtype == 'R3': phi = [0.0, 0.0, 1.0]
        else:
            print(f"Attention: Type d'articulation '{jtype}' non géré pour {bodyname}.")
            
        phi_list.append(phi)
        psi_list.append(psi)
        
    # -------------------------------------------------------------
    # FORMATAGE FINAL (Conversion en tableaux Numpy)
    # -------------------------------------------------------------
    topologie = {
        "inbody": np.array(inbody_list),
        "phi":    np.array(phi_list).T,  # JE VIENS DE LE RETIRER .T pour transposer et avoir la taille (3, n+1)
        "psi":    np.array(psi_list).T,
        "d_hi":   np.array(d_hi_list).T
    }
    
    return topologie


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
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis[1] == 1.0: # Autour de Y
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis[2] == 1.0: # Autour de Z
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        return np.eye(3)

######################################################
####             NERi Formalism                 ######
######################################################

def forward_kinematics(q, qd, qdd, topology, mbs_data: Robotran.MbsData): 
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
    omega       = np.zeros((3, N_body + 1))
    omega_c_dot = np.zeros((3, N_body + 1))
    alpha_c     = np.zeros((3, N_body + 1))
    beta_c      = np.zeros((3, 3, N_body + 1))
    R           = np.zeros((3, 3, N_body + 1)) # Matrices de rotation R^{i,h}

    O_M = np.zeros((3, N_body + 1, N_body + 1))
    A_M = np.zeros((3, N_body + 1, N_body + 1))

    #Condition initiale 
    alpha_c[:, 0] = -mbs_data.g[1:4]  
    R[:, :, 0]    = np.eye(3)

    for i in range(N_body):  #Boucle sur tout les corps/joints
        h_id =  inbody(i)  #Prend l'indice du corps parents
        R[:, :, i] = rotation_matrix(phi[:, i], q[i]).T 
        R_ih = R[:, :, i]
        
        phi_i = phi[:, i]
        psi_i = psi[:, i]
        
        # --- TIROIR "c" : Termes liés aux vitesses et à la gravité ---
        
        # Vitesse angulaire (Eq 3.58)
        omega[:, i] = R_ih @ omega[:, h] + phi_i * qd[i]
        
        # Accélération angulaire sans les qdd (Eq 3.59)
        omega_c_dot[:, i] = R_ih @ omega_c_dot[:, h] + tilde(omega[:, i]) @ (phi_i * qd[i])
        
        # Terme quadratique (Eq 3.60)
        beta_c[:, :, i] = tilde(omega_c_dot[:, i]) + tilde(omega[:, i]) @ tilde(omega[:, i])
        
        # Accélération linéaire sans les qdd (Eq 3.61)
        terme_parent_alpha = alpha_c[:, h] + beta_c[:, :, h] @ d_hi[:, i]
        alpha_c[:, i] = R_ih @ terme_parent_alpha + 2.0 * tilde(omega[:, i]) @ (psi_i * qd[i])







    
    return omega, omega_d, alpha


def backward_dynamics(q, qd, qdd, mbs_data, omega, omega_d, alpha): 
    """
    Etape 2: Parcourt de l'arbre des feuilles vers la base 
    Calcule les forces et couples aux articulations, et projette pour obtenir Q. 
    
    """
    return Q

def  ner_generique(q, qd, qdd, mbs_data): 
    """
    Appelle la cinématique et puis la dynamique 
    Renvoie le vecteur des efforts articulaires Q pour un état cinématique donné. 
    """

    omega, omega_d, alpha = forward_kinematics(q, qd, qdd, mbs_data)
    Q = backward_dynamics(q, qd, mbs_data, omega, omega_d, alpha)
    return Q

    