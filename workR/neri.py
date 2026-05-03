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





    topology = {
        # Topologie
        "inbody": np.array(inbody_list, dtype=object),             # Taille: (n+1,)
        "phi":    np.array(phi_list, dtype=object),                # Taille: (n+1, 3) --> phi[i] = [x,y,z]
        "psi":    np.array(psi_list, dtype=object),                # Taille: (n+1, 3) --> psi[i] = [x,y,z]
        "d_hi":   np.array(d_hi_list, dtype=object),               # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]
        "z":      np.array(z_list, dtype=object),                  # Taille: (n+1, 3) --> d_hi[i] = [x,y,z]

        # Dynamique
        "m":      np.array(m_list, dtype=object),                  # Taille: (n+1,)     --> m[i] = scalaire
        "d_ii":   np.array(d_ii_list, dtype=object),               # Taille: (n+1, 3) --> d_ii[i] = [x,y,z]
        "I":      np.array(I_list, dtype=object)                   # Taille: (n+1, 3, 3)--> I[i] = matrice 3x3
    }
    
    return topology


def _xml_float(node, default=0.0):
    if node is None or node.text is None:
        return default
    return float(node.text)


def _xml_vec3(node):
    if node is None:
        return np.zeros(3)

    coords = node.find("coordinates")
    if coords is None:
        coords = node

    return np.array([
        _xml_float(coords.find("x")),
        _xml_float(coords.find("y")),
        _xml_float(coords.find("z")),
    ], dtype=float)


def _joint_axis(joint_type):
    """
    Robotran joint type to NERi phi/psi.
    R1,R2,R3 = rotations around x,y,z.
    T1,T2,T3 = translations along x,y,z.
    """
    phi = np.zeros(3)
    psi = np.zeros(3)

    if joint_type == "R1":
        phi[0] = 1.0
    elif joint_type == "R2":
        phi[1] = 1.0
    elif joint_type == "R3":
        phi[2] = 1.0
    elif joint_type == "T1":
        psi[0] = 1.0
    elif joint_type == "T2":
        psi[1] = 1.0
    elif joint_type == "T3":
        psi[2] = 1.0

    return phi, psi


def define_topology_from_mbs(mbs_path):
    """
    Build topology automatically from the Robotran .mbs XML file.

    Convention:
    - Each Robotran joint gets one NERi index.
    - If a body has several joints, the body mass/inertia is attached to the last joint.
    - Intermediate joints of the same body are massless.
    """

    tree = ET.parse(mbs_path)
    root = tree.getroot()

    bodies = root.find("bodytree").findall("body")

    # --------------------------------------------------
    # First pass: collect joints in Robotran order
    # --------------------------------------------------
    joint_names = ["base"]
    joint_types = [None]
    joint_to_index = {}

    body_to_last_joint = {"base": 0}

    for body in bodies:
        body_name = body.findtext("bodyname")
        joints = body.findall("joint")

        last_idx = None
        for joint in joints:
            jname = joint.findtext("jointname")
            jtype = joint.findtext("type")

            idx = len(joint_names)
            joint_names.append(jname)
            joint_types.append(jtype)
            joint_to_index[jname] = idx
            last_idx = idx

        body_to_last_joint[body_name] = last_idx

    n = len(joint_names) - 1

    inbody = np.empty(n + 1, dtype=object)
    phi = np.zeros((n + 1, 3))
    psi = np.zeros((n + 1, 3))
    d_hi = np.zeros((n + 1, 3))
    m = np.zeros(n + 1)
    d_ii = np.zeros((n + 1, 3))
    I = np.zeros((n + 1, 3, 3))

    inbody[0] = None

    # --------------------------------------------------
    # Helper: points of each body
    # --------------------------------------------------
    body_points = {}

    for body in bodies:
        body_name = body.findtext("bodyname")
        points = {}

        for point in body.findall("point"):
            pname = point.findtext("pointname")
            points[pname] = _xml_vec3(point.find("coordinates"))

        body_points[body_name] = points

    # --------------------------------------------------
    # Second pass: build kinematic chain
    # --------------------------------------------------
    for body in bodies:
        body_name = body.findtext("bodyname")
        parent = body.find("parent")
        parent_body_name = parent.findtext("bodyname")
        parent_point_name = parent.findtext("pointname")

        parent_joint_idx = body_to_last_joint[parent_body_name]

        joints = body.findall("joint")

        previous_idx = parent_joint_idx

        for local_j, joint in enumerate(joints):
            jname = joint.findtext("jointname")
            jtype = joint.findtext("type")
            idx = joint_to_index[jname]

            inbody[idx] = previous_idx

            phi[idx], psi[idx] = _joint_axis(jtype)

            # First joint of a body sits at the selected parent point.
            # Further joints of the same body are colocated by default.
            if local_j == 0:
                if parent_body_name == "base":
                    d_hi[idx] = np.zeros(3)
                else:
                    d_hi[idx] = body_points[parent_body_name].get(
                        parent_point_name,
                        np.zeros(3)
                    )
            else:
                d_hi[idx] = np.zeros(3)

            previous_idx = idx

        # Attach mass/inertia/COM to the last joint of this body
        last_idx = body_to_last_joint[body_name]

        mass_node = body.find("mass")
        if mass_node is not None and mass_node.text is not None:
            m[last_idx] = float(mass_node.text)

        d_ii[last_idx] = _xml_vec3(body.find("com"))

        inertia = body.find("inertia")
        if inertia is not None:
            ixx = _xml_float(inertia.find("Ixx"))
            iyy = _xml_float(inertia.find("Iyy"))
            izz = _xml_float(inertia.find("Izz"))
            ixy = _xml_float(inertia.find("Ixy"))
            ixz = _xml_float(inertia.find("Ixz"))
            iyz = _xml_float(inertia.find("Iyz"))

            I[last_idx] = np.array([
                [ixx, ixy, ixz],
                [ixy, iyy, iyz],
                [ixz, iyz, izz],
            ], dtype=float)

    topology = {
        "joint_names": joint_names,
        "inbody": inbody,
        "phi": phi,
        "psi": psi,
        "d_hi": d_hi,
        "m": m,
        "d_ii": d_ii,
        "I": I,
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
                  t_current=0.0, motor_state=None):
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
    F_ext, L_ext = compute_spring_damper_cartesian_forces(q, qd, mbs_data, topology)
    M, Q = ner_generique(q, qd, qdd_dummy, mbs_data, topology, F_ext=F_ext, L_ext=L_ext)


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
    #Q_ext[1] = 1000.0     
    Q_ext[1] = main_motor_torque(t_current, qd[1], motor_state)

    # Viscous damping in arm-pendulum hinges
    # damping torque: T_damp = -d * qd
    Q_ext[4]  += -100.0   * qd[4]     # Pendule1
    Q_ext[11] += -100.0   * qd[11]    # Pendule4
    Q_ext[14] += -100.0   * qd[14]    # Pendule3
    Q_ext[17] += -20000.0 * qd[17]    # Pendule2, rusted

    # Viscous damping in pendulum-nacelle Cardan joints
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
        qc, qdc, qddc = commanded_motion(t, c)
        #qc = np.zeros(len(c))
        #qdc = np.zeros(len(c))
        #qddc = np.zeros(len(c))

        q[c] = qc
        qd[c] = qdc
    else:
        qddc = None
    # Beschleunigungen der unabhängigen Koordinaten berechnen
    qdd_u, _, _ = compute_qdd_u(q, qd, mbs_data, topology, u, c, qdd_c=qddc, 
                                t_current=t, motor_state=motor_state)

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


def main_motor_torque(t, qd1, motor_state):
    """
    Motor torque from assignment.
    T = 1000 Nm until qd1 first reaches 0.8 rad/s.
    Then T = 500 * (1 + cos(2*pi*(t - t0))) for 0.5 s.
    Then T = 0.
    """

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


def spring_damper_debug_points(q, qd, mbs_data, topology):
    """
    Debug: berechnet die Punkte der vier Feder-Dämpfer-Elemente.

    Noch keine Dynamikänderung.
    Nur Geometrieprüfung.
    """

    omega, omega_c_dot, alpha_c, beta_c, z, O_M, A_M, R, R_abs, p_abs = forward_kinematics(
        q, qd, topology, mbs_data
    )

    # Für jeden Subsystem-Zweig:
    # arm_joint = Pendel-Hinge am Arm
    # pend_joint = Cardan/Pendelende
    #
    # Punkt A am Arm: 0.5 m vom Mast weg entlang Armrichtung
    # Punkt B am Pendel: 1.5 m unter Hinge und 0.1 m nach innen
    #
    # Erstmal nur grob für Geometrie.
    data = []

    branches = [
        # name, hinge joint, cardan joint, inward direction in inertial frame at rest
        ("pend1", 4, 5, np.array([-1.0, 0.0, 0.0])),
        ("pend4", 11, 12, np.array([0.0, 1.0, 0.0])),
        ("pend3", 14, 15, np.array([1.0, 0.0, 0.0])),
        ("pend2", 17, 18, np.array([0.0, -1.0, 0.0])),
    ]

    for name, hinge, cardan, inward_global in branches:
        # Arm attachment point:
        # p_abs[hinge] liegt bei 1 m vom Mast. Feder hängt bei 0.5 m vom Mast.
        # Also nehmen wir Mittelpunkt zwischen Mastachse-Höhe und Hinge.
        p_hinge = p_abs[hinge].copy()

        p_arm = p_hinge.copy()
        p_arm[0:2] *= 0.5

        # Pendulum attachment:
        # 1.5 m unter Hinge entlang Pendelrichtung.
        # Die Pendelrichtung approximieren wir als Richtung hinge -> cardan.
        p_cardan = p_abs[cardan].copy()
        pend_vec = p_cardan - p_hinge
        L_pend = np.linalg.norm(pend_vec)

        if L_pend < 1e-12:
            e_pend = np.array([0.0, 0.0, -1.0])
        else:
            e_pend = pend_vec / L_pend

        # Punkt 1.5 m unter dem Hinge
        p_pend = p_hinge + 1.5 * e_pend

        # 0.1 m inward.
        # Für Debug erstmal mit globaler inward Richtung.
        p_pend = p_pend + 0.1 * inward_global

        spring_vec = p_pend - p_arm
        spring_len = np.linalg.norm(spring_vec)

        data.append({
            "name": name,
            "p_arm": p_arm,
            "p_pend": p_pend,
            "length": spring_len,
        })

    return data

def compute_spring_damper_Q(q, qd, mbs_data, topology):
    """
    Approximative generalized forces from the linear spring-damper elements
    between arm and pendulum.

    This version projects the force-induced moment onto the corresponding
    arm-pendulum hinge coordinate:
        q4, q11, q14, q17

    It is not yet the full virtual-work treatment on all coordinates,
    but it is the next useful step for stabilizing the pendulum motion.
    """

    k_spring = 500.0     # N/m
    L0 = 10.0            # m
    c_damp = 700.0       # Ns/m

    N_body = mbs_data.njoint

    Q_spring = np.zeros(N_body + 1)

    # Current kinematics
    omega, omega_c_dot, alpha_c, beta_c, z, O_M, A_M, R, R_abs, p_abs = forward_kinematics(
        q, qd, topology, mbs_data
    )

    # Finite-difference kinematics for point velocities
    eps = 1e-6
    q_eps = q + eps * qd

    omega_e, omega_c_dot_e, alpha_c_e, beta_c_e, z_e, O_M_e, A_M_e, R_e, R_abs_e, p_abs_e = forward_kinematics(
        q_eps, qd, topology, mbs_data
    )

    phi = topology["phi"]

    branches = [
        # name, hinge joint, cardan joint, inward direction in global rest frame
        ("pend1", 4, 5, np.array([-1.0, 0.0, 0.0])),
        ("pend4", 11, 12, np.array([0.0, 1.0, 0.0])),
        ("pend3", 14, 15, np.array([1.0, 0.0, 0.0])),
        ("pend2", 17, 18, np.array([0.0, -1.0, 0.0])),
    ]

    for name, hinge, cardan, inward_global in branches:
        # Current points
        p_hinge = p_abs[hinge].copy()
        p_cardan = p_abs[cardan].copy()

        # Arm attachment:
        # hinge is roughly 1 m from pole axis; spring attachment is 0.5 m from pole axis
        p_arm = p_hinge.copy()
        p_arm[0:2] *= 0.5

        # Pendulum direction from hinge to cardan
        pend_vec = p_cardan - p_hinge
        L_pend = np.linalg.norm(pend_vec)

        if L_pend < 1e-12:
            e_pend = np.array([0.0, 0.0, -1.0])
        else:
            e_pend = pend_vec / L_pend

        # Pendulum attachment:
        # 1.5 m below hinge along pendulum, plus 0.1 m inward
        p_pend = p_hinge + 1.5 * e_pend + 0.1 * inward_global

        # Same points at q_eps for velocity approximation
        p_hinge_e = p_abs_e[hinge].copy()
        p_cardan_e = p_abs_e[cardan].copy()

        p_arm_e = p_hinge_e.copy()
        p_arm_e[0:2] *= 0.5

        pend_vec_e = p_cardan_e - p_hinge_e
        L_pend_e = np.linalg.norm(pend_vec_e)

        if L_pend_e < 1e-12:
            e_pend_e = np.array([0.0, 0.0, -1.0])
        else:
            e_pend_e = pend_vec_e / L_pend_e

        p_pend_e = p_hinge_e + 1.5 * e_pend_e + 0.1 * inward_global

        # Point velocities
        v_arm = (p_arm_e - p_arm) / eps
        v_pend = (p_pend_e - p_pend) / eps

        # Spring vector from arm point to pendulum point
        spring_vec = p_pend - p_arm
        L = np.linalg.norm(spring_vec)

        if L < 1e-12:
            continue

        e_spring = spring_vec / L

        # Relative speed along spring
        Ldot = np.dot(v_pend - v_arm, e_spring)

        # Force applied on pendulum point
        # F = - (k(L-L0) + c Ldot) e
        F_pend = -(k_spring * (L - L0) + c_damp * Ldot) * e_spring

        # Moment about pendulum hinge
        r = p_pend - p_hinge
        M_hinge_global = np.cross(r, F_pend)

        # Project moment onto hinge axis
        axis_global = R_abs[hinge] @ phi[hinge]
        Q_spring[hinge] += np.dot(M_hinge_global, axis_global)

    return Q_spring


def spring_points(q, qd, mbs_data, topology):
    """
    Returns spring attachment points for all 4 arm-pendulum spring-damper elements.

    For each branch:
    p_arm  : attachment point on arm, 0.5 m from pole axis
    p_pend : attachment point on pendulum, 1.5 m below hinge and 0.1 m inward
    """

    omega, omega_c_dot, alpha_c, beta_c, z, O_M, A_M, R, R_abs, p_abs = forward_kinematics(
        q, qd, topology, mbs_data
    )

    branches = [
        ("pend1", 4, 5),
        ("pend4", 11, 12),
        ("pend3", 14, 15),
        ("pend2", 17, 18),
    ]

    points = []

    for name, hinge, cardan in branches:
        p_hinge = p_abs[hinge].copy()
        p_cardan = p_abs[cardan].copy()

        # Arm point: same height as hinge, halfway from pole axis to hinge.
        r_xy = p_hinge[:2]
        r_norm = np.linalg.norm(r_xy)

        if r_norm < 1e-12:
            inward = np.zeros(3)
            p_arm = p_hinge.copy()
        else:
            radial = np.array([r_xy[0], r_xy[1], 0.0]) / r_norm
            inward = -radial

            p_arm = p_hinge.copy()
            p_arm[:2] = 0.5 * radial[:2]

        # Pendulum direction from hinge to cardan
        pend_vec = p_cardan - p_hinge
        L_pend = np.linalg.norm(pend_vec)

        if L_pend < 1e-12:
            e_pend = np.array([0.0, 0.0, -1.0])
        else:
            e_pend = pend_vec / L_pend

        # Pendulum attachment:
        # 1.5 m below hinge along pendulum axis + 0.1 m inward toward pole
        p_pend = p_hinge + 1.5 * e_pend + 0.1 * inward

        points.append({
            "name": name,
            "hinge": hinge,
            "cardan": cardan,
            "p_arm": p_arm,
            "p_pend": p_pend,
        })

    return points


def compute_spring_damper_Q_virtual_work(q, qd, mbs_data, topology, active_indices):
    """
    Correct spring-damper generalized forces using numerical virtual work.

    Q_j = F_arm · dp_arm/dq_j + F_pend · dp_pend/dq_j

    This is better than projecting a moment only on the pendulum hinge.
    """

    k_spring = 500.0
    L0 = 10.0
    c_damp = 700.0

    N_body = mbs_data.njoint
    Q_spring = np.zeros(N_body + 1)

    eps = 1e-6

    # Current points
    points = spring_points(q, qd, mbs_data, topology)

    # Points at q + eps*qd for velocity approximation
    q_vel = q + eps * qd
    points_vel = spring_points(q_vel, qd, mbs_data, topology)

    # For each spring: compute physical forces
    spring_forces = []

    for p_now, p_vel in zip(points, points_vel):
        p_arm = p_now["p_arm"]
        p_pend = p_now["p_pend"]

        p_arm_vel = p_vel["p_arm"]
        p_pend_vel = p_vel["p_pend"]

        v_arm = (p_arm_vel - p_arm) / eps
        v_pend = (p_pend_vel - p_pend) / eps

        d = p_pend - p_arm
        L = np.linalg.norm(d)

        if L < 1e-12:
            F_pend = np.zeros(3)
            F_arm = np.zeros(3)
        else:
            e = d / L
            Ldot = np.dot(v_pend - v_arm, e)

            # Force on pendulum point
            F_pend = -(k_spring * (L - L0) + c_damp * Ldot) * e

            # Equal and opposite force on arm point
            F_arm = -F_pend

        spring_forces.append({
            "F_arm": F_arm,
            "F_pend": F_pend,
        })

    # Numerical virtual work:
    # perturb each active coordinate and recompute points
    for j in active_indices:
        q_plus = q.copy()
        q_plus[j] += eps

        points_plus = spring_points(q_plus, qd, mbs_data, topology)

        Qj = 0.0

        for p_now, p_plus, forces in zip(points, points_plus, spring_forces):
            dp_arm_dq = (p_plus["p_arm"] - p_now["p_arm"]) / eps
            dp_pend_dq = (p_plus["p_pend"] - p_now["p_pend"]) / eps

            Qj += np.dot(forces["F_arm"], dp_arm_dq)
            Qj += np.dot(forces["F_pend"], dp_pend_dq)

        Q_spring[j] = Qj

    return Q_spring


def compute_spring_damper_cartesian_forces(q, qd, mbs_data, topology):
    """
    Spring-damper elements as Cartesian external forces.

    Correct assignment values:
    k = 500 N/m
    L0 = 10 m
    c = 700 Ns/m
    """

    k_spring = 500.0
    L0 = 10.0
    c_damp = 700.0

    N_body = mbs_data.njoint

    F_ext = np.zeros((N_body + 1, 3))
    L_ext = np.zeros((N_body + 1, 3))

    omega, omega_c_dot, alpha_c, beta_c, z, O_M, A_M, R, R_abs, p_abs = forward_kinematics(
        q, qd, topology, mbs_data
    )

    d_ii = topology["d_ii"]
    inbody = topology["inbody"]

    # velocities by finite difference
    eps = 1e-6
    q_eps = q + eps * qd

    _, _, _, _, _, _, _, _, R_abs_eps, p_abs_eps = forward_kinematics(
        q_eps, qd, topology, mbs_data
    )

    branches = [
        # name, pendulum body / hinge joint, cardan/end joint
        ("pend1", 4, 5),
        ("pend4", 11, 12),
        ("pend3", 14, 15),
        ("pend2", 17, 18),
    ]

    for name, pend_body, cardan_body in branches:
        arm_body = int(inbody[pend_body])

        # current hinge / cardan positions
        p_hinge = p_abs[pend_body].copy()
        p_cardan = p_abs[cardan_body].copy()

        # arm attachment: 0.5 m from pole axis, same direction as hinge
        r_xy = p_hinge[:2]
        r_norm = np.linalg.norm(r_xy)

        if r_norm < 1e-12:
            radial = np.zeros(3)
            inward = np.zeros(3)
            p_arm = p_hinge.copy()
        else:
            radial = np.array([r_xy[0], r_xy[1], 0.0]) / r_norm
            inward = -radial

            p_arm = p_hinge.copy()
            p_arm[:2] = 0.5 * radial[:2]

        # pendulum attachment: 1.5 m below hinge along pendulum axis + 0.1 m inward
        pend_vec = p_cardan - p_hinge
        L_pend = np.linalg.norm(pend_vec)

        if L_pend < 1e-12:
            e_pend = np.array([0.0, 0.0, -1.0])
        else:
            e_pend = pend_vec / L_pend

        p_pend = p_hinge + 1.5 * e_pend + 0.1 * inward

        # same points at q + eps*qd for velocity
        p_hinge_eps = p_abs_eps[pend_body].copy()
        p_cardan_eps = p_abs_eps[cardan_body].copy()

        r_xy_eps = p_hinge_eps[:2]
        r_norm_eps = np.linalg.norm(r_xy_eps)

        if r_norm_eps < 1e-12:
            radial_eps = np.zeros(3)
            inward_eps = np.zeros(3)
            p_arm_eps = p_hinge_eps.copy()
        else:
            radial_eps = np.array([r_xy_eps[0], r_xy_eps[1], 0.0]) / r_norm_eps
            inward_eps = -radial_eps

            p_arm_eps = p_hinge_eps.copy()
            p_arm_eps[:2] = 0.5 * radial_eps[:2]

        pend_vec_eps = p_cardan_eps - p_hinge_eps
        L_pend_eps = np.linalg.norm(pend_vec_eps)

        if L_pend_eps < 1e-12:
            e_pend_eps = np.array([0.0, 0.0, -1.0])
        else:
            e_pend_eps = pend_vec_eps / L_pend_eps

        p_pend_eps = p_hinge_eps + 1.5 * e_pend_eps + 0.1 * inward_eps

        v_arm = (p_arm_eps - p_arm) / eps
        v_pend = (p_pend_eps - p_pend) / eps

        # spring force
        d = p_pend - p_arm
        L = np.linalg.norm(d)

        if L < 1e-12:
            continue

        e = d / L
        Ldot = np.dot(v_pend - v_arm, e)

        # force on pendulum point
        F_pend_global = -(k_spring * (L - L0) + c_damp * Ldot) * e

        # opposite force on arm point
        F_arm_global = -F_pend_global

        # helper: apply point force to body as force + moment about COM
        def apply_point_force(body, p_point_global, F_global):
            p_com_global = p_abs[body] + R_abs[body] @ d_ii[body]

            M_global = np.cross(p_point_global - p_com_global, F_global)

            # transform global force/moment into body frame
            F_local = R_abs[body].T @ F_global
            M_local = R_abs[body].T @ M_global

            F_ext[body] += F_local
            L_ext[body] += M_local

        apply_point_force(pend_body, p_pend, F_pend_global)
        apply_point_force(arm_body, p_arm, F_arm_global)

    return F_ext, L_ext