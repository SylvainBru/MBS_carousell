import numpy as np 
import MBsysPy as Robotran
import xml.etree.ElementTree as ET



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
        if phi[0] == 1.0:   # Autour de X — rotation passive R^{i,h}
            return np.array([[1, 0,  0],
                            [0, c,  s],
                            [0, -s, c]])
        elif phi[1] == 1.0: # Autour de Y — rotation passive R^{i,h}
            return np.array([[c, 0, -s],
                            [0, 1,  0],
                            [s, 0,  c]])
        elif phi[2] == 1.0: # Autour de Z — rotation passive R^{i,h}
            return np.array([[ c, s, 0],
                            [-s, c, 0],
                            [ 0, 0, 1]])


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




'''
def define_topology_from_mbs(mbs_path):
    """
    Parse le fichier .mbs XML et construit automatiquement le dictionnaire
    de topologie NERi (inbody, phi, psi, d_hi, m, d_ii, I).

    Convention NERi:
      - Un corps avec N joints => N corps virtuels chaînés
      - Premier joint du corps : d_hi = coordonnées du point d'ancrage sur le parent
      - Joints suivants (même corps) : d_hi = [0,0,0]
      - Masse/inertie sur le DERNIER joint du corps
    """
    JOINT_AXES = {
        'R1': (np.array([1., 0., 0.]), np.zeros(3)),
        'R2': (np.array([0., 1., 0.]), np.zeros(3)),
        'R3': (np.array([0., 0., 1.]), np.zeros(3)),
        'T1': (np.zeros(3), np.array([1., 0., 0.])),
        'T2': (np.zeros(3), np.array([0., 1., 0.])),
        'T3': (np.zeros(3), np.array([0., 0., 1.])),
    }

    mbs_data = Robotran.MbsData(mbs_path)
    N = mbs_data.njoint

    inbody = np.empty(N + 1, dtype=object)
    phi    = np.zeros((N + 1, 3))
    psi    = np.zeros((N + 1, 3))
    d_hi   = np.zeros((N + 1, 3))
    m      = np.zeros(N + 1)
    d_ii   = np.zeros((N + 1, 3))
    I_arr  = np.zeros((N + 1, 3, 3))
    inbody[0] = None

    jname_to_idx = dict(mbs_data.joint_id)

    tree = ET.parse(mbs_path)
    root = tree.getroot()

    # Pré-calcul des données physiques depuis l'API mbs_data (comme Nicolas)
    masses   = mbs_data.m                                              # shape (nbody+2,)
    coms     = mbs_data.l[1:4]                                        # shape (3, nbody+2)
    inertias = np.reshape(mbs_data.In[1:10].T, (-1, 3, 3), order='F')# shape (nbody+2, 3, 3)
    bname_to_bidx = dict(mbs_data.body_id)

    # Collecte des données de chaque corps
    body_info = {}
    body_info['base'] = {
        'last_joint_idx': 0,
        'points': {'origin': np.zeros(3)},
    }

    bodytree = root.find('bodytree')
    for body_elem in bodytree.findall('body'):
        bname = body_elem.find('bodyname').text

        par = body_elem.find('parent')
        parent_bname = par.find('bodyname').text
        parent_ptname = par.find('pointname').text

        joints = [(j.find('jointname').text, j.find('type').text)
                  for j in body_elem.findall('joint')]

        bidx     = bname_to_bidx[bname]
        mass_val = float(masses[bidx])
        com_vec  = np.array(coms[:, bidx], dtype=float)
        I_mat    = inertias[bidx].copy()

        points = {}
        for pt in body_elem.findall('point'):
            ptname = pt.find('pointname').text
            coords = pt.find('coordinates')
            points[ptname] = np.array([float(coords.find('x').text),
                                       float(coords.find('y').text),
                                       float(coords.find('z').text)])

        body_info[bname] = {
            'parent_body':     parent_bname,
            'parent_point':    parent_ptname,
            'joints':          joints,
            'mass':            mass_val,
            'com':             com_vec,
            'inertia':         I_mat,
            'points':          points,
            'last_joint_idx':  None,
        }

    # Remplissage des tableaux en suivant l'ordre DFS du fichier
    for body_elem in bodytree.findall('body'):
        bname  = body_elem.find('bodyname').text
        bd     = body_info[bname]
        joints = bd['joints']

        # Coordonnées du point d'ancrage sur le corps parent
        parent_pts = body_info[bd['parent_body']]['points']
        ptname     = bd['parent_point']
        attach     = (np.zeros(3) if ptname == 'origin'
                      else parent_pts.get(ptname, np.zeros(3)))

        parent_last = body_info[bd['parent_body']]['last_joint_idx']

        for j_pos, (jname, jtype) in enumerate(joints):
            idx = jname_to_idx.get(jname)
            if idx is None:
                continue

            phi_v, psi_v = JOINT_AXES[jtype]
            phi[idx] = phi_v
            psi[idx] = psi_v

            if j_pos == 0:
                inbody[idx] = parent_last
                d_hi[idx]   = attach
            else:
                prev_idx    = jname_to_idx[joints[j_pos - 1][0]]
                inbody[idx] = prev_idx
                d_hi[idx]   = np.zeros(3)

            if j_pos == len(joints) - 1:
                m[idx]     = bd['mass']
                d_ii[idx]  = bd['com']
                I_arr[idx] = bd['inertia']
                bd['last_joint_idx'] = idx

    return {
        'inbody': inbody,
        'phi':    phi,
        'psi':    psi,
        'd_hi':   d_hi,
        'm':      m,
        'd_ii':   d_ii,
        'I':      I_arr,
    }'''




'''
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
    np.array([0, 0, 0]),           # 3: R2_Pole (no translation)
    np.array([1, 0, 4.5]),           # 4: R2_arm_pend1 (no translation)
    np.array([0, 0, -3]),           # 5: R2_cardan1 (no translation)
    np.array([0, 0, 0]),           # 6: R1_cardan1 (no translation)
    np.array([0, 0, 4.5]),           # 7: R2_arm_pend2 (T1 translation along X)
    np.array([0, -1, 0]),           # 8: T1_effort_normal (translation along X)
    np.array([0, 0, 0]),           # 9: T3_effort_tranchant (translation along Z)
    np.array([0, 0, 0]),           # 10: R2_effort_flechissant (no translation)
    np.array([0, 0, 0]),           # 11: arm_pend4 (no translation)
    np.array([0, 0, -3]),
    np.array([0, 0, 0]),           # 12: R_cardan4a (no translation)
    np.array([-1, 0, 4.5]),           # 13: R_cardan4b (no translation)
    np.array([0, 0, -3]),           # 14: arm_pend3 (no translation)
    np.array([0, 0, 0]),           # 15: R_cardan3a (no translation)
    np.array([0, 1, 4.5]),           # 16: R_cardan3b (no translation)
    np.array([0, 0, -3]),           # 17: arm_pend2 (no translation),           # 18: R_cardan2a (no translation)
    np.array([0, 0, 0])            # 19: R_cardan2b (no translation)
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
    np.array([0.0, 0.0, 0.0]),     
    np.array([0.0, 0.0, 0.0]),     
    np.array([0.0, 0.0, 0.0]),     
    np.array([0.0, 0.0, 2.0]),     
    np.array([0.0, 0.0, -1.5]),   
    np.array([0.0, 0.0, 0.0]),   
    np.array([0.0, 0.0, -1.0]),   
    np.array([-0.5, 0.0, 0.0]),    
    np.array([0.0, 0.0, 0.0]),    
    np.array([0.0, 0.0, 0.0]),    
    np.array([0.0, 0.0, 0.0]),    
    np.array([0.0, 0.0, -1.5]),   
    np.array([0.0, 0.0, 0.0]),   
    np.array([0.0, 0.0, -1.0]),    
    np.array([0.0, 0.0, -1.5]),   
    np.array([0.0, 0.0, 0.0]),    
    np.array([0.0, 0.0, -1.0]),    
    np.array([0.0, 0.0, -1.5]),    
    np.array([0.0, 0.0, 0.0]),    
    np.array([0.0, 0.0, -1.0])    
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
    
    return topology'''





'''
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

    print("total mass =", total_mass)'''