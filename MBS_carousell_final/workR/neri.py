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
