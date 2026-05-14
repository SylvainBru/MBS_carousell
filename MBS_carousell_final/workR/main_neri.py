import os
import numpy as np
import MBsysPy as Robotran
from scipy import integrate
import matplotlib.pyplot as plt
import neri


#########################################
###     Chargement du fichier .mbs   ####
#########################################

_script_dir = os.path.dirname(os.path.abspath(__file__))
_mbs_path   = os.path.join(_script_dir, '..', 'dataR', 'Merry_go_round.mbs')
_mbs_data   = Robotran.MbsData(_mbs_path)


#########################################
###     Definition de la topologie   ####
#########################################
N = 19

inbody = np.array(
    [None, 0, 1, 2, 3, 4, 5, 3, 7, 8, 9, 10, 11, 12, 3, 14, 15, 3, 17, 18],
    dtype=object)

phi_arr = np.array([
    [0,0,0],  # 0
    [0,0,1],  # 1  R3_Pole
    [1,0,0],  # 2  R1_Pole  (commandé)
    [0,1,0],  # 3  R2_Pole  (commandé)
    [0,1,0],  # 4  pend1
    [0,1,0],  # 5  cardan1a
    [1,0,0],  # 6  nacelle1
    [0,0,0],  # 7  T1
    [0,0,0],  # 8  T1
    [0,0,0],  # 9  T3
    [0,1,0],  # 10
    [1,0,0],  # 11 pend4
    [1,0,0],  # 12 cardan4a
    [0,1,0],  # 13 nacelle4
    [0,1,0],  # 14 pend3
    [0,1,0],  # 15 cardan3a
    [1,0,0],  # 16 nacelle3
    [1,0,0],  # 17 pend2
    [1,0,0],  # 18 cardan2a
    [0,1,0],  # 19 nacelle2
], dtype=float)

psi_arr = np.array([
    [0,0,0],
    [0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],
    [1,0,0],  # 7  T1
    [1,0,0],  # 8  T1
    [0,0,1],  # 9  T3
    [0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],
    [0,0,0],[0,0,0],[0,0,0],[0,0,0],
], dtype=float)

d_hi = np.array([
    [ 0,  0,   0  ],
    [ 0,  0,   0  ],
    [ 0,  0,   0  ],
    [ 0,  0,   0  ],
    [ 1,  0,   4.5],  # 4  bras1
    [ 0,  0,  -3  ],  # 5
    [ 0,  0,   0  ],  # 6  nacelle1
    [ 0,  0,   4.5],  # 7
    [ 0, -1,   0  ],  # 8  bras4
    [ 0,  0,   0  ],
    [ 0,  0,   0  ],
    [ 0,  0,   0  ],  # 11 pend4
    [ 0,  0,  -3  ],
    [ 0,  0,   0  ],  # 13 nacelle4
    [-1,  0,   4.5],  # 14 bras3
    [ 0,  0,  -3  ],
    [ 0,  0,   0  ],  # 16 nacelle3
    [ 0,  1,   4.5],  # 17 bras2
    [ 0,  0,  -3  ],
    [ 0,  0,   0  ],  # 19 nacelle2
], dtype=float)

# Données physiques depuis le .mbs 
_mass_raw = _mbs_data.m
_com_raw  = _mbs_data.l[1:4].T
_In_raw   = np.reshape(_mbs_data.In[1:10].T, (-1, 3, 3), order='F')

_body_to_joint = {
    "Pole":      3,  "Pendule1":  4,  "nacelle1":  6,
    "arm_part1": 7,  "arm_part2": 10, "Pendule4":  11,
    "nacelle4":  13, "Pendule3":  14, "nacelle3":  16,
    "Pendule2":  17, "nacelle2":  19,
}

m_arr = np.zeros(N + 1)
d_ii  = np.zeros((N + 1, 3))
I_arr = np.zeros((N + 1, 3, 3))

for _bname, _ji in _body_to_joint.items():
    _bi        = _mbs_data.body_id[_bname]
    m_arr[_ji] = _mass_raw[_bi]
    d_ii[_ji]  = _com_raw[_bi]
    I_arr[_ji] = _In_raw[_bi]

# Dictionnaire topologie pour neri.py 
topology = {
    "inbody": inbody,
    "phi":    phi_arr,
    "psi":    psi_arr,
    "d_hi":   d_hi,
    "m":      m_arr,
    "d_ii":   d_ii,
    "I":      I_arr,
}


#########################################
###     couple commande moteur       ####
#########################################

motor_state = {"t0": None}

def motor_torque(t, qd1):
    A, w = 500.0, 2.0 * np.pi
    if motor_state["t0"] is None:
        if abs(qd1) < 0.8:
            return 1000.0
        motor_state["t0"] = t
    t0 = motor_state["t0"]
    if t < t0 + 0.5:
        return A * (1.0 + np.cos(w * t - 2.0 * np.pi * t0))
    return 0.0


##########################################################################
###     Formalisme NERi + Joint/external Force, Driven variable       ####
##########################################################################

#  Joints commandés
_A_tilt = 2.5 * 2.0 * np.pi / 360.0
_w_tilt = 0.4 * np.pi

def driven_kinematics(t):
    q2   = _A_tilt * (1.0 - np.cos(_w_tilt * t + np.pi / 2.0))
    qd2  = _A_tilt * _w_tilt * np.sin(_w_tilt * t + np.pi / 2.0)
    qdd2 = _A_tilt * _w_tilt**2 * np.cos(_w_tilt * t + np.pi / 2.0)
    q3   = _A_tilt * (1.0 - np.cos(_w_tilt * t))
    qd3  = _A_tilt * _w_tilt * np.sin(_w_tilt * t)
    qdd3 = _A_tilt * _w_tilt**2 * np.cos(_w_tilt * t)
    return {2: (q2, qd2, qdd2), 3: (q3, qd3, qdd3)}

#  Modèle physique
def physical_model(t, q, qd, u_idx, c_idx, qdd_c_ext):

    # Forward
    (omega, omega_c_dot, alpha_c, beta_c,
     z, O_M, A_M, R_neri, R_abs, _) = neri.forward_kinematics(
        q, qd, topology, _mbs_data
    )

    # forces extérieures
    F_wind_t        = 300.0 * (1.0 - np.cos(6.0 * np.pi * t))
    F_wind_inertial = np.array([F_wind_t, 0.0, 0.0])

    F_ext = np.zeros((N + 1, 3))
    for nac_j in [6, 13, 16, 19]:
        F_ext[nac_j] = R_abs[nac_j].T @ F_wind_inertial

    # Backward
    W_c, F_c, L_c, Q_neri, W_M, F_M, L_M, M_mat = neri.backward_dynamics(
        topology, _mbs_data,
        omega, omega_c_dot, alpha_c, beta_c, z, R_neri, O_M, A_M,
        F_ext=F_ext, L_ext=None
    )

    # couple au joints
    Q_ext = np.zeros(N + 1)

    Q_ext[1] += motor_torque(t, qd[1])

    q_eq     = 75.0 * np.pi / 180.0
    D_rot_eq = 175.0
    K_rot_eq = 125.0
    pend_cfg = {
        4:  (   100.0,  -2145.0),
        17: ( 20000.0,  +2145.0),
        14: (   100.0,  +2145.0),
        11: (   100.0,  -2145.0),
    }
    for jp, (D_h, T_pre) in pend_cfg.items():
        Q_ext[jp] += -(D_h + D_rot_eq) * qd[jp]
        Q_ext[jp] += -K_rot_eq * (q[jp] - q_eq)
        Q_ext[jp] += T_pre

    D_cardan = 6.0
    for jc in [5, 6, 12, 13, 15, 16, 18, 19]:
        Q_ext[jc] += -D_cardan * qd[jc]

    # Résolution partionnée
    qdd_c = np.zeros(len(c_idx))
    for k, j in enumerate(c_idx):
        if j in qdd_c_ext:
            qdd_c[k] = qdd_c_ext[j]

    Muu = M_mat[np.ix_(u_idx, u_idx)]
    Muc = M_mat[np.ix_(u_idx, c_idx)]
    rhs = (Q_ext - Q_neri)[u_idx] - Muc @ qdd_c

    qdd_u = np.linalg.solve(Muu, rhs)

    qdd_full        = np.zeros(N + 1)
    qdd_full[u_idx] = qdd_u
    qdd_full[c_idx] = qdd_c

    Mcu = M_mat[np.ix_(c_idx, u_idx)]
    Mcc = M_mat[np.ix_(c_idx, c_idx)]
    Q_c = Mcu @ qdd_u + Mcc @ qdd_c + Q_neri[c_idx]

    return qdd_full, Q_c


#  main
if __name__ == "__main__":

    mbs_data = _mbs_data
    mbs_data.process = 1
    mbs_part = Robotran.MbsPart(mbs_data)
    mbs_part.set_options(rowperm=1, verbose=1)
    mbs_part.run()

    u_idx = np.array(mbs_data.qu[1:mbs_data.nqu + 1], dtype=int)
    c_idx = np.array(mbs_data.qc[1:mbs_data.nqc + 1], dtype=int)
    print("u (indépendants)    =", u_idx)
    print("c (commandés) =", c_idx)

    # Conditions initiales depuis le .mbs
    q0  = np.zeros(N + 1)
    qd0 = np.zeros(N + 1)
    q0[1:N+1]  = _mbs_data.q0[1:N+1]
    qd0[1:N+1] = _mbs_data.qd0[1:N+1]

    dk0 = driven_kinematics(0.0)
    for j, (qv, qdv, _) in dk0.items():
        q0[j]  = qv
        qd0[j] = qdv

    y0 = np.concatenate((q0[u_idx], qd0[u_idx]))
    nu = len(u_idx)

    def ode_fun(t, y):
        q_full  = np.zeros(N + 1)
        qd_full = np.zeros(N + 1)
        q_full[u_idx]  = y[:nu]
        qd_full[u_idx] = y[nu:]

        dk = driven_kinematics(t)
        qdd_c_driven = {}
        for j, (qv, qdv, qddv) in dk.items():
            q_full[j]       = qv
            qd_full[j]      = qdv
            qdd_c_driven[j] = qddv

        qdd_full, Q_full = physical_model(t, q_full, qd_full, u_idx, c_idx, qdd_c_driven)
        return np.concatenate((qd_full[u_idx], qdd_full[u_idx]))

    tf = 3.0
    print(f"\nIntégration  0 → {tf} s  ...")

    t_eval = np.arange(0.0, tf, 1e-3)

    sol = integrate.solve_ivp(
        ode_fun, (0.0, tf), y0,
        method='RK45',
        rtol=1e-5, atol=1e-8,
        t_eval=t_eval,
    )
    print("Statut :", sol.message)

    time   = sol.t
    q_sol  = sol.y[:nu].T
    qd_sol = sol.y[nu:].T

    def jq(j):
        return q_sol[:,  np.where(u_idx == j)[0][0]]
    def jqd(j):
        return qd_sol[:, np.where(u_idx == j)[0][0]]

    idx_j1  = np.where(u_idx == 1)[0][0]
    qd1_deg = qd_sol[:, idx_j1] * 180.0 / np.pi

    print(f"\nVitesse finale pôle : {qd1_deg[-1]:.2f} deg/s")
    print(f"Motor t0_reach      : {motor_state['t0']}")

    q2_t = np.array([driven_kinematics(t)[2][0] for t in time])
    q3_t = np.array([driven_kinematics(t)[3][0] for t in time])

    _plot_dir = os.path.join(_script_dir, "neri_plot")
    os.makedirs(_plot_dir, exist_ok=True)

    def save(fig, name):
        fig.savefig(os.path.join(_plot_dir, name + ".png"), dpi=150)
        plt.close(fig)

    # ── Plot 1 : vitesse angulaire pôle principal ─────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, qd1_deg, label='Main pole angular velocity')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular velocity [deg/s]')
    ax.set_title('Angular velocity around the vertical axis of the main pole')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    save(fig, "01_pole_velocity")

    # ── Plot 2 : angles des pendules ──────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, np.rad2deg(jq(4)),  label='Pendule 1')
    ax.plot(time, np.rad2deg(jq(17)), label='Pendule 2 (rouillé)')
    ax.plot(time, np.rad2deg(jq(14)), label='Pendule 3')
    ax.plot(time, np.rad2deg(jq(11)), label='Pendule 4')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Pendulum angles')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    save(fig, "02_pendulum_angles")

    # ── Plot 3 : vitesses angulaires des pendules ─────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, np.rad2deg(jqd(4)),  label='Pendule 1')
    ax.plot(time, np.rad2deg(jqd(17)), label='Pendule 2 (rouillé)')
    ax.plot(time, np.rad2deg(jqd(14)), label='Pendule 3')
    ax.plot(time, np.rad2deg(jqd(11)), label='Pendule 4')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular velocity [deg/s]')
    ax.set_title('Pendulum angular velocities')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    save(fig, "03_pendulum_velocities")

    # ── Plot 4 : joints commandés (inclinaison pôle) ──────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, np.rad2deg(q2_t), label='q2 — R1_Pole')
    ax.plot(time, np.rad2deg(q3_t), label='q3 — R2_Pole')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Driven joint angles (pole tilt)')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    save(fig, "04_driven_joints")


    ### effort interne
    print("Post-traitement : calcul de Q_c sur toute la simulation...")
    
    # On prépare un tableau pour stocker Q_c (taille : nb_points_de_temps x nb_joints_commandés)
    Q_c_sol = np.zeros((len(time), len(c_idx)))
    
    # Si tu as modifié physical_model pour renvoyer Q_full (tous les joints), 
    # tu pourrais créer : Q_full_sol = np.zeros((len(time), N + 1))

    for i, t in enumerate(time):
        q_full  = np.zeros(N + 1)
        qd_full = np.zeros(N + 1)
        
        # 1. Assigner les états indépendants (solveur)
        q_full[u_idx]  = q_sol[i, :]
        qd_full[u_idx] = qd_sol[i, :]
        
        # 2. Assigner les états commandés (cinématique imposée)
        dk = driven_kinematics(t)
        qdd_c_driven = {}
        for j, (qv, qdv, qddv) in dk.items():
            q_full[j]       = qv
            qd_full[j]      = qdv
            qdd_c_driven[j] = qddv
            
        # 3. Rappel du modèle physique
        # (Si ton physical_model renvoie Q_full à l'avenir, tu le récupères ici)
        qdd_full_t, Q_c_t = physical_model(t, q_full, qd_full, u_idx, c_idx, qdd_c_driven)
        
        # 4. Sauvegarde pour cet instant t
        Q_c_sol[i, :] = Q_c_t
    
    # -------------------------------------------------------------------------
    # Sauvegarde de Q_c dans un fichier .txt au format Robotran
    # -------------------------------------------------------------------------
    print("Sauvegarde des données Q_c dans un fichier texte...")

    # Création d'une matrice remplie de zéros : (nb_temps) lignes x (N + 1) colonnes
    # L'index 0 sera pour le temps, les index 1 à N (19) pour les joints.
    output_data = np.zeros((len(time), N + 1))

    # 1. Remplir la première colonne avec le temps
    output_data[:, 0] = time

    # 2. Remplir les colonnes des variables commandées (les autres restent à 0.0)
    for k, j in enumerate(c_idx):
        output_data[:, j] = Q_c_sol[:, k]

    # Chemin du fichier (tu peux changer le nom du .txt si besoin)

    _data_dir = os.path.join(_script_dir, "data")
    os.makedirs(_data_dir, exist_ok=True)
    output_filepath = os.path.join(_data_dir, "Qc_driven_neri.txt")

    # Sauvegarde avec 6 décimales en notation scientifique, séparées par un espace
    np.savetxt(
        output_filepath, 
        output_data, 
        fmt='%.6e',     # Format: 0.000000e+00
        delimiter=' '   # Espace entre chaque colonne
    )
    print(f"Fichier sauvegardé : {output_filepath}\n")
    # -------------------------------------------------------------------------

    print("Post-traitement terminé.\n")

    # ── Plot : Efforts requis pour les joints commandés (Q_c) ─────────────────
    fig, ax = plt.subplots(figsize=(9, 5))

    # On trouve dynamiquement quelle colonne de Q_c_sol correspond au joint 8 et 9
    idx_8 = np.where(c_idx == 8)[0][0]
    idx_9 = np.where(c_idx == 9)[0][0]

    # On trace les colonnes correspondantes
    ax.plot(time, Q_c_sol[:, idx_8], label='Effort joint 8 (T1 - Normal)')
    ax.plot(time, Q_c_sol[:, idx_9], label='Effort joint 9 (T3 - Tranchant)')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Effort généralisé [N]') # En Newtons car ce sont des translations
    ax.set_title('Efforts internes nécessaires (Normal et Tranchant)')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    
    save(fig, "06_efforts_internes")
    plt.show()


    ### =======================================================================
    ### INTERNAL FORCES SPATIAL ANALYSIS (-Y) WITH TIME & SIZING
    ### =======================================================================
    print("\nStarting internal forces spatial analysis...")

    # Beam parameters
    L_tot = 1.0       # Total length (since max at Y = -1)
    M_tot = 30.0      # Total mass of this section
    num_points = 20   
    
    # x represents the distance from the base of the beam (always positive)
    x_positions = np.linspace(0.01, L_tot - 0.01, num_points)

    # Arrays to store MAX internal forces (Envelope)
    max_normal  = np.zeros(num_points)
    max_shear   = np.zeros(num_points)
    max_bending = np.zeros(num_points)

    # Arrays to store the exact time when the max occurs
    time_max_normal  = np.zeros(num_points)
    time_max_shear   = np.zeros(num_points)
    time_max_bending = np.zeros(num_points)

    # Arrays to store the full time history (for time plots)
    history_normal  = np.zeros((num_points, len(time)))
    history_shear   = np.zeros((num_points, len(time)))
    history_bending = np.zeros((num_points, len(time)))

    idx_8  = np.where(c_idx == 8)[0][0]  # T1 (Normal)
    idx_9  = np.where(c_idx == 9)[0][0]  # T3 (Shear)
    idx_10 = np.where(c_idx == 10)[0][0] # R2 (Bending)

    import copy
    topo_base = copy.deepcopy(topology)

    # Create a subfolder for the spatial data text files
    _data_spatial_dir = os.path.join(_script_dir, "data_spatial")
    os.makedirs(_data_spatial_dir, exist_ok=True)

    for idx_x, x in enumerate(x_positions):
        
        # 1. Update topology for a cut at position Y = -x
        m1 = M_tot * (x / L_tot)
        m2 = M_tot * ((L_tot - x) / L_tot)
        
        topology["m"][7]  = m1  
        topology["m"][10] = m2  
        
        topology["d_ii"][7]  = np.array([0.0, -x / 2.0, 0.0]) 
        topology["d_ii"][10] = np.array([0.0, -(L_tot - x) / 2.0, 0.0])
        
        I1_val = (1.0 / 12.0) * m1 * x**2
        I2_val = (1.0 / 12.0) * m2 * (L_tot - x)**2
        topology["I"][7]  = np.diag([I1_val, 0.0, I1_val]) 
        topology["I"][10] = np.diag([I2_val, 0.0, I2_val])
        
        topology["d_hi"][8] = np.array([0.0, -x, 0.0])

        # 2. Time loop for this specific cut position
        Q_c_x = np.zeros((len(time), len(c_idx)))
        
        for i, t in enumerate(time):
            q_full  = np.zeros(N + 1)
            qd_full = np.zeros(N + 1)
            
            q_full[u_idx]  = q_sol[i, :]
            qd_full[u_idx] = qd_sol[i, :]
            
            dk = driven_kinematics(t)
            qdd_c_driven = {}
            for j, (qv, qdv, qddv) in dk.items():
                q_full[j]       = qv
                qd_full[j]      = qdv
                qdd_c_driven[j] = qddv
                
            qdd_full_t, Q_c_t = physical_model(t, q_full, qd_full, u_idx, c_idx, qdd_c_driven)
            Q_c_x[i, :] = Q_c_t
            
        # 3. Save into history arrays
        history_normal[idx_x, :]  = Q_c_x[:, idx_8]
        history_shear[idx_x, :]   = Q_c_x[:, idx_9]
        history_bending[idx_x, :] = Q_c_x[:, idx_10]
        
        # 4. Find the MAX absolute value and its corresponding time
        i_max_n = np.argmax(np.abs(history_normal[idx_x, :]))
        i_max_s = np.argmax(np.abs(history_shear[idx_x, :]))
        i_max_b = np.argmax(np.abs(history_bending[idx_x, :]))
        
        max_normal[idx_x]  = np.abs(history_normal[idx_x, i_max_n])
        max_shear[idx_x]   = np.abs(history_shear[idx_x, i_max_s])
        max_bending[idx_x] = np.abs(history_bending[idx_x, i_max_b])
        
        time_max_normal[idx_x]  = time[i_max_n]
        time_max_shear[idx_x]   = time[i_max_s]
        time_max_bending[idx_x] = time[i_max_b]
        
        print(f"Cut at Y=-{x:.2f}m -> Max Normal: {max_normal[idx_x]:.0f}N (at {time_max_normal[idx_x]:.2f}s) | Max Bending: {max_bending[idx_x]:.0f}Nm (at {time_max_bending[idx_x]:.2f}s)")

        # 5. Export text file for this position
        output_data = np.zeros((len(time), N + 1))
        output_data[:, 0] = time
        for k, j in enumerate(c_idx):
            output_data[:, j] = Q_c_x[:, k]
            
        file_name = f"Qc_driven_neri_x_{x:.2f}.txt"
        output_filepath = os.path.join(_data_spatial_dir, file_name)
        np.savetxt(output_filepath, output_data, fmt='%.6e', delimiter=' ')

    # Restore original topology
    topology = topo_base

    print(f"\nAll spatial data files successfully saved in: {_data_spatial_dir}")

    # =========================================================================
    # PLOT 1 : TIME EVOLUTION OF FORCES FOR EACH CUT POSITION
    # =========================================================================
    import matplotlib.cm as cm
    colors = cm.viridis(np.linspace(0, 1, num_points))

    fig_time, axs_time = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    for idx_x, x in enumerate(x_positions):
        axs_time[0].plot(time, history_normal[idx_x, :], color=colors[idx_x], alpha=0.7)
        axs_time[1].plot(time, history_shear[idx_x, :], color=colors[idx_x], alpha=0.7)
        axs_time[2].plot(time, history_bending[idx_x, :], color=colors[idx_x], alpha=0.7)

    axs_time[0].set_ylabel('Normal Force [N]')
    axs_time[0].set_title("Time evolution of internal forces (Blue = Beam base, Yellow = Beam tip)")
    axs_time[0].grid(True)
    
    axs_time[1].set_ylabel('Shear Force [N]')
    axs_time[1].grid(True)
    
    axs_time[2].set_ylabel('Bending Moment [N.m]')
    axs_time[2].set_xlabel('Time [s]')
    axs_time[2].grid(True)

    fig_time.tight_layout()
    save(fig_time, "08_efforts_temporels_multiples_ENG")

    # =========================================================================
    # PLOT 2 : SPATIAL ENVELOPE (WITH TIME COLORMAP)
    # =========================================================================
    fig_env, axs_env = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    cmap_time = 'plasma'

    # Normal Force
    axs_env[0].plot(x_positions, max_normal, color='gray', linestyle='--', alpha=0.5) 
    sc1 = axs_env[0].scatter(x_positions, max_normal, c=time_max_normal, cmap=cmap_time, zorder=5, s=60)
    axs_env[0].set_ylabel('Max Normal Force [N]')
    axs_env[0].set_title("Envelope of maximum internal forces (Color = Time of max in s)")
    axs_env[0].grid(True)
    fig_env.colorbar(sc1, ax=axs_env[0], label='Time [s]')

    # Shear Force
    axs_env[1].plot(x_positions, max_shear, color='gray', linestyle='--', alpha=0.5)
    sc2 = axs_env[1].scatter(x_positions, max_shear, c=time_max_shear, cmap=cmap_time, zorder=5, s=60)
    axs_env[1].set_ylabel('Max Shear Force [N]')
    axs_env[1].grid(True)
    fig_env.colorbar(sc2, ax=axs_env[1], label='Time [s]')

    # Bending Moment
    axs_env[2].plot(x_positions, max_bending, color='gray', linestyle='--', alpha=0.5)
    sc3 = axs_env[2].scatter(x_positions, max_bending, c=time_max_bending, cmap=cmap_time, zorder=5, s=60)
    axs_env[2].set_ylabel('Max Bending Moment [N.m]')
    axs_env[2].set_xlabel('Position along the beam (Distance from base) [m]')
    axs_env[2].grid(True)
    fig_env.colorbar(sc3, ax=axs_env[2], label='Time [s]')

    fig_env.tight_layout()
    save(fig_env, "07_enveloppe_efforts_poutre_avec_temps_ENG")
    
    # =========================================================================
    # SOLID RECTANGULAR CROSS-SECTION SIZING (RDM)
    # =========================================================================
    print("\n" + "="*50)
    print(" SOLID RECTANGULAR BEAM SIZING (STRESS ANALYSIS)")
    print("="*50)
    from scipy.optimize import fsolve



    # --- DONNÉES D'ENTRÉE ---
    N_max  = np.max(max_normal)
    V_max  = np.max(max_shear)
    M_max = np.max(max_bending)

    print(f"Absolute Max Normal Force (N)   : {N_max:.0f} N")
    print(f"Absolute Max Shear Force (V)    : {V_max:.0f} N")
    print(f"Absolute Max Bending Moment (M) : {M_max:.0f} N.m")

    n = 10            # Facteur de sécurité souhaité

    # Propriété du matériau (Acier classique AISI 1020 HR selon Juvinall)
    Sy = 210e6        # Limite d'élasticité en Pascals (210 MPa)
    stress_adm = Sy / n

    def dimensionner_poutre(ratio_h_b=2.0):
        """
        Calcule la largeur 'b' et la hauteur 'h' de la section.
        Hypothèse par défaut : h = 2 * b (section rectangulaire standard)
        """
        def critere_faille(b):
            h = ratio_h_b * b
            A = b * h           # Aire
            Z = (b * h**2) / 6  # Module de section (flexion)
            
            # 1. Point critique à la fibre extrême (Max flexion)
            # Juvinall Eq. (4.11) & (4.16)
            sigma_flexion = M_max / Z
            sigma_axial = N_max / A
            sigma_tot_fiber = sigma_flexion + sigma_axial
            tau_fiber = 0
            # Von Mises (Distortion Energy Theory) - Juvinall Eq. (6.8)
            vm_fiber = np.sqrt(sigma_tot_fiber**2 + 3 * tau_fiber**2)
            
            # 2. Point critique à l'axe neutre (Max cisaillement)
            # Juvinall Eq. (4.20) pour un rectangle : tau = 1.5 * V / A
            sigma_tot_neutral = sigma_axial
            tau_max_shear = (1.5 * V_max) / A
            vm_neutral = np.sqrt(sigma_tot_neutral**2 + 3 * tau_max_shear**2)
            
            # On dimensionne sur la contrainte la plus élevée des deux
            return max(vm_fiber, vm_neutral) - stress_adm

        # Résolution numérique pour b
        b_sol = fsolve(critere_faille, 0.01)[0] # 0.01 est l'estimation initiale (10mm)
        h_sol = b_sol * ratio_h_b
        
        return b_sol, h_sol

    # --- CALCUL ET AFFICHAGE ---
    b, h = dimensionner_poutre(ratio_h_b=2.0)

    print(f"--- RÉSULTATS DU DIMENSIONNEMENT (Acier Sy={Sy/1e6} MPa, n={n}) ---")
    print(f"Largeur requise (b) : {b*1000:.2f} mm")
    print(f"Hauteur requise (h) : {h*1000:.2f} mm")
    print(f"Section : {b*1000:.1f} x {h*1000:.1f} mm")

    # Vérification finale
    A = b * h
    Z = (b * h**2) / 6
    print(f"\nVérification des contraintes :")
    print(f"Contrainte de flexion seule : {M_max/Z/1e6:.2f} MPa")
    print(f"Contrainte combinée Max (Von Mises) : {(M_max/Z + N_max/A)/1e6:.2f} MPa")
    print(f"Contrainte admissible (Sy/n) : {stress_adm/1e6:.2f} MPa")

    plt.show()


    # ### =======================================================================
    # ### ANALYSE SPATIALE : IMPACT SANS LA MASSE DE LA DEUXIÈME PARTIE DE POUTRE
    # ### =======================================================================
    # print("\nDébut de l'analyse spatiale (sans la masse de arm_part2)...")

    # # Paramètres de la poutre
    # L_tot = 1.0       
    # M_tot = 30.0      
    # num_points = 20   
    
    # x_positions = np.linspace(0.01, L_tot - 0.01, num_points)

    # max_normal  = np.zeros(num_points)
    # max_shear   = np.zeros(num_points)
    # max_bending = np.zeros(num_points)

    # time_max_normal  = np.zeros(num_points)
    # time_max_shear   = np.zeros(num_points)
    # time_max_bending = np.zeros(num_points)

    # history_normal  = np.zeros((num_points, len(time)))
    # history_shear   = np.zeros((num_points, len(time)))
    # history_bending = np.zeros((num_points, len(time)))

    # idx_8  = np.where(c_idx == 8)[0][0]  
    # idx_9  = np.where(c_idx == 9)[0][0]  
    # idx_10 = np.where(c_idx == 10)[0][0] 

    # import copy
    # topo_base = copy.deepcopy(topology)

    # # NOUVEAU DOSSIER pour ne pas écraser les données précédentes
    # _data_spatial_dir = os.path.join(_script_dir, "data_spatial_sans_m2")
    # os.makedirs(_data_spatial_dir, exist_ok=True)

    # for idx_x, x in enumerate(x_positions):
        
    #     # 1. Mise à jour de la topologie avec arm_part2 "fantôme" (masse = 0)
    #     m1 = M_tot * (x / L_tot)
    #     m2 = 0.0  # <--- LA MODIFICATION EST ICI : on annule la masse en aval
        
    #     topology["m"][7]  = m1  
    #     topology["m"][10] = m2  
        
    #     topology["d_ii"][7]  = np.array([0.0, -x / 2.0, 0.0]) 
    #     topology["d_ii"][10] = np.array([0.0, 0.0, 0.0]) # Centre de masse nul
        
    #     I1_val = (1.0 / 12.0) * m1 * x**2
    #     topology["I"][7]  = np.diag([I1_val, 0.0, I1_val]) 
    #     topology["I"][10] = np.zeros((3, 3)) # Inertie nulle
        
    #     topology["d_hi"][8] = np.array([0.0, -x, 0.0])

    #     # 2. Boucle temporelle (Idem)
    #     Q_c_x = np.zeros((len(time), len(c_idx)))
        
    #     for i, t in enumerate(time):
    #         q_full  = np.zeros(N + 1)
    #         qd_full = np.zeros(N + 1)
            
    #         q_full[u_idx]  = q_sol[i, :]
    #         qd_full[u_idx] = qd_sol[i, :]
            
    #         dk = driven_kinematics(t)
    #         qdd_c_driven = {}
    #         for j, (qv, qdv, qddv) in dk.items():
    #             q_full[j]       = qv
    #             qd_full[j]      = qdv
    #             qdd_c_driven[j] = qddv
                
    #         qdd_full_t, Q_c_t = physical_model(t, q_full, qd_full, u_idx, c_idx, qdd_c_driven)
    #         Q_c_x[i, :] = Q_c_t
            
    #     # 3. Historique
    #     history_normal[idx_x, :]  = Q_c_x[:, idx_8]
    #     history_shear[idx_x, :]   = Q_c_x[:, idx_9]
    #     history_bending[idx_x, :] = Q_c_x[:, idx_10]
        
    #     # 4. Maximums et temps associés
    #     i_max_n = np.argmax(np.abs(history_normal[idx_x, :]))
    #     i_max_s = np.argmax(np.abs(history_shear[idx_x, :]))
    #     i_max_b = np.argmax(np.abs(history_bending[idx_x, :]))
        
    #     max_normal[idx_x]  = np.abs(history_normal[idx_x, i_max_n])
    #     max_shear[idx_x]   = np.abs(history_shear[idx_x, i_max_s])
    #     max_bending[idx_x] = np.abs(history_bending[idx_x, i_max_b])
        
    #     time_max_normal[idx_x]  = time[i_max_n]
    #     time_max_shear[idx_x]   = time[i_max_s]
    #     time_max_bending[idx_x] = time[i_max_b]
        
    #     print(f"Coupe Y=-{x:.2f}m (sans m2) -> Max Normal: {max_normal[idx_x]:.0f}N | Flexion: {max_bending[idx_x]:.0f}Nm")

    #     # 5. Sauvegarde
    #     output_data = np.zeros((len(time), N + 1))
    #     output_data[:, 0] = time
    #     for k, j in enumerate(c_idx):
    #         output_data[:, j] = Q_c_x[:, k]
            
    #     file_name = f"Qc_driven_neri_x_{x:.2f}_sans_m2.txt"
    #     output_filepath = os.path.join(_data_spatial_dir, file_name)
    #     np.savetxt(output_filepath, output_data, fmt='%.6e', delimiter=' ')

    # topology = topo_base

    # print(f"\nToutes les données (sans m2) ont été sauvegardées dans : {_data_spatial_dir}")

    # # =========================================================================
    # # PLOTS : COMPARAISON (SANS m2)
    # # =========================================================================
    # import matplotlib.cm as cm
    # colors = cm.viridis(np.linspace(0, 1, num_points))

    # # PLOT TEMPOREL
    # fig_time, axs_time = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    # for idx_x, x in enumerate(x_positions):
    #     axs_time[0].plot(time, history_normal[idx_x, :], color=colors[idx_x], alpha=0.7)
    #     axs_time[1].plot(time, history_shear[idx_x, :], color=colors[idx_x], alpha=0.7)
    #     axs_time[2].plot(time, history_bending[idx_x, :], color=colors[idx_x], alpha=0.7)

    # axs_time[0].set_ylabel('Effort Normal (N)')
    # axs_time[0].set_title("Évolution temporelle SANS la masse de la poutre en aval")
    # axs_time[0].grid(True)
    # axs_time[1].set_ylabel('Effort Tranchant (N)')
    # axs_time[1].grid(True)
    # axs_time[2].set_ylabel('Moment Fléchissant (N.m)')
    # axs_time[2].set_xlabel('Temps [s]')
    # axs_time[2].grid(True)

    # fig_time.tight_layout()
    # save(fig_time, "09_efforts_temporels_sans_m2")

    # # PLOT ENVELOPPE SPATIALE AVEC TEMPS
    # fig_env, axs_env = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    # cmap_time = 'plasma'

    # axs_env[0].plot(x_positions, max_normal, color='gray', linestyle='--', alpha=0.5)
    # sc1 = axs_env[0].scatter(x_positions, max_normal, c=time_max_normal, cmap=cmap_time, zorder=5, s=60)
    # axs_env[0].set_ylabel('Normal Max (N)')
    # axs_env[0].set_title("Enveloppe maximaux SANS la masse de la poutre en aval (Couleur = Temps)")
    # axs_env[0].grid(True)
    # fig_env.colorbar(sc1, ax=axs_env[0], label='Temps (s)')

    # axs_env[1].plot(x_positions, max_shear, color='gray', linestyle='--', alpha=0.5)
    # sc2 = axs_env[1].scatter(x_positions, max_shear, c=time_max_shear, cmap=cmap_time, zorder=5, s=60)
    # axs_env[1].set_ylabel('Tranchant Max (N)')
    # axs_env[1].grid(True)
    # fig_env.colorbar(sc2, ax=axs_env[1], label='Temps (s)')

    # axs_env[2].plot(x_positions, max_bending, color='gray', linestyle='--', alpha=0.5)
    # sc3 = axs_env[2].scatter(x_positions, max_bending, c=time_max_bending, cmap=cmap_time, zorder=5, s=60)
    # axs_env[2].set_ylabel('Flexion Max (N.m)')
    # axs_env[2].set_xlabel('Position dans la poutre (Distance depuis la base) [m]')
    # axs_env[2].grid(True)
    # fig_env.colorbar(sc3, ax=axs_env[2], label='Temps (s)')

    # fig_env.tight_layout()
    # save(fig_env, "10_enveloppe_efforts_poutre_sans_m2")
    
    # plt.show()