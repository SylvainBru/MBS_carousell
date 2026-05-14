import os
import numpy as np
import MBsysPy as Robotran
from scipy import integrate
import matplotlib.pyplot as plt


import neri

# Chargement du fichier .mbs 
_script_dir = os.path.dirname(os.path.abspath(__file__))
_mbs_path   = os.path.join(_script_dir, '..', 'dataR', 'Merry_go_round.mbs')
_mbs_data   = Robotran.MbsData(_mbs_path)

# Topologie
N = 19

inbody = np.array(
    [None, 0, 1, 2, 3, 4, 5, 3, 7, 8, 9, 10, 11, 12, 3, 14, 15, 3, 17, 18],
    dtype=object)

fi_arr = np.array([
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
    "phi":    fi_arr,
    "psi":    psi_arr,
    "d_hi":   d_hi,
    "m":      m_arr,
    "d_ii":   d_ii,
    "I":      I_arr,
}

# couple moteur
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
    print("u (libres)    =", u_idx)
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

        qdd_full, _ = physical_model(t, q_full, qd_full, u_idx, c_idx, qdd_c_driven)
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

    _comp_dir = os.path.join(_script_dir, '..', 'comparaison_neri_vs_dirdyn')
    os.makedirs(_comp_dir, exist_ok=True)
    np.save(os.path.join(_comp_dir, 'neri_results.npy'), np.column_stack((time, qd1_deg)))

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
    ax.grid(True)
    ax.legend()
    fig.suptitle('NERi')
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
    ax.grid(True)
    ax.legend()
    fig.suptitle('NERi')
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
    ax.grid(True)
    ax.legend()
    fig.suptitle('NERi')
    fig.tight_layout()
    save(fig, "03_pendulum_velocities")

    # ── Plot 4 : joints commandés (inclinaison pôle) ──────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, np.rad2deg(q2_t), label='q2 — R1_Pole')
    ax.plot(time, np.rad2deg(q3_t), label='q3 — R2_Pole')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.grid(True)
    ax.legend()
    fig.suptitle('NERi')
    fig.tight_layout()
    save(fig, "04_driven_joints")

    plt.show()
