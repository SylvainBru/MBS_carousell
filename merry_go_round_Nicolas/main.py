import numpy as np 
import matplotlib.pyplot as plt
import MBsysPy as Robotran 
import scipy.linalg as sc
from scipy import integrate

class MBSystem:

    def __init__(self):
        self.g = np.array([0, 0, -9.81]) # Gravity vector of the system
        self.nbody = 0 # Number of bodies in the system
        self.njoint = 0 # Number of joints in the system
        self.mass = np.array([]) # Mass of the bodies in the system
        self.com = np.array([]) # Center of mass of the bodies in the system
        self.inertia = np.array([]) # Inertia of the bodies in the system
        self.body_names = np.array([]) # Names of the bodies in the system
        self.inbody_vector = np.array([]) # Vector of the bodies in the system
        self.d = np.array([])
        self.joints = np.array([]) # Type of joints in the system
        self.q_c = np.array([]) # Configuration coordinates of the system
        self.q_f = np.array([]) # Actuated configuration coordinates of the system
        self.q_u = np.array([]) # Unactuated configuration coordinates of the system

        self.fi = np.array([])
        self.psi = np.array([])


def R(i,h,q):
    """Compute the rotation matrix from body h to body i."""
    global system
    
    if (system.inbody_vector[i] != h):
        raise ValueError("The body h is not the parent of body i.")
    
    if (system.joints[i] == "R1"):
        R = np.array([[1, 0, 0],
                      [0, np.cos(q[i]), -np.sin(q[i])],
                      [0, np.sin(q[i]), np.cos(q[i])]])
    elif (system.joints[i] == "R2"):
        R = np.array([[np.cos(q[i]), 0, np.sin(q[i])],
                      [0, 1, 0],
                      [-np.sin(q[i]), 0, np.cos(q[i])]])
    elif (system.joints[i] == "R3"):
        R = np.array([[np.cos(q[i]), -np.sin(q[i]), 0],
                      [np.sin(q[i]), np.cos(q[i]), 0],
                      [0, 0, 1]])
    elif (system.joints[i] == "T1" or system.joints[i] == "T2" or system.joints[i] == "T3"):
        R = np.eye(3)
    else:
        raise ValueError("The type of joint is not recognized.")
    
    return R.T

def fi(joint_type):
    """Compute the axis of rotation of the joint between body h and body i."""
    
    if (joint_type == "R1"):
        fi = np.array([1, 0, 0])
    elif (joint_type == "R2"):
        fi = np.array([0, 1, 0])
    elif (joint_type == "R3"):
        fi = np.array([0, 0, 1])
    elif (joint_type == "T1" or joint_type == "T2" or joint_type == "T3"):
        fi = np.array([0, 0, 0])
    else:
        raise ValueError("The type of joint is not recognized.")
    
    return fi

def psi(joint_type):
    """Compute the axis of translation of the joint between body h and body i."""
    
    if (joint_type == "R1" or joint_type == "R2" or joint_type == "R3"):
        psi = np.array([0, 0, 0])
    elif (joint_type == "T1"):
        psi = np.array([1, 0, 0])
    elif (joint_type == "T2"):
        psi = np.array([0, 1, 0])
    elif (joint_type == "T3"):
        psi = np.array([0, 0, 1])
    else:
        raise ValueError("The type of joint is not recognized.")

    return psi

def tilde(v):
    """Compute the skew-symmetric matrix of a vector v."""
    tilde = np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])
    return tilde

def physical_model(system,tsim,q,qd):
    """Compute the physical model of the system.
    Newton Euler Recursive Formalism
    Computes the acceleration of the system at the current time step
    """

    # 1. Forward kinematic recursion
    alpha_c = np.zeros((system.nbody+1, 3)) # Alpha vector of the system
    alpha_c[0] = - system.g # Alpha vector of the base of the system

    beta_c = np.zeros((system.nbody+1, 3, 3)) # Beta vector of the system
    beta_c[0] = np.zeros((3, 3)) # Beta vector of the base of the system

    w = np.zeros((system.nbody+1, 3)) # Angular velocity of the system
    wd = np.zeros((system.nbody+1, 3)) # Angular acceleration of the system

    O_M = np.zeros((system.nbody+1,system.nbody+1,3)) # 
    A_M = np.zeros((system.nbody+1,system.nbody+1,3)) # 

    for i in range(1,system.nbody+1):

        # C VECTOR CONTRIBUTION
        h = system.inbody_vector[i]
        R_ih = R(i, h, q)
        qd_i = qd[i]
        z_i = system.psi[i]*q[i]

        w[i] = R_ih@w[h] + system.fi[i]*qd_i
        wd[i] = R_ih@wd[h] + np.cross(w[i], system.fi[i]*qd_i)
        beta_c[i] = tilde(wd[i]) + tilde(w[i])@tilde(w[i])
    
        d = system.d[i]
        alpha_c[i] = R_ih@(alpha_c[h] + beta_c[h]@(d+z_i)) + 2*np.cross(w[i], system.psi[i]*qd_i)

        # MASS MATRIX CONTRIBUTION
        for k in range(1, i + 1):
            delta = 0
            if i == k:
                delta = 1
            O_M[i,k] = R_ih@O_M[h,k] + delta*system.fi[i]
            A_M[i,k] = R_ih@(A_M[h,k] + np.cross(O_M[h,k], d+z_i)) + delta*system.psi[i]
        
    # # 2. Backward dynamic recursion
    W_c = np.zeros((system.nbody+1, 3)) # W vector of the system
    F_c = np.zeros((system.nbody+1, 3)) # Force vector of the system
    L_c = np.zeros((system.nbody+1, 3)) # Torque vector of the system

    W_M = np.zeros((system.nbody+1,system.nbody+1,3))
    F_M = np.zeros((system.nbody+1,system.nbody+1,3))
    L_M = np.zeros((system.nbody+1,system.nbody+1,3))

    M = np.zeros((system.njoint+1, system.njoint+1))
    c = np.zeros(system.njoint+1)
    Q = np.zeros(system.nbody+1)
    qdd_c = np.zeros(system.nbody+1)

    for i in range(system.nbody, 0, -1):

        z_i = system.psi[i]*q[i]

        # C VECTOR CONTRIBUTION
        W_c[i] = system.mass[i]*(alpha_c[i] + beta_c[i]@(system.com[i]+z_i)) 
        F_c[i] = W_c[i].copy()
        L_c[i] = np.cross(system.com[i]+z_i, W_c[i]) + system.inertia[i]@wd[i] + np.cross(w[i], system.inertia[i]@w[i])

        for j in range(1, system.nbody+1):
            if (system.inbody_vector[j] == i):
                d = system.d[j]
                R_ij = R(j,i,q).T
                Fc_j = R_ij @ F_c[j]
                Lc_j = R_ij @ L_c[j]
                F_c[i] += Fc_j
                L_c[i] += Lc_j + np.cross(d+z_i, Fc_j)

        # MASS MATRIX CONTRIBUTION
        for k in range(1, i + 1):
            delta = 0
            if i == k:
                delta = 1
            W_M[i,k] = system.mass[i]*(A_M[i,k] + np.cross(O_M[i,k], system.com[i]+z_i))
            F_M[i,k] = W_M[i,k].copy()
            L_M[i,k] = np.cross(system.com[i]+z_i, W_M[i,k]) + system.inertia[i]@O_M[i,k]

            for l in range(1, system.nbody+1):
                if (system.inbody_vector[l] == i):
                    d = system.d[l]
                    R_il = R(l,i, q).T
                    F_M[i,k] += R_il@F_M[l,k]
                    L_M[i,k] += R_il@L_M[l,k] + np.cross(d+z_i, R_il@F_M[l,k])

        # C and Q VECTOR ASSEMBLY
        c[i] = np.dot(system.psi[i],F_c[i]) + np.dot(system.fi[i],L_c[i])

        F_ext = np.zeros(3)
        L_ext = np.zeros(3) 
        if(i in system.q_f):
            L_ext = 3000 * np.sin(2 * np.pi * tsim / 10) * system.fi[i]
        Q[i] = np.dot(system.psi[i],F_ext) + np.dot(system.fi[i],L_ext)

        if (i == 2):
            qdd_c[i] = -0.3 * (2 * np.pi / 10)**2 * np.sin(2 * np.pi * tsim / 10)

        # MASS MATRIX ASSEMBLY
        for j in range(1, system.njoint+1) : 
            M[i,j] += np.dot(system.psi[i],F_M[i,j]) + np.dot(system.fi[i],L_M[i,j])

    # Solve for the acceleration of the system
    # M qdd + c = Q
    M = M + M.T - np.diag(M.diagonal()) # Make M symmetric

    # # Verify that M is positive definite
    # if not np.all(np.linalg.eigvals(M[1:,1:]) > 0):
    #     raise ValueError("The mass matrix is not positive definite.")
    # # Verify that M is symmetric
    # if not np.allclose(M[1:,1:], M[1:,1:].T):
    #     raise ValueError("The mass matrix is not symmetric.")

    # Partitioning the system
    qu = np.concatenate((system.q_u, system.q_f))
    M_uu = M[np.ix_(qu, qu)]
    M_uc = M[np.ix_(qu, system.q_c)]
    c_u = c[qu]
    Q_u = Q[qu]

    qdd_c = qdd_c[system.q_c]

    try:
        cho   = sc.cho_factor(M_uu)
        qdd_u = sc.cho_solve(cho, Q_u - c_u - M_uc @ qdd_c)
    except np.linalg.LinAlgError:
        # Au cas où, mais normalement ça ne devrait pas arriver
        qdd_u = np.linalg.solve(M_uu, Q_u - c_u - M_uc @ qdd_c)

    qdd     = np.zeros(system.njoint+1)
    qdd[qu] = qdd_u
    qdd[system.q_c] = qdd_c
    
    # Find Q_c
    Q_c = np.zeros(len(system.q_c))
    M_cu = M[np.ix_(system.q_c, qu)]
    M_cc = M[np.ix_(system.q_c, system.q_c)]
    c_c = c[system.q_c]
    Q_c = M_cu @ qdd_u + M_cc @ qdd_c + c_c

    print(f"Time {tsim:.3f} s", end="\r")

    return qdd, Q_c


if __name__ == "__main__":

    # 1. Importing the multibody system data from the file
    file_path = "Carroussel/dataR/Carroussel.mbs"
    mbs_data = Robotran.MbsData(file_path)

    system = MBSystem()

    system.g = mbs_data.g[1:] # Gravity vector of the system
    system.mass = mbs_data.m # Mass of the bodies in the system
    system.nbody = mbs_data.nbody # Number of bodies in the system
    system.njoint = mbs_data.njoint # Number of joints in the system
    system.com = mbs_data.l[1:4].T # Center of mass of the bodies in the system
    system.inertia = mbs_data.In[1:10].T # Inertia of the bodies in the system
    # Unravel each inertia vector into a 3x3 inertia matrix
    system.inertia = np.reshape(system.inertia, (-1,3, 3), order='F')

    system.body_names = np.array(list(mbs_data.body_id.keys())) # Names of the bodies in the system
    system.inbody_vector = np.array([-1,0,1,2,3,4,2,6,7,2,9,10,2,12,13]) # Vector of the bodies in the system
    #                                 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14

    system.d = np.array([(0,0,0),(0,0,0),(0,0,4),(0,-0.4,0),(0,-1,0),(0,0,-3),
                         # 0        1        2        3         4         5
                         (0,0.4,0),(0,1,0),(0,0,-3),(0.4,0,0),(1,0,0),(0,0,-3),
                         # 6        7        8        9         10        11
                         (-0.4,0,0),(-1,0,0),(0,0,-3)])
                         # 12       13        14
    
    
    system.joints = np.array(["","R3","T3","R1","R1","R1","R1","R1","R1","R2","R2","R2","R2","R2","R2"]) # Type of joints in the system
    #                         0    1    2    3    4    5    6    7    8    9   10   11   12   13   14

    system.fi = np.zeros((system.njoint+1, 3))
    system.psi = np.zeros((system.njoint+1, 3))
    for i in range(1, system.njoint+1):
        system.fi[i] = fi(system.joints[i])
        system.psi[i] = psi(system.joints[i])

    system.q_c = np.trim_zeros(np.array(mbs_data.qc[1:])) # Given q,qd,qdd
    system.q_f = np.array([1]) # Given Q
    system.q_u = np.array([i for i in range(1, system.njoint+1) if ( i not in system.q_c and i not in system.q_f )])

    t0 = mbs_data.t0 # Initial time of the simulation
    tf = 10 # Final time of the simulation
    dt = 0.001 # Time step of the simulation

    # Initialize arrays
    time = np.arange(t0, tf+dt, dt)
    q = np.zeros((len(time), system.njoint+1))
    qd = np.zeros((len(time), system.njoint+1))
    qdd = np.zeros((len(time), system.njoint+1))
    Q = np.zeros((len(time), system.njoint+1))

    # Set initial conditions
    q[0] = mbs_data.q0
    qd[0] = mbs_data.qd0
    qdd[0] = mbs_data.qdd0
    # Set initial conditions for the commanded joint
    q[0,2] = 0.3 * np.sin(2 * np.pi * t0 / 10)
    qd[0,2] = 0.3 * 2 * np.pi / 10 * np.cos(2 * np.pi * t0 / 10)
    qdd[0,2] = -0.3 * (2 * np.pi / 10)**2 * np.sin(2 * np.pi * t0 / 10)


    def fun(t, y):
        q = y[:system.njoint+1]
        qd = y[system.njoint+1:]
        qdd,_ = physical_model(system, t, q, qd)
        return np.concatenate((qd, qdd))
        
    # Integration using Runge-Kutta 4th order method
    sol = integrate.solve_ivp(fun, (t0, tf), np.concatenate((q[0], qd[0])), t_eval=time,dense_output=True,method='RK45')
    q = sol.y[:system.njoint+1].T
    qd = sol.y[system.njoint+1:].T
    qdd = np.zeros_like(q)
    Q_c = np.zeros_like(q)
    # for i in range(len(time)):
    #     qdd[i], Q_c_cut = physical_model(system, time[i], q[i], qd[i])
    #     Q_c[i,system.q_c] = Q_c_cut

    # Print results position 
    with open("results_q.res", "w") as f:
        for i in range(len(time)):
            f.write(f"{time[i]:.6e} " + " ".join([f"{q[i,j]:.6e}" for j in range(1, system.njoint+1)]) + "\n")
    # Print results velocity
    with open("results_qd.res", "w") as f:
        for i in range(len(time)):
            f.write(f"{time[i]:.6e} " + " ".join([f"{qd[i,j]:.6e}" for j in range(1, system.njoint+1)]) + "\n")
    # Print animation file
    with open("animation.anim", "w") as f:
        for i in range(len(time)):
            f.write(f"{time[i]:.3f} " + " ".join([f"{q[i,j]:.6e}" for j in range(1, system.njoint+1)]) + "\n")

    # Compute reference solution with q computed with the physical model
    mbs_q = np.loadtxt("Carroussel/resultsR/dirdyn_q.res")
    mbs_qd = np.loadtxt("Carroussel/resultsR/dirdyn_qd.res")
    mbs_qdd = np.loadtxt("Carroussel/resultsR/dirdyn_qdd.res")
    mbs_Q = np.loadtxt("Carroussel/resultsR/dirdyn_Qc.res")

    # Get error between my solution and the reference solution
    # error_q = np.linalg.norm(q[:,1:] - mbs_q[:,1:], axis=1)
    # error_qd = np.linalg.norm(qd[:,1:] - mbs_qd[:,1:], axis=1)
    # error_qdd = np.linalg.norm(qdd[:,1:] - mbs_qdd[:,1:], axis=1)

    # print(f"Max error in q: {np.max(error_q):.6e}")
    # print(f"Max error in qd: {np.max(error_qd):.6e}")
    # print(f"Max error in qdd: {np.max(error_qdd):.6e}")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1,3,figsize=(12, 8))

    q_plot = 2 # Joint to plot

    # PLot my solution
    ax[0].plot(time, q[:,q_plot], label='q[1]')
    ax[1].plot(time, qd[:,q_plot], label='qd[1]')
    ax[2].plot(time, qdd[:,q_plot], label='qdd[1]')

    # Plot reference solution
    ax[0].plot(mbs_q[:,0], mbs_q[:,q_plot], label='mbs q[1]', linestyle='dashed')
    ax[1].plot(mbs_qd[:,0], mbs_qd[:,q_plot], label='mbs qd[1]', linestyle='dashed')
    ax[2].plot(mbs_qdd[:,0], mbs_qdd[:,q_plot], label='mbs qdd[1]', linestyle='dashed')

    ax[0].set_xlabel('Time [s]')
    ax[1].set_xlabel('Time [s]')
    ax[2].set_xlabel('Time [s]')
    ax[0].set_ylabel('q [rad]')
    ax[1].set_ylabel('qd [rad/s]')
    ax[2].set_ylabel('qdd [rad/s²]')
    ax[0].set_title('Configuration of the system')
    ax[1].set_title('Velocity of the system')
    ax[2].set_title('Acceleration of the system')
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()
    plt.tight_layout()
    plt.savefig("comparison.pdf")
    plt.show()


    fig, ax = plt.subplots()

    ax.plot(time, Q[:,0], label='Q[2]')
    ax.plot(mbs_Q[:,0], mbs_Q[:,2], label='mbs Q[2]', linestyle='dashed')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Force [Nm]')
    ax.set_title('Force applied to the system')
    ax.legend()
    plt.tight_layout()
    plt.savefig("force_comparison.pdf")
    plt.show()


