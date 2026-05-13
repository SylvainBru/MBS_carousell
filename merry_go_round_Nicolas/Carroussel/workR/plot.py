import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":

    q = np.loadtxt('Carroussel/resultsR/dirdyn_q.res')
    qd = np.loadtxt('Carroussel/resultsR/dirdyn_qd.res')
    qdd = np.loadtxt('Carroussel/resultsR/dirdyn_qdd.res')
    Q = np.loadtxt('Carroussel/resultsR/dirdyn_Qq.res')

    my_q = np.loadtxt('results_q.res')
    my_qd = np.loadtxt('results_qd.res')
    my_qdd = np.loadtxt('results_qdd.res')

    t = q[:,0]

    fig, ax = plt.subplots()
    ax.plot(t,Q[:,1],label='Torque')
    ax.set_xlabel('Time [s]')
    ax.set_title('Torque applied to the system')
    ax.legend()
    ax.grid()
    plt.show()

    fig, axx = plt.subplots(3,1,sharex=True)

    axx[0].plot(t,q[:,4],label='q')
    axx[0].plot(t,my_q[:,4],label='q (ours)', linestyle='dashed')
    axx[1].plot(t,qd[:,4],label='qd')
    axx[1].plot(t,my_qd[:,4],label='qd (ours)', linestyle='dashed')
    axx[2].plot(t,qdd[:,4],label='qdd')
    axx[2].plot(t,my_qdd[:,4],label='qdd (ours)', linestyle='dashed')

    axx[2].set_xlabel('Time [s]')
    axx[0].set_ylabel(r'q [rad]')
    axx[1].set_ylabel(r'$q_d$ [rad/s]')
    axx[2].set_ylabel(r'$q_{dd}$ [rad/s²]')
    axx[0].set_title('Direct dynamics results')
    axx[0].legend()

    axx[0].grid(); axx[1].grid(); axx[2].grid()

    plt.tight_layout()
    plt.savefig('../../GoNoGoGraph.pdf')
    plt.show()


    

