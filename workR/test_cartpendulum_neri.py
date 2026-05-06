import os
import numpy as np
import MBsysPy as Robotran
import neri
import cartpendulum

script_dir = os.path.dirname(os.path.abspath(__file__))

# Achtung: dein Upload heißt eventuell "cartpendulum .mbs" mit Leerzeichen.
# Lokal bei dir wahrscheinlich: "cartpendulum.mbs" oder "Cartpendulum.mbs"
mbs_path = os.path.join(script_dir, "cartpendulum .mbs")

mbs_data = Robotran.MbsData(mbs_path)
#topology = neri.define_topology_from_mbs(mbs_path)
topology = neri.define_cartpendulum_reference_topology(mbs_data)


# Partitioning
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

u = np.array(mbs_data.qu[1:mbs_data.nqu + 1], dtype=int)
c = np.array(mbs_data.qc[1:mbs_data.nqc + 1], dtype=int)

q = np.array(mbs_data.q0, dtype=float).copy()
qd = np.array(mbs_data.qd0, dtype=float).copy()
qdd = np.zeros_like(q)

print("\n--- partition ---")
print("u =", u)
print("c =", c)
print("joint_id =", mbs_data.joint_id)
print("body_id =", mbs_data.body_id)

print("\n--- topology ---")
for i in range(1, mbs_data.njoint + 1):
    print(
        i,
        topology["joint_names"][i],
        "parent =", topology["inbody"][i],
        "phi =", topology["phi"][i],
        "psi =", topology["psi"][i],
        "d_hi =", topology["d_hi"][i],
        "m =", topology["m"][i],
        "d_ii =", topology["d_ii"][i],
    )

# NERi M and Q
M_neri, Q_neri = neri.ner_generique(q, qd, qdd, mbs_data, topology)

Muu_neri = M_neri[np.ix_(u, u)]
Qu_neri = Q_neri[u]

print("\n--- NERi result ---")
print("Muu_neri =")
print(Muu_neri)
print("Qu_neri =")
print(Qu_neri)

# Reference derivative from cartpendulum.py
j_cart = mbs_data.joint_id["Cart_T2"]
j_pend = mbs_data.joint_id["Pendulum_R1"]

y_ref = np.array([
    q[j_cart],
    q[j_pend],
    qd[j_cart],
    qd[j_pend],
], dtype=float)

yd_ref = cartpendulum.compute_derivatives(0.0, y_ref, cartpendulum.mbs_data)

# NERi acceleration from the same M and Q as printed above,
# plus the external cart force used in cartpendulum.py
Fmax = cartpendulum.mbs_data.user_model["Force"]["Fmax"]

Q_applied = np.zeros_like(Qu_neri)
Q_applied[0] = Fmax

qdd_u = np.linalg.solve(Muu_neri, Qu_neri + Q_applied)

qdd_ref = yd_ref[2:4]

print("\n--- force check ---")
print("Fmax =", Fmax)
print("Q_applied =", Q_applied)

qdd_ref = yd_ref[2:4]

print("\n--- acceleration comparison ---")
print("qdd_neri =", qdd_u)
print("qdd_ref  =", qdd_ref)
print("qdd error =", qdd_u - qdd_ref)
print("||qdd error|| =", np.linalg.norm(qdd_u - qdd_ref))
print("\n--- cartpendulum.py reference ---")
print("y_ref =", y_ref)
print("yd_ref = [qd1, qd2, qdd1, qdd2] =")
print(yd_ref)

print("\n--- NERi M/Q diagnostic ---")
print("Muu_neri =")
print(Muu_neri)
print("Qu_neri =")
print(Qu_neri)

print("\nNote:")
print("cartpendulum.py is a hand-written 2-DOF reference model.")
print("define_topology_from_mbs() is a general Robotran-body parser, so exact M/Q equality is not expected if the .mbs body structure differs from the hand model.")


print("\n--- c / commanded coordinate test in cartpendulum ---")

t_test = 2.0

q_test = q.copy()
qd_test = qd.copy()

qc, qdc, qddc = neri.commanded_motion(t_test, c)

q_test[c] = qc
qd_test[c] = qdc

qdd_zero = np.zeros_like(q_test)

M, Q = neri.ner_generique(q_test, qd_test, qdd_zero, mbs_data, topology)

Muu = M[np.ix_(u, u)]
Muc = M[np.ix_(u, c)]
Qu = Q[u]

# Manual reduced equation:
# Muu qdd_u + Muc qdd_c = Qu
# => Muu qdd_u = Qu - Muc qdd_c
rhs_manual = Qu - Muc @ qddc
qdd_u_manual = np.linalg.solve(Muu, rhs_manual)

# Same thing through compute_qdd_u()
qdd_u_func, Muu_func, Qu_func = neri.compute_qdd_u(
    q_test,
    qd_test,
    mbs_data,
    topology,
    u,
    c=c,
    qdd_c=qddc,
    t_current=t_test,
    motor_state=None,
    use_carousel_forces=False,
)

print("t_test =", t_test)
print("u =", u)
print("c =", c)
print("qc =", qc)
print("qdc =", qdc)
print("qddc =", qddc)

print("\nMuc =")
print(Muc)

print("\nmanual reduced qdd_u =", qdd_u_manual)
print("compute_qdd_u qdd_u =", qdd_u_func)

print("\ndifference =", qdd_u_func - qdd_u_manual)
print("||difference|| =", np.linalg.norm(qdd_u_func - qdd_u_manual))

qdd_without_correction = np.linalg.solve(Muu, Qu)
qdd_with_correction = np.linalg.solve(Muu, Qu - Muc @ qddc)

print("\nqdd_without_correction =", qdd_without_correction)
print("qdd_with_correction    =", qdd_with_correction)
print("effect of c correction =", qdd_with_correction - qdd_without_correction)