import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

_script_dir = os.path.dirname(os.path.abspath(__file__))

neri_data   = np.load(os.path.join(_script_dir, 'neri_results.npy'))
dirdyn_data = np.load(os.path.join(_script_dir, 'dirdyn_results.npy'))

t_neri,   qd_neri   = neri_data[:, 0],   neri_data[:, 1]
t_dirdyn, qd_dirdyn = dirdyn_data[:, 0], dirdyn_data[:, 1]

# Interpolation dirdyn sur la grille NERi
qd_dirdyn_i = interp1d(t_dirdyn, qd_dirdyn, bounds_error=False, fill_value='extrapolate')(t_neri)
diff = qd_neri - qd_dirdyn_i

# ── Plot 1 : superposition ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))
ax.plot(t_neri,   qd_neri,   label='NERi',     linewidth=1.5)
ax.plot(t_dirdyn, qd_dirdyn, label='Robotran', linewidth=1.5, linestyle='--')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Angular velocity [deg/s]')
ax.grid(True)
ax.legend()
fig.suptitle('Main pole angular velocity — NERi vs Robotran')
fig.tight_layout()
fig.savefig(os.path.join(_script_dir, '01_overlay.png'), dpi=150)
plt.close(fig)

# ── Plot 2 : différence NERi − Robotran ──────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))
ax.plot(t_neri, diff, color='tab:red', linewidth=1.2)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Δ Angular velocity [deg/s]')
ax.grid(True)
fig.suptitle('Difference NERi − Robotran (main pole velocity)')
fig.tight_layout()
fig.savefig(os.path.join(_script_dir, '02_difference.png'), dpi=150)
plt.close(fig)

print(f"Max |diff| = {np.max(np.abs(diff)):.4f} deg/s")
plt.show()
