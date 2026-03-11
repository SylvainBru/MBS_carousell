#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NERi Solver Validation and Comparison with Robotran
=====================================================

This script demonstrates the custom NERi (Non-Euclidean Rigid dynamics) solver
implementation for a merry-go-round carousel system.

Workflow:
1. Load MBS data from MBsysPad
2. Run reference simulation with Robotran's built-in solver
3. Run custom NERi solver with RK4 integration
4. Compare results and validate
5. Compute internal forces and stresses
6. Perform dimensioning with safety factor

Universite catholique de Louvain
CEREM : Centre for research in mechatronics

(c) UCLouvain
"""

import numpy as np
import os
import sys

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# Import custom NERi solver
from NERi_solver import NERiSolver
from NERi_postprocessor import NERiPostProcessor

# %%===========================================================================
# Project loading
# =============================================================================
script_dir = os.path.dirname(os.path.abspath(__file__))
mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
mbs_data = Robotran.MbsData(mbs_path)

print("\n" + "="*70)
print("NERi SOLVER VALIDATION AND COMPARISON WITH ROBOTRAN")
print("="*70)

# %%===========================================================================
# Partitioning
# =============================================================================
print("\n[1/5] Partitioning the system...")
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=0)
mbs_part.run()
print(f"  ✓ System partitioned successfully")
print(f"    - Independent (qu): {mbs_data.nqu}")
print(f"    - Driven (qc): {mbs_data.nqc}")




# %%===========================================================================
# Reference: Robotran Direct Dynamics (for validation)
# =============================================================================
print("\n[2/5] Running ROBOTRAN reference simulation...")
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=3, save2file=1)
robotran_results = mbs_dirdyn.run()
print(f"  ✓ Robotran simulation completed")
print(f"    - Duration: {mbs_dirdyn.get_options('tf')}s")
print(f"    - Time steps: {len(robotran_results.t)}")

# %%===========================================================================
# NERi Solver: Custom Implementation
# =============================================================================
print("\n[3/5] Running NERi CUSTOM SOLVER...")

# Create NERi solver instance
neri_solver = NERiSolver(mbs_data)

# Define initial conditions
initial_conditions = {
    'qu': mbs_data.qu[1:mbs_data.nqu+1],      # Initial independent coordinates
    'qdu': np.zeros(mbs_data.nqu),             # Initial independent velocities
    'qdc': np.zeros(mbs_data.nqc)              # Initial driven velocities
}

# Integration parameters
t0 = 0.0
tf = 3.0
dt = 1e-3

# Run NERi integration
neri_results = neri_solver.integrate(t0, tf, dt, initial_conditions)
print(f"  ✓ NERi simulation completed")
print(f"    - Method: RK45")
print(f"    - Time steps: {len(neri_results['t'])}")

# %%===========================================================================
# Post-processing and Comparison
# =============================================================================
print("\n[4/5] Post-processing and comparison...")

# Create post-processor
post_proc = NERiPostProcessor(mbs_data.nqu, mbs_data.nqc)

# Compute statistics
stats = post_proc.summary_statistics(neri_results, robotran_results)
print(f"  ✓ Statistics computed:")
print(f"    - NERi max position: {np.rad2deg(stats['neri']['q_max']):.4f}°")
print(f"    - Robotran max position: {np.rad2deg(stats['robotran']['q_max']):.4f}°")
if 'rmse_q' in stats['comparison']:
    print(f"    - RMSE (position): {np.rad2deg(stats['comparison']['rmse_q']):.4f}°")
    print(f"    - Max error (position): {np.rad2deg(stats['comparison']['max_error_q']):.4f}°")













# %%===========================================================================
# Visualization
# =============================================================================
print("\n[5/5] Generating visualization plots...")

try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')

# Create results directory if needed
results_dir = os.path.join(script_dir, '..', 'resultsR', 'NERi_validation')
os.makedirs(results_dir, exist_ok=True)

# Plot 1: Results comparison
fig1 = post_proc.plot_results(neri_results, robotran_results,
                              save_path=os.path.join(results_dir, 
                                                     '01_results_comparison.png'))

# Plot 2: Error analysis
fig2 = post_proc.plot_error_analysis(neri_results, robotran_results,
                                     save_path=os.path.join(results_dir,
                                                            '02_error_analysis.png'))

# Plot 3: Energy analysis
fig3 = post_proc.plot_energy_analysis(neri_results, mbs_data,
                                      save_path=os.path.join(results_dir,
                                                             '03_energy_analysis.png'))

# Plot 4: Internal forces
if neri_results['forces']:
    fig4 = post_proc.plot_forces(neri_results['forces'],
                                 save_path=os.path.join(results_dir,
                                                        '04_internal_forces.png'))

print(f"  ✓ Visualizations saved to: {results_dir}")

# %%===========================================================================
# Internal Stresses and Dimensioning
# =============================================================================
print("\n" + "="*70)
print("INTERNAL STRESSES AND DIMENSIONING (Safety Factor = 10)")
print("="*70)

# Compute stresses
stresses = neri_solver.compute_stresses(safety_factor=10.0)

# %%===========================================================================
# Summary Report
# =============================================================================
print("\n" + "="*70)
print("SIMULATION SUMMARY REPORT")
print("="*70)

print("\n📊 SOLVER COMPARISON:")
print(f"  Robotran:")
print(f"    - Duration: {len(robotran_results.t)} steps")
print(f"    - Time range: {robotran_results.t[0]:.3f}s - {robotran_results.t[-1]:.3f}s")
print(f"  NERi:")
print(f"    - Duration: {len(neri_results['t'])} steps")
print(f"    - Time range: {neri_results['t'][0]:.3f}s - {neri_results['t'][-1]:.3f}s")

print(f"\n🔧 COORDINATE INFO:")
print(f"  - Independent (qu): {mbs_data.nqu}")
print(f"  - Driven (qc): {mbs_data.nqc}")
print(f"  - Total: {mbs_data.nqu + mbs_data.nqc}")

print(f"\n📈 VALIDATION METRICS:")
if 'comparison' in stats and 'rmse_q' in stats['comparison']:
    print(f"  - Position RMSE: {np.rad2deg(stats['comparison']['rmse_q']):.4f}°")
    print(f"  - Maximum position error: {np.rad2deg(stats['comparison']['max_error_q']):.4f}°")
    
    # Compute acceptable error threshold (< 1%)
    acceptable_error = np.rad2deg(stats['robotran']['q_max']) * 0.01
    if stats['comparison']['rmse_q'] < np.deg2rad(acceptable_error):
        print(f"  ✓ VALIDATION PASSED (error < 1%)")
    else:
        print(f"  ⚠ VALIDATION WARNING (error > 1%)")

print(f"\n💾 OUTPUT FILES:")
print(f"  Results directory: {results_dir}")
print(f"  - 01_results_comparison.png: NERi vs Robotran comparison")
print(f"  - 02_error_analysis.png: Error analysis")
print(f"  - 03_energy_analysis.png: Energy conservation")
print(f"  - 04_internal_forces.png: Internal forces")

print("\n" + "="*70)
print("✓ SIMULATION COMPLETE")
print("="*70 + "\n")

# Show plots
plt.show()
