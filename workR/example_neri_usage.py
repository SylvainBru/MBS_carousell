#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example: Complete NERi Solver Usage

This script demonstrates:
1. Loading the MBS system
2. Running NERi solver with RK4 integration
3. Comparing with Robotran reference
4. Computing stresses and dimensioning
5. Generating full analysis report
"""

import numpy as np
import os
import sys

# Add paths
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

try:
    import MBsysPy as Robotran
except ImportError:
    print("ERROR: MBsysPy not found. Please install Robotran.")
    sys.exit(1)

from NERi_solver import NERiSolver
from NERi_postprocessor import NERiPostProcessor
from NERi_stress_analysis import StructuralDimensioning, BeamMaterial


def main():
    """Main execution function."""
    
    print("\n" + "="*70)
    print("         NERi SOLVER - COMPLETE USAGE EXAMPLE")
    print("="*70)
    
    # =========================================================================
    # 1. LOAD MBS DATA
    # =========================================================================
    print("\n[1] Loading MBS data...")
    mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
    mbs_data = Robotran.MbsData(mbs_path)
    print(f"  ✓ Loaded: {os.path.basename(mbs_path)}")
    
    # =========================================================================
    # 2. PARTITION SYSTEM
    # =========================================================================
    print("\n[2] Partitioning system...")
    mbs_data.process = 1
    mbs_part = Robotran.MbsPart(mbs_data)
    mbs_part.set_options(rowperm=1, verbose=0)
    mbs_part.run()
    print(f"  ✓ Independent (qu): {mbs_data.nqu}")
    print(f"  ✓ Driven (qc): {mbs_data.nqc}")
    
    # =========================================================================
    # 3. RUN ROBOTRAN REFERENCE (optional - for comparison)
    # =========================================================================
    print("\n[3] Running Robotran reference simulation...")
    print("  (This may take a moment...)")
    mbs_data.process = 3
    mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
    mbs_dirdyn.set_options(dt0=1e-3, tf=3.0, save2file=1)
    robotran_results = mbs_dirdyn.run()
    print(f"  ✓ Completed: {len(robotran_results.t)} time steps")
    
    # =========================================================================
    # 4. RUN NERi SOLVER
    # =========================================================================
    print("\n[4] Running NERi solver...")
    
    # Create solver
    neri_solver = NERiSolver(mbs_data)
    
    # Set initial conditions
    initial_conditions = {
        'qu': mbs_data.qu[1:mbs_data.nqu+1],
        'qdu': np.zeros(mbs_data.nqu),
        'qdc': np.zeros(mbs_data.nqc)
    }
    
    # Run integration
    neri_results = neri_solver.integrate(
        t0=0.0,
        tf=3.0,
        dt=1e-3,
        initial_conditions=initial_conditions
    )
    print(f"  ✓ Completed: {len(neri_results['t'])} time steps")
    
    # =========================================================================
    # 5. COMPARE RESULTS
    # =========================================================================
    print("\n[5] Comparing results...")
    post_proc = NERiPostProcessor(mbs_data.nqu, mbs_data.nqc)
    stats = post_proc.summary_statistics(neri_results, robotran_results)
    
    print(f"\n  Position Statistics (radians):")
    print(f"    NERi max: {stats['neri']['q_max']:.6f}")
    print(f"    Robotran max: {stats['robotran']['q_max']:.6f}")
    print(f"    Difference: {abs(stats['neri']['q_max'] - stats['robotran']['q_max']):.6f}")
    
    if 'comparison' in stats:
        print(f"\n  Comparison:")
        if 'rmse_q' in stats['comparison']:
            print(f"    RMSE: {stats['comparison']['rmse_q']:.6f} rad")
            print(f"    Max error: {stats['comparison']['max_error_q']:.6f} rad")
    
    # =========================================================================
    # 6. COMPUTE STRESSES
    # =========================================================================
    print("\n[6] Computing internal stresses...")
    stresses = neri_solver.compute_stresses(safety_factor=10.0)
    
    # =========================================================================
    # 7. STRUCTURAL DIMENSIONING
    # =========================================================================
    print("\n[7] Structural dimensioning...")
    
    # Extract design loads from simulation
    if stresses['max_forces']:
        max_force = stresses['max_forces']['max']
        print(f"  Maximum force: {max_force:.2f} N")
    else:
        max_force = 100.0
    
    # Assume some torque (from geometry/control)
    max_torque = 50.0  # Placeholder (N·m)
    max_radial_load = 1000.0  # Placeholder (N)
    
    # Create dimensioning tool
    dimensioner = StructuralDimensioning(
        safety_factor=10.0,
        material=BeamMaterial.STEEL
    )
    
    # Generate report
    diagram_report = dimensioner.generate_report(
        max_torque=max_torque,
        max_force=max_force,
        max_radial_load=max_radial_load
    )
    
    # Print report
    dimensioner.print_report(diagram_report)
    
    # =========================================================================
    # 8. GENERATE PLOTS
    # =========================================================================
    print("\n[8] Generating plots...")
    
    results_dir = os.path.join(script_dir, '..', 'resultsR', 'NERi_example')
    os.makedirs(results_dir, exist_ok=True)
    
    # Plot results
    fig1 = post_proc.plot_results(
        neri_results, 
        robotran_results,
        save_path=os.path.join(results_dir, 'results_comparison.png')
    )
    
    # Plot errors
    fig2 = post_proc.plot_error_analysis(
        neri_results,
        robotran_results,
        save_path=os.path.join(results_dir, 'error_analysis.png')
    )
    
    # Plot energy
    fig3 = post_proc.plot_energy_analysis(
        neri_results,
        mbs_data,
        save_path=os.path.join(results_dir, 'energy_analysis.png')
    )
    
    # Plot forces
    if neri_results['forces']:
        fig4 = post_proc.plot_forces(
            neri_results['forces'],
            save_path=os.path.join(results_dir, 'internal_forces.png')
        )
    
    print(f"  ✓ Plots saved to: {results_dir}")
    
    # =========================================================================
    # 9. SUMMARY
    # =========================================================================
    print("\n" + "="*70)
    print("                    SUMMARY REPORT")
    print("="*70)
    print(f"\n✓ Simulation successfully completed!")
    print(f"\n  Simulation time: 3.0 seconds")
    print(f"  Time steps: {len(neri_results['t'])}")
    print(f"  Integration method: RK45")
    print(f"  Safety factor: 10.0")
    print(f"\n  Output directory: {results_dir}")
    print(f"  Generated files:")
    print(f"    - results_comparison.png")
    print(f"    - error_analysis.png")
    print(f"    - energy_analysis.png")
    print(f"    - internal_forces.png")
    
    print("\n" + "="*70 + "\n")
    
    return neri_results, robotran_results, stresses


if __name__ == '__main__':
    try:
        neri_res, robotran_res, stresses = main()
        
        # Try to show plots
        try:
            import matplotlib.pyplot as plt
            plt.show()
        except:
            print("Note: Could not display plots (matplotlib not available)")
            
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
