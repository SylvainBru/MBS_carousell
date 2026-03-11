#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Post-processing and visualization module for NERi solver results.

Provides:
1. Plotting coordinate trajectories
2. Comparing NERi vs Robotran results  
3. Energy analysis
4. Constraint violations analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from typing import Dict, List, Optional, Tuple


class NERiPostProcessor:
    """Handle visualization and analysis of NERi simulation results."""
    
    def __init__(self, nqu: int, nqc: int):
        """
        Initialize post-processor.
        
        Parameters
        ----------
        nqu : int
            Number of independent coordinates
        nqc : int
            Number of driven coordinates
        """
        self.nqu = nqu
        self.nqc = nqc
        self.nq = nqu + nqc
        
    def plot_results(self, neri_results: Dict, robotran_results: Optional[Dict] = None,
                    save_path: Optional[str] = None):
        """
        Plot simulation results with optional comparison to Robotran.
        
        Parameters
        ----------
        neri_results : Dict
            Results from NERi solver with keys: t, q, qd, qdd
        robotran_results : Dict, optional
            Results from Robotran solver for comparison
        save_path : str, optional
            Path to save figure
        """
        t = neri_results['t']
        q_neri = neri_results['q']
        qd_neri = neri_results['qd']
        qdd_neri = neri_results['qdd']
        
        # Create figure with multiple subplots
        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # ===== Plot Independent Coordinates =====
        for i in range(min(3, self.nqu)):
            ax = fig.add_subplot(gs[0, i])
            ax.plot(t, np.rad2deg(q_neri[:, i+1]), 'b-', label='NERi', linewidth=2)
            if robotran_results is not None:
                ax.plot(robotran_results['t'], np.rad2deg(robotran_results['q'][:, i+1]),
                       'r--', label='Robotran', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'q₍ᵤ₎{i+1} (deg)')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.set_title(f'Independent Coordinate {i+1}')
        
        # ===== Plot Independent Velocities =====
        for i in range(min(3, self.nqu)):
            ax = fig.add_subplot(gs[1, i])
            ax.plot(t, np.rad2deg(qd_neri[:, i+1]), 'b-', label='NERi', linewidth=2)
            if robotran_results is not None:
                ax.plot(robotran_results['t'], np.rad2deg(robotran_results['qd'][:, i+1]),
                       'r--', label='Robotran', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'q̇₍ᵤ₎{i+1} (deg/s)')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.set_title(f'Independent Velocity {i+1}')
        
        # ===== Plot Independent Accelerations =====
        for i in range(min(3, self.nqu)):
            ax = fig.add_subplot(gs[2, i])
            ax.plot(t, np.rad2deg(qdd_neri[:, i+1]), 'b-', label='NERi', linewidth=2)
            if robotran_results is not None:
                ax.plot(robotran_results['t'], np.rad2deg(robotran_results['qdd'][:, i+1]),
                       'r--', label='Robotran', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'q̈₍ᵤ₎{i+1} (deg/s²)')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.set_title(f'Independent Acceleration {i+1}')
        
        # ===== Plot Driven Coordinates (if any) =====
        if self.nqc > 0:
            for i in range(min(3, self.nqc)):
                ax = fig.add_subplot(gs[3, i])
                idx = self.nqu + i + 1
                ax.plot(t, np.rad2deg(qd_neri[:, idx]), 'g-', label='Driven', linewidth=2)
                if robotran_results is not None:
                    ax.plot(robotran_results['t'], np.rad2deg(robotran_results['qd'][:, idx]),
                           'r--', label='Robotran', linewidth=1.5)
                ax.set_xlabel('Time (s)')
                ax.set_ylabel(f'q̇₍c₎{i+1} (deg/s)')
                ax.grid(True, alpha=0.3)
                ax.legend()
                ax.set_title(f'Driven Velocity {i+1}')
        
        plt.suptitle('NERi Solver Results Comparison', fontsize=16, fontweight='bold')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Figure saved to {save_path}")
        
        return fig
    
    def plot_error_analysis(self, neri_results: Dict, robotran_results: Dict,
                           save_path: Optional[str] = None):
        """
        Plot error analysis between NERi and Robotran results.
        
        Parameters
        ----------
        neri_results : Dict
            Results from NERi solver
        robotran_results : Dict
            Results from Robotran solver
        save_path : str, optional
            Path to save figure
        """
        t_neri = neri_results['t']
        t_robotran = robotran_results['t']
        
        # Use NERi time grid
        t = t_neri
        
        # Interpolate Robotran results to NERi time grid
        q_robotran_interp = np.zeros_like(neri_results['q'])
        qd_robotran_interp = np.zeros_like(neri_results['qd'])
        
        for i in range(self.nq + 1):
            q_robotran_interp[:, i] = np.interp(t, t_robotran, 
                                                robotran_results['q'][:, i])
            qd_robotran_interp[:, i] = np.interp(t, t_robotran, 
                                                 robotran_results['qd'][:, i])
        
        # Compute errors
        q_error = neri_results['q'] - q_robotran_interp
        qd_error = neri_results['qd'] - qd_robotran_interp
        
        # Create figure
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        # Plot coordinate errors
        ax = axes[0]
        for i in range(1, min(4, self.nqu + 1)):
            ax.plot(t, np.rad2deg(q_error[:, i]), label=f'q_{i}', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (deg)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Position Error: NERi vs Robotran')
        
        # Plot velocity errors
        ax = axes[1]
        for i in range(1, min(4, self.nqu + 1)):
            ax.plot(t, np.rad2deg(qd_error[:, i]), label=f'ḋ_{i}', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity Error (deg/s)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Velocity Error: NERi vs Robotran')
        
        # Plot RMS errors over time
        ax = axes[2]
        pos_rms_error = np.sqrt(np.mean(q_error[:, 1:]**2, axis=1))
        vel_rms_error = np.sqrt(np.mean(qd_error[:, 1:]**2, axis=1))
        
        ax.plot(t, np.rad2deg(pos_rms_error), 'b-', label='Position RMS', linewidth=2)
        ax.plot(t, np.rad2deg(vel_rms_error), 'r-', label='Velocity RMS', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('RMS Error (deg or deg/s)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('RMS Error Over Time')
        
        plt.suptitle('Error Analysis: NERi vs Robotran', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Error analysis saved to {save_path}")
        
        return fig
    
    def plot_energy_analysis(self, neri_results: Dict, mbs_data,
                            save_path: Optional[str] = None):
        """
        Plot energy analysis (kinetic, potential, total).
        
        Parameters
        ----------
        neri_results : Dict
            Results from NERi solver
        mbs_data : MBsysPy.MbsData
            MBS data object for mass information
        save_path : str, optional
            Path to save figure
        """
        t = neri_results['t']
        q = neri_results['q']
        qd = neri_results['qd']
        
        # Compute kinetic energy: KE = 0.5 * m * v^2 (simplified)
        # In rotating systems: KE = 0.5 * I * ω^2
        
        # Get inertias from mbs_data
        if hasattr(mbs_data, 'l') and hasattr(mbs_data, 'm'):
            inertias = mbs_data.l[2, 1:]  # Izz for each body
            masses = mbs_data.m[1:]
        else:
            inertias = np.ones(self.nq) * 1.0
            masses = np.ones(self.nq) * 1.0
        
        KE = np.zeros(len(t))
        PE = np.zeros(len(t))
        
        for i in range(len(t)):
            # Kinetic energy (rotational for each independent coord)
            for j in range(1, min(len(inertias)+1, self.nqu+1)):
                KE[i] += 0.5 * inertias[j-1] * qd[i, j]**2
            
            # Potential energy (height-based, simplified)
            if hasattr(mbs_data, 'g'):
                g = np.linalg.norm(mbs_data.g)
                PE[i] = g * np.sum(masses * np.sin(q[i, 1:len(masses)+1]))
        
        TE = KE + PE
        
        # Create figure
        fig, axes = plt.subplots(2, 1, figsize=(14, 8))
        
        # Plot energies
        ax = axes[0]
        ax.plot(t, KE, 'b-', label='Kinetic Energy', linewidth=2)
        ax.plot(t, PE, 'r-', label='Potential Energy', linewidth=2)
        ax.plot(t, TE, 'g-', label='Total Energy', linewidth=2, linestyle='--')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Energy (J)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Energy Evolution')
        
        # Plot energy conservation error
        ax = axes[1]
        energy_error = (TE - TE[0]) / (np.max(TE) - np.min(TE) + 1e-10)
        ax.plot(t, energy_error * 100, 'purple', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Energy Error (%)')
        ax.grid(True, alpha=0.3)
        ax.set_title('Energy Conservation Error')
        
        plt.suptitle('Energy Analysis', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Energy analysis saved to {save_path}")
        
        return fig
    
    def plot_forces(self, forces_history: List[Dict], save_path: Optional[str] = None):
        """
        Plot internal forces (link forces).
        
        Parameters
        ----------
        forces_history : List[Dict]
            History of internal forces
        save_path : str, optional
            Path to save figure
        """
        if not forces_history:
            print("No forces to plot")
            return None
        
        times = np.array([f['t'] for f in forces_history])
        flinks = np.array([f['Flink'] for f in forces_history])
        
        fig, axes = plt.subplots(2, 1, figsize=(14, 8))
        
        # Plot link forces
        ax = axes[0]
        for i in range(min(4, flinks.shape[1])):
            ax.plot(times, flinks[:, i], label=f'Link {i+1}', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Force (N)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Internal Link Forces')
        
        # Plot link force derivatives (acceleration-like)
        ax = axes[1]
        flink_acc = np.gradient(flinks, times, axis=0)
        for i in range(min(4, flinks.shape[1])):
            ax.plot(times, flink_acc[:, i], label=f'Link {i+1}', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Force Rate (N/s)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Link Force Rate of Change')
        
        plt.suptitle('Internal Forces Analysis', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Forces analysis saved to {save_path}")
        
        return fig
    
    def summary_statistics(self, neri_results: Dict, 
                          robotran_results: Optional[Dict] = None) -> Dict:
        """
        Compute summary statistics.
        
        Parameters
        ----------
        neri_results : Dict
            Results from NERi solver
        robotran_results : Dict, optional
            Results from Robotran solver for comparison
            
        Returns
        -------
        Dict
            Summary statistics
        """
        stats = {
            'neri': {},
            'robotran': {},
            'comparison': {}
        }
        
        # NERi statistics
        q_neri = neri_results['q']
        qd_neri = neri_results['qd']
        
        stats['neri']['q_max'] = np.max(np.abs(q_neri[:, 1:]))
        stats['neri']['q_mean'] = np.mean(np.abs(q_neri[:, 1:]))
        stats['neri']['qd_max'] = np.max(np.abs(qd_neri[:, 1:]))
        stats['neri']['qd_mean'] = np.mean(np.abs(qd_neri[:, 1:]))
        
        # Robotran statistics
        if robotran_results is not None:
            q_robotran = robotran_results['q']
            qd_robotran = robotran_results['qd']
            
            stats['robotran']['q_max'] = np.max(np.abs(q_robotran[:, 1:]))
            stats['robotran']['q_mean'] = np.mean(np.abs(q_robotran[:, 1:]))
            stats['robotran']['qd_max'] = np.max(np.abs(qd_robotran[:, 1:]))
            stats['robotran']['qd_mean'] = np.mean(np.abs(qd_robotran[:, 1:]))
            
            # Comparison errors
            # Interpolate both to common grid
            t_neri = neri_results['t']
            t_robotran = robotran_results['t']
            t_common = np.linspace(max(t_neri[0], t_robotran[0]), 
                                   min(t_neri[-1], t_robotran[-1]), 1000)
            
            q_neri_common = np.zeros((len(t_common), q_neri.shape[1]))
            q_robotran_common = np.zeros((len(t_common), q_neri.shape[1]))
            
            for i in range(q_neri.shape[1]):
                q_neri_common[:, i] = np.interp(t_common, t_neri, q_neri[:, i])
                q_robotran_common[:, i] = np.interp(t_common, t_robotran, q_robotran[:, i])
            
            q_diff = q_neri_common - q_robotran_common
            stats['comparison']['rmse_q'] = np.sqrt(np.mean(q_diff[:, 1:]**2))
            stats['comparison']['max_error_q'] = np.max(np.abs(q_diff[:, 1:]))
        
        return stats
