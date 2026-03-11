#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NERi (Non-Euclidean Rigid dynamics) Solver for Multibody Systems
Implements the NERi formalism with custom RK4 integration

This module handles:
1. Loading and managing MBS data
2. Partitioning independent vs driven coordinates
3. Computing mass matrix and constraint vector
4. Custom RK4 numerical integration
5. Computing internal forces and stresses

(c) UCLouvain - CEREM
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
from typing import Tuple, List, Dict, Callable
import sys
import os

# Import the generated symbolic functions
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'symbolicR'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'userfctR'))

from mbs_dirdyna_Merry_go_round import dirdyna
from mbs_link_Merry_go_round import link


class NERiSolver:
    """
    NERi Solver for multibody dynamics simulation.
    
    The NERi formalism separates coordinates into:
    - qu: independent generalized coordinates (to be integrated)
    - qc: driven/commanded coordinates (prescribed by actuators)
    
    Equation of motion structure:
        [M_uu  M_uc] [q̈_u]   [c_u]
        [M_cu  M_cc] [q̈_c] = [c_c]
    
    Where:
    - M: Mass matrix computed from kinematics
    - c: Constraint vector (external + internal forces)
    - q̈_u: Independent accelerations (computed)
    - q̈_c: Driven accelerations (prescribed)
    """
    
    def __init__(self, mbs_data):
        """
        Initialize the NERi solver with MBS data.
        
        Parameters
        ----------
        mbs_data : MBsysPy.MbsData
            The loaded multibody system data
        """
        self.mbs_data = mbs_data
        
        # Extract coordinate information
        self.nqu = mbs_data.nqu  # Number of independent coordinates
        self.nqc = mbs_data.nqc  # Number of driven coordinates
        self.nq = self.nqu + self.nqc  # Total coordinates
        
        print(f"NERi Solver initialized:")
        print(f"  Independent coordinates (qu): {self.nqu}")
        print(f"  Driven coordinates (qc): {self.nqc}")
        print(f"  Total coordinates: {self.nq}")
        
        # Storage for simulation results
        self.time_history = []
        self.q_history = []
        self.qd_history = []
        self.qdd_history = []
        self.M_history = []  # Mass matrices
        self.c_history = []  # Constraint vectors
        self.forces_history = []  # Link forces
        
    def driven_joint_law(self, t: float) -> np.ndarray:
        """
        Define the law for driven (commanded) joints.
        
        Currently implements a simple startup ramp:
        Motor starts at t=0 and reaches full speed gradually.
        
        Parameters
        ----------
        t : float
            Current time
            
        Returns
        -------
        np.ndarray
            Acceleration values for driven coordinates [nqc]
        """
        qc_acc = np.zeros(self.nqc)
        
        # Example: Motor ramp-up (assuming Pole1 is index 0 in driven coords)
        if t < 1.0:
            # Acceleration phase (0-1s): reach target angular velocity
            qc_acc[0] = 2.0  # rad/s^2
        elif t < 2.0:
            # Constant velocity phase
            qc_acc[0] = 0.0
        else:
            # Deceleration phase
            qc_acc[0] = -1.0
            
        return qc_acc
    
    def state_derivative(self, t: float, state: np.ndarray, 
                        state_obj=None) -> np.ndarray:
        """
        Compute the state derivative (RK4 integration function).
        
        This is the core function implementing the NERi formalism:
        1. Extract state: [q_u, qd_u, qd_c]
        2. Compute driven joint accelerations
        3. Call dirdyna to compute M and c
        4. Perform partition to extract independent accelerations
        5. Return derivatives: [qd_u, qdd_u, qdd_c]
        
        Parameters
        ----------
        t : float
            Current time
        state : np.ndarray
            Current state [q_u, qd_u, qd_c]
        state_obj : object
            State object with q, qd, qdd arrays (for dirdyna compatibility)
            
        Returns
        -------
        np.ndarray
            State derivatives [dq_u/dt, dqd_u/dt, dqd_c/dt]
        """
        # Reconstruct full state arrays
        q_u = state[:self.nqu]
        qd_u = state[self.nqu:2*self.nqu]
        qd_c = state[2*self.nqu:]
        
        # Reconstruct full coordinate arrays
        q = np.zeros(self.nq + 1)  # +1 for 1-based indexing in Robotran
        qd = np.zeros(self.nq + 1)
        qdd = np.zeros(self.nq + 1)
        
        # Fill independent coordinates
        q[1:self.nqu+1] = q_u
        qd[1:self.nqu+1] = qd_u
        
        # Fill driven coordinates (from integration state)
        qd[self.nqu+1:] = qd_c
        
        # Create state object for dirdyna
        class State:
            pass
        
        state_obj = State()
        state_obj.q = q
        state_obj.qd = qd
        state_obj.qdd = qdd
        state_obj.g = self.mbs_data.g
        state_obj.dpt = self.mbs_data.dpt
        
        # Compute mass matrix M and constraint vector c
        M = np.zeros((self.nq + 1, self.nq + 1))
        c = np.zeros(self.nq + 1)
        
        dirdyna(M, c, state_obj, t)
        
        # Store for debugging/analysis
        self.time_history.append(t)
        self.M_history.append(M.copy())
        self.c_history.append(c.copy())
        
        # ===== PARTITION: Separate independent from driven equations =====
        # Equation system: M @ qdd = c
        # [M_uu  M_uc] [qdd_u]   [c_u]
        # [M_cu  M_cc] [qdd_c] = [c_c]
        
        # Extract partitions (0-based indexing internally)
        M_uu = M[1:self.nqu+1, 1:self.nqu+1]
        M_uc = M[1:self.nqu+1, self.nqu+1:]
        M_cu = M[self.nqu+1:, 1:self.nqu+1]
        M_cc = M[self.nqu+1:, self.nqu+1:]
        
        c_u = c[1:self.nqu+1]
        c_c = c[self.nqu+1:]
        
        # Get driven accelerations from prescribed motion law
        qdd_c = self.driven_joint_law(t)
        
        # Compute independent accelerations:
        # M_uu @ qdd_u = c_u - M_uc @ qdd_c
        rhs = c_u - M_uc @ qdd_c
        
        try:
            qdd_u = np.linalg.solve(M_uu, rhs)
        except np.linalg.LinAlgError:
            print(f"Warning: Singular mass matrix at t={t}")
            qdd_u = np.linalg.lstsq(M_uu, rhs, rcond=None)[0]
        
        # ===== Return state derivatives =====
        # State = [q_u, qd_u, qd_c]
        dstate = np.concatenate([
            qd_u,      # dq_u/dt = qd_u
            qdd_u,     # dqd_u/dt = qdd_u
            qdd_c      # dqd_c/dt = qdd_c (prescribed)
        ])
        
        # Store history
        qdd_full = np.zeros(self.nq + 1)
        qdd_full[1:self.nqu+1] = qdd_u
        qdd_full[self.nqu+1:] = qdd_c
        self.qdd_history.append(qdd_full.copy())
        
        return dstate
    
    def integrate(self, t0: float, tf: float, dt: float, 
                 initial_conditions: Dict) -> Dict:
        """
        Perform time integration using RK4 (or scipy's solve_ivp).
        
        Parameters
        ----------
        t0 : float
            Initial time
        tf : float
            Final time
        dt : float
            Integration step size
        initial_conditions : Dict
            Dictionary with keys:
            - 'qu': initial independent coordinates
            - 'qdu': initial independent velocities
            - 'qdc': initial driven velocities
            
        Returns
        -------
        Dict
            Results dictionary with keys: t, q, qd, qdd
        """
        # Extract initial conditions
        qu0 = initial_conditions.get('qu', self.mbs_data.qu[1:self.nqu+1])
        qdu0 = initial_conditions.get('qdu', np.zeros(self.nqu))
        qdc0 = initial_conditions.get('qdc', np.zeros(self.nqc))
        
        # Form initial state vector [q_u, qd_u, qd_c]
        y0 = np.concatenate([qu0, qdu0, qdc0])
        
        # Time vector
        t_eval = np.arange(t0, tf + dt, dt)
        
        print(f"\nNERi Integration:")
        print(f"  Time range: {t0}s to {tf}s")
        print(f"  Time step: {dt}s")
        print(f"  Number of steps: {len(t_eval)}")
        print(f"  Initial state shape: {y0.shape}")
        
        # ===== Use scipy's RK45 integrator =====
        sol = solve_ivp(
            self.state_derivative,
            (t0, tf),
            y0,
            method='RK45',
            t_eval=t_eval,
            dense_output=False,
            vectorized=False,
            max_step=dt
        )
        
        if not sol.success:
            print(f"Warning: Integration failed - {sol.message}")
        
        # ===== Extract and reconstruct full state =====
        n_steps = len(sol.t)
        q_full = np.zeros((n_steps, self.nq + 1))
        qd_full = np.zeros((n_steps, self.nq + 1))
        qdd_full = np.array(self.qdd_history[:n_steps])
        
        for i, t in enumerate(sol.t):
            state = sol.y[:, i]
            
            # Reconstruct state
            q_u = state[:self.nqu]
            qd_u = state[self.nqu:2*self.nqu]
            qd_c = state[2*self.nqu:]
            
            q_full[i, 1:self.nqu+1] = q_u
            q_full[i, self.nqu+1:] = np.zeros(self.nqc)  # Would need integration
            
            qd_full[i, 1:self.nqu+1] = qd_u
            qd_full[i, self.nqu+1:] = qd_c
        
        # Store history
        self.q_history = q_full
        self.qd_history = qd_full
        
        # ===== Compute internal forces =====
        print(f"\nComputing internal forces...")
        self._compute_internal_forces(sol.t, q_full, qd_full)
        
        # Return results
        results = {
            't': sol.t,
            'q': q_full,
            'qd': qd_full,
            'qdd': qdd_full,
            'M_history': self.M_history,
            'c_history': self.c_history,
            'forces': self.forces_history
        }
        
        return results
    
    def _compute_internal_forces(self, t_eval: np.ndarray, 
                                 q_history: np.ndarray, 
                                 qd_history: np.ndarray):
        """
        Compute internal forces (link forces) for each time step.
        
        Parameters
        ----------
        t_eval : np.ndarray
            Time values
        q_history : np.ndarray
            Coordinate history [n_steps, nq]
        qd_history : np.ndarray
            Velocity history [n_steps, nq]
        """
        # Reset forces history
        self.forces_history = []
        
        for i, t in enumerate(t_eval):
            # Create state object
            class State:
                pass
            
            state_obj = State()
            state_obj.q = q_history[i]
            state_obj.qd = qd_history[i]
            state_obj.qdd = np.zeros(self.nq + 1)
            state_obj.g = self.mbs_data.g
            state_obj.dpt = self.mbs_data.dpt
            
            # Compute link forces
            # Note: This requires knowing the number and type of links
            # Placeholder implementation
            frc = self.mbs_data.frc if hasattr(self.mbs_data, 'frc') else np.zeros((1, 1))
            trq = self.mbs_data.trq if hasattr(self.mbs_data, 'trq') else np.zeros((1, 1))
            
            try:
                # Call link function to compute internal forces
                # This is a placeholder - the actual implementation depends on your links
                Z = np.zeros(8)  # Number of links in your system
                Zd = np.zeros(8)
                Flink = np.zeros(8)
                
                link(frc, trq, Flink, Z, Zd, state_obj, t)
                
                self.forces_history.append({
                    't': t,
                    'Flink': Flink.copy(),
                    'Z': Z.copy(),
                    'Zd': Zd.copy()
                })
            except Exception as e:
                print(f"Warning: Could not compute link forces at t={t}: {e}")
                self.forces_history.append({
                    't': t,
                    'Flink': np.zeros(8),
                    'Z': np.zeros(8),
                    'Zd': np.zeros(8)
                })
    
    def compute_stresses(self, safety_factor: float = 10.0) -> Dict:
        """
        Compute internal stresses in structural elements (beams, joints, links).
        
        Parameters
        ----------
        safety_factor : float
            Safety factor for dimensioning
            
        Returns
        -------
        Dict
            Stresses and dimensioning results
        """
        print(f"\nComputing internal stresses (safety factor = {safety_factor})...")
        
        stresses = {
            'max_forces': {},
            'max_moments': {},
            'dimensioning': {}
        }
        
        # Extract maximum forces from link forces history
        if self.forces_history:
            all_forces = np.array([f['Flink'] for f in self.forces_history])
            max_force = np.max(np.abs(all_forces))
            min_force = np.min(all_forces)
            
            stresses['max_forces'] = {
                'max': max_force,
                'min': min_force,
                'mean': np.mean(all_forces)
            }
            
            print(f"  Link forces: min={min_force:.4e}, max={max_force:.4e}")
        
        # Extract maximum accelerations (related to inertial forces)
        if self.qdd_history:
            all_acc = np.array(self.qdd_history)
            max_acc = np.max(np.abs(all_acc))
            
            stresses['max_moments'] = {
                'max_angular_acc': max_acc
            }
            
            print(f"  Max angular acceleration: {max_acc:.4e} rad/s^2")
        
        # Dimensioning with safety factor
        if self.forces_history and self.mbs_data.m is not None:
            # Example: Dimension a shaft
            total_mass = np.sum(self.mbs_data.m[1:])  # Skip index 0
            inertial_force = total_mass * 9.81  # Gravity
            
            max_force = stresses['max_forces']['max']
            design_force = (inertial_force + max_force) * safety_factor
            
            # Assume shaft stress calculation: sigma = M / (pi * d^3 / 32)
            # For a given max allowable stress (e.g., steel: 250 MPa)
            max_stress_allowable = 250e6  # Pa
            
            # Required shaft diameter
            required_d = (32 * design_force / (np.pi * max_stress_allowable)) ** (1/3)
            
            stresses['dimensioning'] = {
                'total_mass': total_mass,
                'inertial_force': inertial_force,
                'design_force': design_force,
                'required_shaft_diameter': required_d,
                'recommended_diameter': np.ceil(required_d * 100) / 100  # Round up
            }
            
            print(f"\nDimensioning results:")
            print(f"  Total mass: {total_mass:.2f} kg")
            print(f"  Design force: {design_force:.4e} N")
            print(f"  Required shaft diameter: {required_d:.4f} m ({required_d*1000:.2f} mm)")
            print(f"  Recommended diameter: {stresses['dimensioning']['recommended_diameter']:.4f} m")
        
        return stresses
