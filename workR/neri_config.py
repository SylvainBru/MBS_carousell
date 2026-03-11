#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NERi Solver Configuration File

This file contains all tunable parameters for the NERi solver.
Modify these values to customize the simulation without editing core modules.
"""

import numpy as np

# ===========================================================================
# SIMULATION PARAMETERS
# ===========================================================================

# Time integration
SIMULATION_T0 = 0.0          # Start time (s)
SIMULATION_TF = 3.0          # End time (s)
SIMULATION_DT = 1e-3         # Time step (s)

# Integration method (scipy's solve_ivp)
INTEGRATION_METHOD = 'RK45'  # 'RK45' (adaptive), 'RK23', 'DOP853', 'Radau'
MAX_STEP = 1e-2              # Maximum time step for adaptive integrator

# ===========================================================================
# NERI SOLVER PARAMETERS
# ===========================================================================

# Driven joint motion law
MOTOR_MODE = 'ramp'          # 'ramp', 'sinusoid', 'step', 'custom'

# Ramp motion parameters
RAMP_ACCELERATION_TIME = 1.0      # Duration of acceleration phase (s)
RAMP_ACCELERATION = 2.0           # Acceleration phase rate (rad/s²)
RAMP_COAST_TIME = 1.0             # Duration of constant velocity phase (s)
RAMP_DECELERATION = -1.0          # Deceleration phase rate (rad/s²)

# Sinusoidal motion parameters (if MOTOR_MODE = 'sinusoid')
SINUSOID_FREQUENCY = 0.5          # Frequency (Hz)
SINUSOID_AMPLITUDE = 2.0          # Amplitude (rad/s²)

# Constraint handling
CONSTRAINT_TOLERANCE = 1e-8       # Constraint violation tolerance
MAX_CONSTRAINT_ITERATIONS = 5     # Max iterations for constraint satisfaction

# ===========================================================================
# STRESS ANALYSIS PARAMETERS
# ===========================================================================

# Global safety factor
GLOBAL_SAFETY_FACTOR = 10.0

# Material selection
MATERIAL_NAME = 'Steel'       # 'Steel', 'Aluminum', 'Titanium', 'Composite'
MATERIAL_YIELD_STRESS = 250e6 # Pa (override if different material)
MATERIAL_YOUNGS_MODULUS = 210e9  # Pa

# Shaft dimensioning
SHAFT_DIAMETER_MARGIN = 1.1   # Add 10% margin to computed diameter
SHAFT_ROUNDNESS = 0.001       # Rounding increment (m)

# Bearing parameters
BEARING_BORE_MULTIPLIER = 1.2 # Bore diameter = shaft diameter × multiplier
BEARING_WIDTH_RATIO = 0.8     # Bearing width / bore diameter

# Connection parameters
NUM_BOLTS = 4                 # Number of bolts in connection
BOLT_SIZE_DEFAULT = 'M12'     # Default bolt size
BOLT_GRADE_SHEAR = '8.8'      # Bolt grade for shear stress

# ===========================================================================
# VISUALIZATION PARAMETERS
# ===========================================================================

# Plot configuration
PLOT_FIGSIZE = (16, 12)       # Figure size (width, height)
PLOT_DPI = 150                # Resolution (dots per inch)
PLOT_GRID = True              # Show grid on plots
PLOT_ALPHA_GRID = 0.3         # Grid transparency

# Line styles
PLOT_NERI_COLOR = 'b'         # NERi color (blue)
PLOT_ROBOTRAN_COLOR = 'r'     # Robotran color (red)
PLOT_NERI_STYLE = '-'         # NERi line style (solid)
PLOT_ROBOTRAN_STYLE = '--'    # Robotran line style (dashed)
PLOT_LINEWIDTH_NERI = 2.0
PLOT_LINEWIDTH_ROBOTRAN = 1.5

# Angle units
PLOT_ANGLE_UNIT = 'deg'       # 'deg' or 'rad'

# Energy plot
PLOT_ENERGY_KE_COLOR = 'b'
PLOT_ENERGY_PE_COLOR = 'r'
PLOT_ENERGY_TOTAL_COLOR = 'g'

# Force plot
PLOT_FORCE_NUM_LINES = 4      # Number of force lines to plot

# ===========================================================================
# OUTPUT PARAMETERS
# ===========================================================================

# Output directory
OUTPUT_DIR_NAME = 'NERi_validation'  # Subdirectory in resultsR/

# Output file names
OUTPUT_RESULTS_PLOT = '01_results_comparison.png'
OUTPUT_ERROR_PLOT = '02_error_analysis.png'
OUTPUT_ENERGY_PLOT = '03_energy_analysis.png'
OUTPUT_FORCES_PLOT = '04_internal_forces.png'
OUTPUT_REPORT_TEXT = 'dimensioning_report.txt'

# Save options
SAVE_PLOTS = True
SAVE_REPORT = True
SAVE_DATA_CSV = False         # Save numerical results to CSV
SAVE_DATA_HDF5 = False        # Save to HDF5 format

# ===========================================================================
# VALIDATION PARAMETERS
# ===========================================================================

# Comparison with Robotran
COMPARISON_INTERPOLATION_POINTS = 1000  # For error computation
MAX_ACCEPTABLE_ERROR = 0.01   # 1% maximum error (relative)
ENERGY_CONSERVATION_TOLERANCE = 0.05    # 5% energy loss acceptable

# ===========================================================================
# DEBUGGING OPTIONS
# ===========================================================================

VERBOSE = True                # Detailed console output
DEBUG_STATE_VECTOR = False    # Print state at each step
DEBUG_MASS_MATRIX = False     # Print mass matrix debug info
SAVE_MASS_MATRICES = False    # Save all M matrices for analysis
SAVE_CONSTRAINT_VECTORS = False  # Save all c vectors

# ===========================================================================
# PHYSICAL PARAMETERS (can override MBS defaults)
# ===========================================================================

# Gravity (if different from MBS file)
GRAVITY_OVERRIDE = None       # Set to [gx, gy, gz] to override, or None
# Example: GRAVITY_OVERRIDE = np.array([0, 0, -9.81])

# System damping (global)
SYSTEM_DAMPING_RATIO = 0.0    # Critical damping ratio (0 = undamped)

# ===========================================================================
# INTERNAL FORCE PARAMETERS
# ===========================================================================

# Link force computation
NUM_LINKS = 8                 # Number of links/springs in system
LINK_FORCE_THRESHOLD = 0.1    # Minimum force to report (N)

# ===========================================================================
# HELPER FUNCTIONS
# ===========================================================================

def get_motor_acceleration_law(motor_mode=None, t=0.0):
    """
    Get motor acceleration based on configured mode.
    
    Can be called from NERi solver to generate prescribed accelerations.
    """
    mode = motor_mode or MOTOR_MODE
    
    if mode == 'ramp':
        # Ramp acceleration profile
        if t < RAMP_ACCELERATION_TIME:
            return RAMP_ACCELERATION  # Acceleration phase
        elif t < RAMP_ACCELERATION_TIME + RAMP_COAST_TIME:
            return 0.0  # Constant velocity phase
        else:
            return RAMP_DECELERATION  # Deceleration phase
            
    elif mode == 'sinusoid':
        # Sinusoidal acceleration
        return SINUSOID_AMPLITUDE * np.sin(2*np.pi*SINUSOID_FREQUENCY*t)
        
    elif mode == 'step':
        # Step at t=0.5s
        return RAMP_ACCELERATION if t < 0.5 else RAMP_DECELERATION
        
    else:
        # Custom mode - return 0 (implement in NERi solver)
        return 0.0


def get_material_properties(material_name=None):
    """Get material properties by name."""
    name = material_name or MATERIAL_NAME
    
    materials = {
        'Steel': {'yield_stress': 250e6, 'young_modulus': 210e9, 'density': 7850},
        'Aluminum': {'yield_stress': 280e6, 'young_modulus': 70e9, 'density': 2700},
        'Titanium': {'yield_stress': 880e6, 'young_modulus': 103e9, 'density': 4500},
        'Composite': {'yield_stress': 600e6, 'young_modulus': 150e9, 'density': 1600},
    }
    
    return materials.get(name, materials['Steel'])


def print_configuration():
    """Print current configuration."""
    print("\n" + "="*70)
    print("NERI SOLVER CONFIGURATION")
    print("="*70)
    
    print("\n⏱️  SIMULATION PARAMETERS:")
    print(f"  Time: {SIMULATION_T0:.2f}s → {SIMULATION_TF:.2f}s")
    print(f"  Time step: {SIMULATION_DT:.6f}s")
    print(f"  Integration method: {INTEGRATION_METHOD}")
    
    print("\n🎯 MOTOR CONTROL:")
    print(f"  Mode: {MOTOR_MODE}")
    if MOTOR_MODE == 'ramp':
        print(f"    - Acceleration: {RAMP_ACCELERATION:.2f} rad/s² ({RAMP_ACCELERATION_TIME:.1f}s)")
        print(f"    - Coast: {RAMP_COAST_TIME:.1f}s")
        print(f"    - Deceleration: {RAMP_DECELERATION:.2f} rad/s²")
    elif MOTOR_MODE == 'sinusoid':
        print(f"    - Frequency: {SINUSOID_FREQUENCY:.1f} Hz")
        print(f"    - Amplitude: {SINUSOID_AMPLITUDE:.2f} rad/s²")
    
    print("\n🔧 STRESS ANALYSIS:")
    print(f"  Safety factor: {GLOBAL_SAFETY_FACTOR:.1f}")
    print(f"  Material: {MATERIAL_NAME}")
    print(f"  Yield stress: {MATERIAL_YIELD_STRESS/1e6:.0f} MPa")
    
    print("\n📊 OUTPUT:")
    print(f"  Save plots: {SAVE_PLOTS}")
    print(f"  Save report: {SAVE_REPORT}")
    print(f"  Output directory: {OUTPUT_DIR_NAME}")
    
    print("\n" + "="*70 + "\n")


# ===========================================================================
# VALIDATION FUNCTION
# ===========================================================================

def validate_configuration():
    """Validate configuration values."""
    errors = []
    warnings = []
    
    # Time parameters
    if SIMULATION_TF <= SIMULATION_T0:
        errors.append("SIMULATION_TF must be > SIMULATION_T0")
    if SIMULATION_DT <= 0:
        errors.append("SIMULATION_DT must be positive")
    if SIMULATION_DT > (SIMULATION_TF - SIMULATION_T0) / 100:
        warnings.append("SIMULATION_DT is quite large (>1% of total time)")
    
    # Safety factor
    if GLOBAL_SAFETY_FACTOR < 1.0:
        errors.append("GLOBAL_SAFETY_FACTOR must be >= 1.0")
    if GLOBAL_SAFETY_FACTOR < 2.0:
        warnings.append("GLOBAL_SAFETY_FACTOR < 2.0 is very aggressive")
    
    # Material
    valid_materials = ['Steel', 'Aluminum', 'Titanium', 'Composite']
    if MATERIAL_NAME not in valid_materials:
        errors.append(f"MATERIAL_NAME must be one of {valid_materials}")
    
    if errors:
        print("❌ Configuration errors:")
        for err in errors:
            print(f"  - {err}")
        raise ValueError("Invalid configuration")
    
    if warnings:
        print("⚠️  Configuration warnings:")
        for warn in warnings:
            print(f"  - {warn}")
    
    if not errors and not warnings:
        print("✓ Configuration validated successfully")


if __name__ == '__main__':
    print_configuration()
    validate_configuration()
