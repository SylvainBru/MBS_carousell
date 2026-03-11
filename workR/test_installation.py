#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quick Validation Test for NERi Solver Setup

Run this script to verify:
1. All modules can be imported
2. MBsysPy is accessible
3. Core functions work
4. Visualization is available
"""

import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")
    
    try:
        import numpy as np
        print("  ✓ numpy")
    except ImportError as e:
        print(f"  ✗ numpy: {e}")
        return False
    
    try:
        import scipy
        from scipy.integrate import odeint, solve_ivp
        print("  ✓ scipy")
    except ImportError as e:
        print(f"  ✗ scipy: {e}")
        return False
    
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        print("  ✓ matplotlib")
    except ImportError as e:
        print(f"  ✗ matplotlib: {e}")
        return False
    
    try:
        import MBsysPy as Robotran
        print("  ✓ MBsysPy (Robotran)")
    except ImportError as e:
        print(f"  ✗ MBsysPy: {e}")
        return False
    
    try:
        from NERi_solver import NERiSolver
        print("  ✓ NERi_solver")
    except ImportError as e:
        print(f"  ✗ NERi_solver: {e}")
        return False
    
    try:
        from NERi_postprocessor import NERiPostProcessor
        print("  ✓ NERi_postprocessor")
    except ImportError as e:
        print(f"  ✗ NERi_postprocessor: {e}")
        return False
    
    try:
        from NERi_stress_analysis import StressAnalyzer, StructuralDimensioning
        print("  ✓ NERi_stress_analysis")
    except ImportError as e:
        print(f"  ✗ NERi_stress_analysis: {e}")
        return False
    
    try:
        import neri_config
        print("  ✓ neri_config")
    except ImportError as e:
        print(f"  ✗ neri_config: {e}")
        return False
    
    return True


def test_mbs_data_loading():
    """Test loading MBS data."""
    print("\nTesting MBS data loading...")
    
    try:
        import MBsysPy as Robotran
        
        mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
        if not os.path.exists(mbs_path):
            print(f"  ✗ MBS file not found: {mbs_path}")
            return False
        
        mbs_data = Robotran.MbsData(mbs_path)
        print(f"  ✓ Loaded MBS data")
        print(f"    - Bodies: {mbs_data.nbody}")
        print(f"    - Joints: {mbs_data.njoint}")
        
        # Test partitioning
        mbs_data.process = 1
        mbs_part = Robotran.MbsPart(mbs_data)
        mbs_part.set_options(rowperm=1, verbose=0)
        mbs_part.run()
        
        print(f"  ✓ Partitioning successful")
        print(f"    - Independent (qu): {mbs_data.nqu}")
        print(f"    - Driven (qc): {mbs_data.nqc}")
        
        return True
    
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def test_neri_solver():
    """Test NERi solver instantiation and basic operation."""
    print("\nTesting NERi solver...")
    
    try:
        import MBsysPy as Robotran
        from NERi_solver import NERiSolver
        import numpy as np
        
        # Load and partition
        mbs_path = os.path.join(script_dir, '..', 'dataR', 'Merry_go_round.mbs')
        mbs_data = Robotran.MbsData(mbs_path)
        mbs_data.process = 1
        mbs_part = Robotran.MbsPart(mbs_data)
        mbs_part.set_options(rowperm=1, verbose=0)
        mbs_part.run()
        
        # Create solver
        solver = NERiSolver(mbs_data)
        print(f"  ✓ NERi solver created")
        
        # Test motor law
        acc = solver.driven_joint_law(0.5)
        print(f"  ✓ Motor law evaluated")
        print(f"    - Shape: {acc.shape}")
        print(f"    - Sample value: {acc[0]:.4f} rad/s²")
        
        return True
    
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_postprocessor():
    """Test post-processor instantiation."""
    print("\nTesting post-processor...")
    
    try:
        from NERi_postprocessor import NERiPostProcessor
        
        post_proc = NERiPostProcessor(nqu=10, nqc=5)
        print(f"  ✓ Post-processor created")
        print(f"    - nqu: {post_proc.nqu}")
        print(f"    - nqc: {post_proc.nqc}")
        
        return True
    
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def test_stress_analysis():
    """Test stress analyzer."""
    print("\nTesting stress analysis...")
    
    try:
        from NERi_stress_analysis import StressAnalyzer, BeamMaterial
        
        analyzer = StressAnalyzer(BeamMaterial.STEEL)
        print(f"  ✓ Stress analyzer created")
        
        # Test bending stress
        M = 1000.0  # N·m
        W = 0.001   # m³
        sigma = analyzer.bending_stress(M, W)
        print(f"  ✓ Bending stress computed: {sigma/1e6:.2f} MPa")
        
        # Test required diameter
        d = analyzer.required_diameter_circular_shaft(50, 250e6)
        print(f"  ✓ Required diameter computed: {d*1000:.2f} mm")
        
        return True
    
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def test_config():
    """Test configuration module."""
    print("\nTesting configuration...")
    
    try:
        import neri_config
        
        # Print config
        neri_config.print_configuration()
        
        # Validate config
        neri_config.validate_configuration()
        print(f"  ✓ Configuration validated")
        
        return True
    
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def main():
    """Run all tests."""
    print("\n" + "="*70)
    print("         NERi SOLVER - INSTALLATION VALIDATION TEST")
    print("="*70 + "\n")
    
    results = {
        'Imports': False,
        'MBS Data': False,
        'NERi Solver': False,
        'Post-processor': False,
        'Stress Analysis': False,
        'Configuration': False
    }
    
    # Run tests
    results['Imports'] = test_imports()
    
    if results['Imports']:
        results['MBS Data'] = test_mbs_data_loading()
        results['NERi Solver'] = test_neri_solver()
        results['Post-processor'] = test_postprocessor()
        results['Stress Analysis'] = test_stress_analysis()
        results['Configuration'] = test_config()
    
    # Summary
    print("\n" + "="*70)
    print("                       TEST SUMMARY")
    print("="*70)
    
    passed = 0
    for test_name, passed_test in results.items():
        status = "✓ PASS" if passed_test else "✗ FAIL"
        print(f"  {status}: {test_name}")
        if passed_test:
            passed += 1
    
    print(f"\n  Total: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("\n✓ All tests passed! NERi solver is ready to use.")
        print("\n  Next steps:")
        print("    1. Run: python main.py")
        print("    2. Or: python example_neri_usage.py")
        print("    3. Check README.md for detailed usage")
        return 0
    else:
        print("\n✗ Some tests failed. Check output above for details.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
