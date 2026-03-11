#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stress and Dimensioning Analysis Module

Computes internal stresses in structural elements and performs
dimensioning with safety factors according to beam theory and
material properties.
"""

import numpy as np
from typing import Dict, Tuple, Optional
from enum import Enum


class BeamMaterial(Enum):
    """Common structural beam materials."""
    STEEL = {'name': 'Steel', 'young_modulus': 210e9, 'yield_stress': 250e6}
    ALUMINUM = {'name': 'Aluminum', 'young_modulus': 70e9, 'yield_stress': 280e6}
    TITANIUM = {'name': 'Titanium', 'young_modulus': 103e9, 'yield_stress': 880e6}
    COMPOSITE = {'name': 'Composite', 'young_modulus': 150e9, 'yield_stress': 600e6}


class StressAnalyzer:
    """Analyze stresses and perform structural dimensioning."""
    
    def __init__(self, material: BeamMaterial = BeamMaterial.STEEL):
        """
        Initialize stress analyzer.
        
        Parameters
        ----------
        material : BeamMaterial
            Material properties
        """
        self.material = material
        self.props = material.value
    
    def bending_stress(self, bending_moment: float, 
                      section_modulus: float) -> float:
        """
        Compute bending stress in a beam.
        
        σ_bend = M / W
        
        Parameters
        ----------
        bending_moment : float
            Bending moment (N·m)
        section_modulus : float
            Section modulus (m³)
            
        Returns
        -------
        float
            Bending stress (Pa)
        """
        if section_modulus <= 0:
            raise ValueError("Section modulus must be positive")
        return bending_moment / section_modulus
    
    def torsional_stress(self, torque: float, 
                         polar_moment: float, radius: float) -> float:
        """
        Compute torsional stress in a shaft.
        
        τ = T * r / J
        
        Parameters
        ----------
        torque : float
            Applied torque (N·m)
        polar_moment : float
            Polar moment of inertia (m⁴)
        radius : float
            Radius of shaft (m)
            
        Returns
        -------
        float
            Torsional stress (Pa)
        """
        if polar_moment <= 0:
            raise ValueError("Polar moment must be positive")
        return (torque * radius) / polar_moment
    
    def combined_stress(self, bending_stress: float, 
                       torsional_stress: float) -> float:
        """
        Compute combined (von Mises) stress.
        
        σ_vm = sqrt(σ² + 3τ²)
        
        Parameters
        ----------
        bending_stress : float
            Bending stress (Pa)
        torsional_stress : float
            Torsional stress (Pa)
            
        Returns
        -------
        float
            von Mises equivalent stress (Pa)
        """
        return np.sqrt(bending_stress**2 + 3 * torsional_stress**2)
    
    def safety_factor(self, yield_stress: float, 
                      applied_stress: float) -> float:
        """
        Compute safety factor.
        
        SF = σ_yield / σ_applied
        
        Parameters
        ----------
        yield_stress : float
            Material yield stress (Pa)
        applied_stress : float
            Applied stress (Pa)
            
        Returns
        -------
        float
            Safety factor
        """
        if applied_stress <= 0:
            return float('inf')
        return yield_stress / applied_stress
    
    def required_diameter_circular_shaft(self, torque: float, 
                                         applied_stress_limit: float) -> float:
        """
        Compute required diameter for circular shaft.
        
        For solid circular shaft: J = π*d⁴/32
        τ_max = T*d/2 / J = 16*T/(π*d³)
        d = ∛(16*T/(π*τ_limit))
        
        Parameters
        ----------
        torque : float
            Applied torque (N·m)
        applied_stress_limit : float
            Maximum allowable stress (Pa)
            
        Returns
        -------
        float
            Required diameter (m)
        """
        return (16 * torque / (np.pi * applied_stress_limit)) ** (1/3)
    
    def required_diameter_hollow_shaft(self, torque: float,
                                       applied_stress_limit: float,
                                       thickness_ratio: float = 0.8) -> Tuple[float, float]:
        """
        Compute required dimensions for hollow circular shaft.
        
        Parameters
        ----------
        torque : float
            Applied torque (N·m)
        applied_stress_limit : float
            Maximum allowable stress (Pa)
        thickness_ratio : float
            Ratio of inner to outer diameter (0 < ratio < 1)
            
        Returns
        -------
        Tuple[float, float]
            (outer_diameter, inner_diameter)
        """
        # For hollow shaft: J = π*(D⁴ - d⁴)/32
        # With d = k*D (k = thickness_ratio)
        # J = π*D⁴/32 * (1 - k⁴)
        # τ_max = 16*T/(π*D³*(1-k⁴))
        # D = ∛(16*T/(π*τ_limit*(1-k⁴)))
        
        k = thickness_ratio
        D = (16 * torque / (np.pi * applied_stress_limit * (1 - k**4))) ** (1/3)
        d = k * D
        
        return D, d
    
    def beam_deflection(self, load: float, length: float, 
                       young_modulus: float, 
                       moment_of_inertia: float) -> float:
        """
        Compute maximum deflection for cantilever beam.
        
        δ_max = P*L³/(3*E*I)
        
        Parameters
        ----------
        load : float
            Point load at free end (N)
        length : float
            Beam length (m)
        young_modulus : float
            Young's modulus (Pa)
        moment_of_inertia : float
            Second moment of inertia (m⁴)
            
        Returns
        -------
        float
            Maximum deflection (m)
        """
        return (load * length**3) / (3 * young_modulus * moment_of_inertia)
    
    def buckling_load(self, length: float, young_modulus: float,
                     moment_of_inertia: float, 
                     boundary_condition: str = 'pinned') -> float:
        """
        Compute Euler buckling load.
        
        Parameters
        ----------
        length : float
            Column length (m)
        young_modulus : float
            Young's modulus (Pa)
        moment_of_inertia : float
            Second moment of inertia (m⁴)
        boundary_condition : str
            'fixed-free', 'pinned', 'fixed-pinned', 'fixed-fixed'
            
        Returns
        -------
        float
            Critical buckling load (N)
        """
        # Euler buckling: P_cr = π²*E*I/(k*L)²
        # where k depends on boundary conditions
        
        k_dict = {
            'fixed-free': 2.0,
            'pinned': 1.0,
            'fixed-pinned': 0.7,
            'fixed-fixed': 0.5
        }
        
        k = k_dict.get(boundary_condition, 1.0)
        return (np.pi**2 * young_modulus * moment_of_inertia) / (k * length)**2
    
    def fatigue_analysis(self, stress_range: float, 
                        endurance_limit: float,
                        mean_stress: float = 0.0) -> float:
        """
        Compute fatigue safety factor using Goodman criterion.
        
        1/SF = σ_a/σ_e + σ_m/σ_ult
        
        Parameters
        ----------
        stress_range : float
            Alternating stress amplitude (Pa)
        endurance_limit : float
            Material endurance limit (Pa)
        mean_stress : float
            Mean stress (Pa)
            
        Returns
        -------
        float
            Fatigue safety factor
        """
        if stress_range <= 0:
            return float('inf')
        
        # Simplified: assume ultimate stress ~ 1.5 * yield stress
        ultimate_stress = self.props['yield_stress'] * 1.5
        
        inv_sf = (stress_range / endurance_limit) + (mean_stress / ultimate_stress)
        
        if inv_sf <= 0:
            return float('inf')
        
        return 1.0 / inv_sf


class StructuralDimensioning:
    """Automated structural dimensioning based on forces and stresses."""
    
    def __init__(self, safety_factor: float = 10.0,
                 material: BeamMaterial = BeamMaterial.STEEL):
        """
        Initialize dimensioning tool.
        
        Parameters
        ----------
        safety_factor : float
            Global safety factor
        material : BeamMaterial
            Material properties
        """
        self.safety_factor_global = safety_factor
        self.analyzer = StressAnalyzer(material)
    
    def dimension_main_shaft(self, max_torque: float, 
                            max_force: float = 0.0) -> Dict:
        """
        Dimension main carousel shaft.
        
        Parameters
        ----------
        max_torque : float
            Maximum torque (N·m)
        max_force : float
            Maximum radial force (N)
            
        Returns
        -------
        Dict
            Dimensioning results
        """
        # Apply safety factor globally
        design_torque = max_torque * self.safety_factor_global
        
        # Allowable stress with safety factor
        allowable_stress = self.analyzer.props['yield_stress'] / self.safety_factor_global
        
        # Compute shaft diameter for torsion
        d_torsion = self.analyzer.required_diameter_circular_shaft(
            design_torque, allowable_stress
        )
        
        # If there's also a bending force, account for it
        if max_force > 0:
            design_force = max_force * self.safety_factor_global
            # Approximate bending moment (assuming L/4 from support)
            # This is a simplification - actual design would be more detailed
            d_bending = (32 * design_force / (np.pi * allowable_stress)) ** (1/3)
            d_final = max(d_torsion, d_bending)
        else:
            d_final = d_torsion
        
        # Round up to practical size
        d_practical = np.ceil(d_final * 1000) / 1000  # mm resolution
        
        # Compute actual stress with practical diameter
        J_practical = np.pi * d_practical**4 / 32
        tau_actual = (design_torque * (d_practical/2)) / J_practical
        sf_actual = self.analyzer.safety_factor(
            self.analyzer.props['yield_stress'], tau_actual
        )
        
        return {
            'component': 'Main Shaft',
            'diameter_required': d_torsion,
            'diameter_practical': d_practical,
            'diameter_mm': d_practical * 1000,
            'design_torque': design_torque,
            'actual_stress': tau_actual,
            'actual_safety_factor': sf_actual,
            'material': self.analyzer.props['name']
        }
    
    def dimension_bearings(self, radial_load: float, 
                          axial_load: float = 0.0) -> Dict:
        """
        Suggest bearing size based on loads.
        
        Parameters
        ----------
        radial_load : float
            Radial load (N)
        axial_load : float
            Axial load (N)
            
        Returns
        -------
        Dict
            Bearing recommendations
        """
        # Apply safety factor
        design_radial = radial_load * self.safety_factor_global
        design_axial = axial_load * self.safety_factor_global if axial_load > 0 else 0
        
        # Combined load
        combined_load = np.sqrt(design_radial**2 + design_axial**2 + 1e-10)
        
        # Empirical bearing sizing (based on common bearing capacities)
        # This is approximate and would normally use bearing supplier data
        
        # Estimate bore diameter based on shaft diameter
        bore_diameter_estimate = 0.1  # Default estimate in meters
        
        return {
            'component': 'Bearings',
            'radial_load': radial_load,
            'axial_load': axial_load,
            'design_radial_load': design_radial,
            'design_axial_load': design_axial,
            'combined_load': combined_load,
            'recommended_bore_diameter': bore_diameter_estimate,
            'note': 'Consult bearing manufacturer for exact selection'
        }
    
    def dimension_connections(self, shear_force: float, 
                             moment: float) -> Dict:
        """
        Dimension bolted connections.
        
        Parameters
        ----------
        shear_force : float
            Shear force (N)
        moment : float
            Bending moment (N·m)
            
        Returns
        -------
        Dict
            Connection design recommendations
        """
        design_force = shear_force * self.safety_factor_global
        design_moment = moment * self.safety_factor_global
        
        # Simplified: assume 4 bolts
        force_per_bolt = design_force / 4
        
        # M12 bolt typical specs (yield ~400 MPa)
        typical_bolt_area = 84.3e-6  # m² for M12
        bolt_stress = force_per_bolt / typical_bolt_area
        
        return {
            'component': 'Bolted Connection',
            'design_shear_force': design_force,
            'design_moment': design_moment,
            'force_per_bolt': force_per_bolt,
            'typical_bolt_size': 'M12',
            'stress_per_bolt': bolt_stress,
            'recommended_grade': '8.8 or 10.9',
            'note': 'Design is approximate; formal calculation required'
        }
    
    def generate_report(self, max_torque: float, max_force: float,
                       max_radial_load: float) -> Dict:
        """
        Generate complete dimensioning report.
        
        Parameters
        ----------
        max_torque : float
            Maximum torque (N·m)
        max_force : float
            Maximum force (N)
        max_radial_load : float
            Maximum radial load (N)
            
        Returns
        -------
        Dict
            Complete dimensioning report
        """
        report = {
            'safety_factor': self.safety_factor_global,
            'material': self.analyzer.props['name'],
            'components': {}
        }
        
        # Dimension main shaft
        report['components']['main_shaft'] = self.dimension_main_shaft(
            max_torque, max_force
        )
        
        # Dimension bearings
        report['components']['bearings'] = self.dimension_bearings(
            max_radial_load, max_force * 0.1  # Assume small axial component
        )
        
        # Dimension connections
        report['components']['connections'] = self.dimension_connections(
            max_radial_load, max_torque * 0.1  # Estimate
        )
        
        return report
    
    def print_report(self, report: Dict):
        """Print formatted dimensioning report."""
        print("\n" + "="*70)
        print("STRUCTURAL DIMENSIONING REPORT")
        print("="*70)
        print(f"Global Safety Factor: {report['safety_factor']}")
        print(f"Material: {report['material']}")
        print(f"Yield Stress: {self.analyzer.props['yield_stress']/1e6:.0f} MPa")
        
        for comp_name, comp_data in report['components'].items():
            print(f"\n{comp_name.upper().replace('_', ' ')}:")
            for key, value in comp_data.items():
                if isinstance(value, float):
                    if 'diameter' in key or 'load' in key:
                        print(f"  {key}: {value:.4f}")
                    elif 'stress' in key:
                        print(f"  {key}: {value/1e6:.2f} MPa")
                    elif 'safety' in key:
                        print(f"  {key}: {value:.2f}")
                    else:
                        print(f"  {key}: {value}")
                else:
                    if key != 'component':
                        print(f"  {key}: {value}")
