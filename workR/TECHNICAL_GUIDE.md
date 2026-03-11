# NERi Solver - Technical Implementation Guide

## Table of Contents
1. [Mathematical Formulation](#mathematical-formulation)
2. [Implementation Architecture](#implementation-architecture)
3. [Key Algorithms](#key-algorithms)
4. [Customization Guide](#customization-guide)
5. [Debugging Guide](#debugging-guide)
6. [Performance Optimization](#performance-optimization)

---

## Mathematical Formulation

### 1. Multibody Dynamics Fundamentals

The dynamics of a multibody system are governed by the **Lagrangian equations of motion**:

```
d/dt(∂L/∂q̇) - ∂L/∂q = Q + λ_c^T
```

Where:
- **L = T - V**: Lagrangian (T = kinetic, V = potential energy)
- **Q**: Generalized external forces/torques
- **λ_c**: Lagrange multipliers for constraints
- **q**: Generalized coordinates

### 2. NERi Formalism Partitioning

The NERi approach partitions the n total coordinates into:
- **k = nqu**: Independent (free) coordinates
- **m = nqc**: Driven (controlled) coordinates
- **Total: n = k + m**

The **mass matrix system**:

```
[M_uu  M_uc] [q̈_u]   [c_u]
[M_cu  M_cc] [q̈_c] = [c_c]
```

**Notation:**
- Subscript u = "uncontrolled" (independent)
- Subscript c = "controlled" (driven)
- M = symmetric mass matrix
- c = constraint/force vector

### 3. Solution Strategy

**Step 1:** Get prescribed accelerations
```
q̈_c = f(t)  [From motor law or user function]
```

**Step 2:** Extract lower block of equations
```
M_uu @ q̈_u + M_uc @ q̈_c = c_u
```

**Step 3:** Solve for independent accelerations
```
q̈_u = M_uu^(-1) @ (c_u - M_uc @ q̈_c)
```

**Step 4:** Integrate state vector
```
dq_u/dt = q̇_u
dq̇_u/dt = q̈_u
dq̇_c/dt = q̈_c
```

### 4. Energy Considerations

**Kinetic Energy:**
```
T = (1/2) * q̇^T @ M @ q̇
```

**Potential Energy (gravity):**
```
V = Σ m_i * g * z_i
```

**Total Mechanical Energy:**
```
E = T + V
```

The system is conservative if dE/dt ≈ 0 (conservation property).

---

## Implementation Architecture

### Module Dependencies

```
main.py
├── NERi_solver.py          [Core integration engine]
│   ├── mbs_dirdyna_*       [Generated symbolic functions]
│   ├── mbs_link_*          [Generated force functions]
│   └── scipy.integrate     [RK45 integrator]
├── NERi_postprocessor.py   [Analysis and plotting]
│   └── matplotlib          [Visualization]
├── NERi_stress_analysis.py [Dimensioning]
└── neri_config.py          [Configuration]
```

### Class Hierarchy

```
NERiSolver (Main solver)
├── state_derivative()      [RHS for integration]
├── integrate()             [Main integration loop]
├── _compute_internal_forces() [Force calculation]
└── compute_stresses()      [Stress analysis]

NERiPostProcessor (Analysis)
├── plot_results()          [Trajectory plots]
├── plot_error_analysis()   [Comparison]
├── plot_energy_analysis()  [Energy evolution]
└── summary_statistics()    [Metrics]

StressAnalyzer (Stress computation)
├── bending_stress()        [Beam formulas]
├── torsional_stress()      [Shaft formulas]
├── combined_stress()       [von Mises]
└── required_diameter_*()   [Dimensioning]

StructuralDimensioning (Component design)
├── dimension_main_shaft()  [Shaft sizing]
├── dimension_bearings()    [Bearing selection]
├── dimension_connections() [Bolt design]
└── generate_report()       [Full report]
```

---

## Key Algorithms

### Algorithm 1: State Partitioning

```
function partition_equations(M, c, nqu, nqc, qdd_c):
    # Extract partitions (1-based indexing converted to 0-based)
    M_uu = M[0:nqu, 0:nqu]
    M_uc = M[0:nqu, nqu:nqu+nqc]
    M_cu = M[nqu:nqu+nqc, 0:nqu]
    M_cc = M[nqu:nqu+nqc, nqu:nqu+nqc]
    
    c_u = c[0:nqu]
    c_c = c[nqu:nqu+nqc]
    
    # Solve for independent accelerations
    rhs = c_u - M_uc @ qdd_c
    qdd_u = solve(M_uu, rhs)  # or lstsq if singular
    
    return qdd_u, M_uu, c_u
```

**Complexity:** O(nqu³) for matrix inversion


### Algorithm 2: RK45 Integration

```
function integrate_rk45(state_deriv, y0, t_span, t_eval):
    # Using scipy's solve_ivp
    solution = solve_ivp(
        fun=state_deriv,
        t_span=t_span,
        y0=y0,
        method='RK45',       # 4th/5th order Runge-Kutta
        t_eval=t_eval,
        dense_output=True,
        max_step=dt
    )
    
    # RK45 steps:
    # k1 = f(t, y)
    # k2 = f(t + h/4, y + h*k1/4)
    # k3 = f(t + 3h/8, y + 3h/32*k1 + 9h/32*k2)
    # ...
    # y_{n+1} = y_n + h*(35/384*k1 + 500/1113*k3 + ...)
    # Error estimate → Adaptive stepping
    
    return solution
```

**Error Order:** O(h^5) for step size h


### Algorithm 3: Stress Computation

```
function compute_von_mises_stress(sigma_bending, tau_torsion):
    # von Mises equivalent stress
    sigma_vm = sqrt(sigma_bending² + 3*tau_torsion²)
    
    # Safety factor
    SF = yield_stress / sigma_vm
    
    # Dimensioning: Required diameter from stress
    # For circular shaft: tau = 16*T/(π*d³)
    # d = ∛(16*T*SF/(π*yield_stress))
    d = (16*T*SF/(π*yield_stress))^(1/3)
    
    return sigma_vm, SF, d
```

---

## Customization Guide

### 1. Changing the Motor Control Law

**File:** `NERi_solver.py`, method `driven_joint_law()`

**Current implementation (ramp):**
```python
def driven_joint_law(self, t: float) -> np.ndarray:
    qc_acc = np.zeros(self.nqc)
    if t < 1.0:
        qc_acc[0] = 2.0      # Accelerate
    elif t < 2.0:
        qc_acc[0] = 0.0      # Coast
    else:
        qc_acc[0] = -1.0     # Decelerate
    return qc_acc
```

**Alternative 1: PID controller**
```python
def driven_joint_law(self, t: float) -> np.ndarray:
    # Target position
    q_target = 2*t  # Increase angle linearly
    
    # Get current angle and velocity
    q_current = self.mbs_data.qu[1]
    qd_current = self.qd_current[1] if hasattr(self, 'qd_current') else 0
    
    # PID gains
    Kp, Ki, Kd = 10.0, 1.0, 2.0
    e = q_target - q_current
    de = 0 - qd_current  # Want zero velocity error
    
    return np.array([Kp*e + Kd*de])
```

**Alternative 2: Multi-phase profile**
```python
def driven_joint_law(self, t: float) -> np.ndarray:
    qc_acc = np.zeros(self.nqc)
    
    # Phase 1: Fast ramp (0-0.5s)
    if t < 0.5:
        qc_acc[0] = 5.0
    # Phase 2: Slow ramp (0.5-1.5s)
    elif t < 1.5:
        qc_acc[0] = 1.0
    # Phase 3: Hold (>1.5s)
    else:
        qc_acc[0] = 0.0
    
    return qc_acc
```

**Import from config:**
```python
from neri_config import get_motor_acceleration_law

def driven_joint_law(self, t: float) -> np.ndarray:
    qc_acc = np.zeros(self.nqc)
    qc_acc[0] = get_motor_acceleration_law(t)
    return qc_acc
```

### 2. Adding External Forces

Edit `state_derivative()` in `NERi_solver.py`:

```python
def state_derivative(self, t: float, state: np.ndarray, 
                    state_obj=None) -> np.ndarray:
    # ... existing code ...
    
    # Call dirdyna (includes gravity and internal forces)
    dirdyna(M, c, state_obj, t)
    
    # ADD EXTERNAL FORCES HERE
    # Example: Aerodynamic damping
    if t > 0.5:  # After 0.5s
        c[1:self.nqu+1] -= 0.1 * qd_u  # Add damping term
    
    # Example: Time-varying load
    if 1.0 < t < 2.0:
        c[2] += 50.0  # Additional load on coordinate 2
    
    # ... rest of code ...
```

### 3. Modifying Material Properties

**Option 1: Edit config file**
```python
# In neri_config.py
MATERIAL_NAME = 'Titanium'
GLOBAL_SAFETY_FACTOR = 5.0
```

**Option 2: Pass to dimensioning**
```python
from NERi_stress_analysis import StructuralDimensioning, BeamMaterial

dimensioner = StructuralDimensioning(
    safety_factor=8.0,
    material=BeamMaterial.TITANIUM
)
```

**Option 3: Custom material**
```python
# Add to neri_config.py
elif MATERIAL_NAME == 'Custom':
    return {
        'yield_stress': 200e6,      # Your value
        'young_modulus': 100e9,     # Your value
        'density': 2500              # Your value
    }
```

### 4. Changing Integration Parameters

```python
# In main.py
neri_results = neri_solver.integrate(
    t0=0.0,
    tf=5.0,                    # Extend simulation time
    dt=5e-4,                   # Finer time step
    initial_conditions={
        'qu': np.array([0.1, 0.2]),  # Different initial position
        'qdu': np.array([0.0, 0.0]),
        'qdc': np.array([0.0])
    }
)
```

---

## Debugging Guide

### Issue 1: Large Errors vs Robotran

**Diagnostic:**
```python
# Check error at each step
from NERi_postprocessor import NERiPostProcessor
post_proc = NERiPostProcessor(nqu, nqc)
stats = post_proc.summary_statistics(neri_results, robotran_results)
print(f"Max error: {stats['comparison']['max_error_q']}")
print(f"RMSE: {stats['comparison']['rmse_q']}")
```

**Solutions (in order of likely effectiveness):**

1. **Reduce time step:**
   ```python
   dt = 5e-4  # Instead of 1e-3
   ```

2. **Check motor law:**
   ```python
   def driven_joint_law(self, t):
       # Debug output
       print(f"t={t:.3f}: qc_acc = {qc_acc}")
       return qc_acc
   ```

3. **Verify mass matrix:**
   ```python
   # Save first few mass matrices
   M_first = neri_solver.M_history[0]
   print(f"M[0,0] = {M_first[0,0]}")
   print(f"det(M) = {np.linalg.det(M_first)}")
   ```

### Issue 2: Singular Mass Matrix Warning

**Cause:** det(M_uu) ≈ 0 → cannot invert

**Solutions:**

1. **Avoid singular configurations:**
   ```python
   # Check initial conditions
   IC_QU = [0.1, 0.2, ...]  # Avoid exact zero angles
   ```

2. **Use least-squares solver:**
   Edit `N to ERi_solver.py`:
   ```python
   try:
       qdd_u = np.linalg.solve(M_uu, rhs)
   except np.linalg.LinAlgError:
       qdd_u = np.linalg.lstsq(M_uu, rhs, rcond=None)[0]
       print(f"Used lstsq at t={t}")
   ```

### Issue 3: Slow Integration

**Diagnostic:**
```python
import time
t_start = time.time()
neri_results = neri_solver.integrate(...)
t_elapsed = time.time() - t_start
print(f"Elapsed: {t_elapsed:.2f}s for {len(neri_results['t'])} steps")
print(f"Speed: {len(neri_results['t'])/t_elapsed:.0f} steps/s")
```

**Solutions:**

1. **Larger time step** (but check accuracy):
   ```python
   dt = 2e-3  # Was 1e-3
   ```

2. **Use RK23** (lower order, ~2x faster):
   Edit `NERi_solver.py` integrate():
   ```python
   sol = solve_ivp(..., method='RK23', ...)
   ```

3. **Profile code:**
   ```python
   import cProfile
   cProfile.run('neri_solver.integrate(...)')
   ```

---

## Performance Optimization

### 1. Computational Complexity

| Operation | Complexity | Time |
|-----------|-----------|------|
| Dirdyna call | O(n³) | ~5-10 ms |
| Matrix inversion | O(nqu³) | ~1-5 ms |
| Integration step | O(dirdyna) | ~10-20 ms |
| Per simulation | O(N * step) | ~3-5 s for 3s sim |

### 2. Optimization Techniques

**A. Code Profiling**
```python
import cProfile, pstats
profiler = cProfile.Profile()
profiler.enable()

neri_results = neri_solver.integrate(...)

profiler.disable()
stats = pstats.Stats(profiler)
stats.sort_stats('cumulative').print_stats(10)
```

**B. Vectorization**
```python
# Slow:
for i in range(n):
    q_interp[i] = np.interp(t[i], ...)

# Fast:
q_interp = np.interp(t, ...)  # Vectorized
```

**C. Memory Pre-allocation**
```python
# Pre-allocate arrays
q_history = np.zeros((num_steps, nq))  # Avoid repeated allocation
```

**D. Parallel Integration** (advanced)
```python
# Multiple scenarios in parallel
from concurrent.futures import ProcessPoolExecutor

with ProcessPoolExecutor() as executor:
    futures = [
        executor.submit(neri_solver.integrate, t0, t_f, dt, ic)
        for ic in initial_conditions_list
    ]
    results = [f.result() for f in futures]
```

### 3. Memory Usage

Typical for 3-second simulation with 1000 steps:
- q, qd, qdd arrays: ~500 KB
- Mass matrices (M_history): ~400 KB
- Constraint vectors (c_history): ~40 KB
- **Total: ~1 MB** (very lightweight)

---

## References

1. **NERi.pdf** - Course material on formalism
2. **Robotran Documentation** - MBsysPy API
3. **Code Comments** - In-line documentation
4. **Example Scripts** - `example_neri_usage.py`

---

**Last Modified:** March 2026
**Version:** 1.0.0
