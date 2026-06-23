"""
Stewart Platform Collapse Simulation
=====================================
- 6 platform joints: rigid hexagon at Z=38.95, connected 1-2-3-4-5-6-1
- 6 base joints: hexagon at Z=0, each can move up/down up to 10 units
- Base i is connected to Platform i with FIXED-LENGTH leg
- When bases move randomly, platform may collapse if geometry is impossible
"""

import numpy as np
import math
import os

# ============================================================
# 1. GEOMETRY DEFINITION
# ============================================================
platform_joints = np.array([
    [8.526279, -7.232051, 38.951552],    # joint 1
    [10.526279, -3.767949, 38.951552],   # joint 2
    [2.000000, 11.000000, 38.951552],    # joint 3
    [-2.000000, 11.000000, 38.951552],   # joint 4
    [-10.526279, -3.767949, 38.951552],  # joint 5
    [-8.526279, -7.232051, 38.951552],   # joint 6
])

base_joints_initial = np.array([
    [3.100000, -12.600000, 0.000000],    # base 1
    [12.461920, 3.615321, 0.000000],     # base 2
    [9.361920, 8.984679, 0.000000],      # base 3
    [-9.361920, 8.984679, 0.000000],     # base 4
    [-12.461920, 3.615321, 0.000000],    # base 5
    [-3.100000, -12.600000, 0.000000],   # base 6
])

# ============================================================
# 2. FIXED LEG LENGTHS
# ============================================================
def calc_leg_lengths(plat, base):
    return np.array([np.linalg.norm(plat[i] - base[i]) for i in range(6)])

LEG_LENGTHS = calc_leg_lengths(platform_joints, base_joints_initial)
PLATFORM_CENTER = np.mean(platform_joints, axis=0)
REF_PLATFORM = platform_joints - PLATFORM_CENTER  # centered at origin

def log(msg, f=None):
    """Print and optionally write to file"""
    print(msg)
    if f:
        f.write(msg + '\n')

# ============================================================
# 3. ROTATION HELPERS
# ============================================================
def rotation_matrix(roll, pitch, yaw):
    """ZYX Euler angles -> rotation matrix"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def transform_platform(center, roll, pitch, yaw):
    """Compute platform joint positions from pose"""
    R = rotation_matrix(roll, pitch, yaw)
    return center + (R @ REF_PLATFORM.T).T

# ============================================================
# 4. FORWARD KINEMATICS SOLVER (Newton-Raphson)
# ============================================================
def solve_platform(bases, max_iter=200, tol=1e-6):
    """
    Solve for platform pose given base positions and fixed leg lengths.
    Returns (success, center, roll, pitch, yaw, final_error)
    """
    # Initial guess: somewhere above the base center
    center = np.array([0.0, 0.0, 35.0])
    rpy = np.array([0.0, 0.0, 0.0])
    
    for it in range(max_iter):
        # Current platform
        plat = transform_platform(center, rpy[0], rpy[1], rpy[2])
        
        # Current leg lengths
        curr_len = np.array([np.linalg.norm(plat[i] - bases[i]) for i in range(6)])
        
        # Error
        error = curr_len - LEG_LENGTHS
        err_norm = np.linalg.norm(error)
        
        if err_norm < tol:
            return True, center, rpy[0], rpy[1], rpy[2], err_norm
        
        # Jacobian
        J = np.zeros((6, 6))
        for i in range(6):
            diff = plat[i] - bases[i]
            leg = np.linalg.norm(diff)
            if leg < 1e-10:
                return False, center, 0, 0, 0, 999
            
            dir_vec = diff / leg  # unit vector along leg
            
            # Position derivatives
            J[i, :3] = dir_vec
            
            # Angular derivatives: r_i × dir_vec (where r_i = plat_i - center)
            r_i = plat[i] - center
            cross = np.cross(r_i, dir_vec)
            J[i, 3:] = cross
        
        # Newton step
        try:
            delta = np.linalg.solve(J, -error)
        except np.linalg.LinAlgError:
            # Use pseudo-inverse if singular
            delta = np.linalg.lstsq(J, -error, rcond=None)[0]
        
        # Apply damping
        alpha = 0.3
        center += alpha * delta[:3]
        rpy += alpha * delta[3:]
        
        # Safety: NaN check
        if np.any(np.isnan(center)) or np.any(np.isnan(rpy)):
            return False, np.zeros(3), 0, 0, 0, 999
    
    # Check final result
    plat = transform_platform(center, rpy[0], rpy[1], rpy[2])
    curr_len = np.array([np.linalg.norm(plat[i] - bases[i]) for i in range(6)])
    final_err = np.linalg.norm(curr_len - LEG_LENGTHS)
    
    return final_err < 0.5, center, rpy[0], rpy[1], rpy[2], final_err

# ============================================================
# 5. COLLAPSE CRITERIA
# ============================================================
def is_collapsed(bases, verbose=False):
    """
    Check if platform has collapsed given base positions.
    Criteria: solver fails, excessive tilt (>60°), platform hits ground
    """
    success, center, roll, pitch, yaw, err = solve_platform(bases)
    
    if not success:
        return True, f"COLLAPSE: Solver failed (err={err:.4f})"
    
    # Compute platform tilt
    R = rotation_matrix(roll, pitch, yaw)
    normal = R @ np.array([0, 0, 1])
    vertical = np.array([0, 0, 1])
    cos_tilt = np.clip(np.dot(normal, vertical), -1.0, 1.0)
    tilt_deg = math.degrees(math.acos(cos_tilt))
    
    reasons = []
    if tilt_deg > 60:
        reasons.append(f"tilt={tilt_deg:.1f}°>60°")
    if center[2] < 0:
        reasons.append(f"Z_center={center[2]:.1f}<0")
    
    # Also check if individual platform joints go below zero
    plat = transform_platform(center, roll, pitch, yaw)
    min_joint_z = np.min(plat[:, 2])
    if min_joint_z < -2:
        reasons.append(f"min_joint_Z={min_joint_z:.1f}<-2")
    
    if reasons:
        return True, f"COLLAPSE: {'; '.join(reasons)}"
    
    msg = (f"STABLE: center=({center[0]:.1f},{center[1]:.1f},{center[2]:.1f}), "
           f"tilt={tilt_deg:.1f}°, err={err:.6f}")
    return False, msg

# ============================================================
# 6. MAIN SIMULATION
# ============================================================
def run_simulation(num_configs=2000, max_z=10.0):
    """Generate random base Z movements and check for collapse"""
    
    f = open('d:/final/simulation_results.txt', 'w')
    log(f"STEWART PLATFORM COLLAPSE SIMULATION", f)
    log(f"="*60, f)
    log(f"Number of random configurations: {num_configs}", f)
    log(f"Max base Z displacement: {max_z}", f)
    log(f"", f)
    log(f"Initial leg lengths: {LEG_LENGTHS}", f)
    log(f"", f)
    log(f"Base joints at rest (Z=0):", f)
    for i in range(6):
        log(f"  Base {i+1}: {base_joints_initial[i]}", f)
    log(f"", f)
    log(f"Platform joints at rest (Z=38.95):", f)
    for i in range(6):
        log(f"  Plat {i+1}: {platform_joints[i]}", f)
    log(f"", f)
    log(f"{'='*60}", f)
    log(f"", f)
    
    # First, verify solver works on the initial configuration
    log("Verifying solver on initial configuration...", f)
    success, center, roll, pitch, yaw, err = solve_platform(base_joints_initial)
    log(f"  Solver success: {success}, error: {err:.8f}", f)
    log(f"  Found center: {center}", f)
    log(f"", f)
    
    np.random.seed(42)
    collapsed_count = 0
    stable_count = 0
    collapse_examples = []
    
    for test_idx in range(1, num_configs + 1):
        # Random Z displacements
        z_vals = np.random.uniform(0, max_z, 6)
        
        # Apply to bases
        bases = base_joints_initial.copy()
        for i in range(6):
            bases[i, 2] = z_vals[i]
        
        # Test
        coll, msg = is_collapsed(bases)
        
        if coll:
            collapsed_count += 1
            if len(collapse_examples) < 10:
                collapse_examples.append((test_idx, z_vals, msg, bases))
        else:
            stable_count += 1
        
        if test_idx % 200 == 0:
            log(f"  Progress: {test_idx}/{num_configs} "
                f"(collapsed: {collapsed_count}, stable: {stable_count})", f)
    
    log(f"", f)
    log(f"{'='*60}", f)
    log(f"RESULTS", f)
    log(f"{'='*60}", f)
    total = collapsed_count + stable_count
    log(f"Total configurations: {total}", f)
    log(f"Stable:  {stable_count} ({100*stable_count/total:.1f}%)", f)
    log(f"Collapsed: {collapsed_count} ({100*collapsed_count/total:.1f}%)", f)
    log(f"", f)
    
    if collapse_examples:
        log(f"COLLAPSE EXAMPLES (first 10):", f)
        log(f"", f)
        for idx, (test_id, z_vals, msg, bases) in enumerate(collapse_examples):
            log(f"  Example #{idx+1} (Test #{test_id}):", f)
            log(f"    {msg}", f)
            log(f"    Base Z displacements: {np.round(z_vals, 3)}", f)
            
            # Try to get the platform pose info
            success, center, roll, pitch, yaw, err = solve_platform(bases)
            if success:
                plat = transform_platform(center, roll, pitch, yaw)
                log(f"    Platform center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})", f)
                log(f"    Platform joint Z range: ({np.min(plat[:,2]):.2f}, {np.max(plat[:,2]):.2f})", f)
            log(f"", f)
    
    # Save best stable example
    log(f"STABLE EXAMPLES (first 3):", f)
    log(f"", f)
    stable_found = 0
    for test_idx in range(1, num_configs + 1):
        if stable_found >= 3:
            break
        z_vals = np.random.RandomState(test_idx).uniform(0, max_z, 6)
        bases = base_joints_initial.copy()
        for i in range(6):
            bases[i, 2] = z_vals[i]
        coll, msg = is_collapsed(bases)
        if not coll:
            stable_found += 1
            success, center, roll, pitch, yaw, err = solve_platform(bases)
            plat = transform_platform(center, roll, pitch, yaw)
            log(f"  Stable #{stable_found}:", f)
            log(f"    Z displacements: {np.round(z_vals, 3)}", f)
            log(f"    {msg}", f)
            log(f"    Platform joint Z range: ({np.min(plat[:,2]):.2f}, {np.max(plat[:,2]):.2f})", f)
            log(f"", f)
    
    log(f"{'='*60}", f)
    log(f"SUMMARY", f)
    log(f"{'='*60}", f)
    if collapsed_count > stable_count:
        log(f"The platform is PRONE TO COLLAPSE under random base Z movement.", f)
    elif collapsed_count == 0:
        log(f"The platform is STABLE for all tested configurations.", f)
    else:
        log(f"The platform is generally STABLE but can collapse in some cases.", f)
    log(f"", f)
    log(f"Collapse rate: {100*collapsed_count/total:.1f}% ({collapsed_count}/{total})", f)
    
    f.close()
    log(f"\nFull results saved to d:/final/simulation_results.txt", None)

if __name__ == "__main__":
    run_simulation(2000, 10.0)
