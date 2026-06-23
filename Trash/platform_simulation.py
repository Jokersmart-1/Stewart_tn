"""
Stewart Platform Collapse Simulation
=====================================
- 6 platform joints connected rigidly in a cycle (hexagon shape)
- 6 base joints can move along Z axis (max displacement 10)
- Base i is connected rigidly to Platform i
- Simulates random base movements and detects platform collapse
"""

import numpy as np
import random
import math
from copy import deepcopy

# ============================================================
# 1. DEFINED GEOMETRY
# ============================================================

# Platform joints (rigidly connected hexagon at Z=38.95)
platform_joints = np.array([
    [8.526279, -7.232051, 38.951552],    # joint 1
    [10.526279, -3.767949, 38.951552],   # joint 2
    [2.000000, 11.000000, 38.951552],    # joint 3
    [-2.000000, 11.000000, 38.951552],   # joint 4
    [-10.526279, -3.767949, 38.951552],  # joint 5
    [-8.526279, -7.232051, 38.951552],   # joint 6
])

# Base joints (hexagon at Z=0, can move along Z up to 10)
base_joints_initial = np.array([
    [3.100000, -12.600000, 0.000000],    # base 1
    [12.461920, 3.615321, 0.000000],     # base 2
    [9.361920, 8.984679, 0.000000],      # base 3
    [-9.361920, 8.984679, 0.000000],     # base 4
    [-12.461920, 3.615321, 0.000000],    # base 5
    [-3.100000, -12.600000, 0.000000],   # base 6
])

# ============================================================
# 2. COMPUTE LEG LENGTHS (initial, at zero position)
# ============================================================

def compute_leg_lengths(platform_pts, base_pts):
    """Compute distance between each base-platform pair"""
    lengths = np.zeros(6)
    for i in range(6):
        diff = platform_pts[i] - base_pts[i]
        lengths[i] = np.linalg.norm(diff)
    return lengths

# In the initial configuration, these are the fixed leg lengths
# (each leg connects base i to platform i)
rest_leg_lengths = compute_leg_lengths(platform_joints, base_joints_initial)
print("Initial leg lengths:")
for i, l in enumerate(rest_leg_lengths):
    print(f"  Leg {i+1}: {l:.6f}")

# ============================================================
# 3. COMPUTE PLATFORM CENTER AND ORIENTATION (at rest)
# ============================================================

def compute_platform_pose(platform_pts):
    """Compute center and orientation of the platform"""
    center = np.mean(platform_pts, axis=0)
    
    # Compute the plane normal of the platform
    v1 = platform_pts[1] - platform_pts[0]
    v2 = platform_pts[2] - platform_pts[0]
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)
    
    # Convert normal to roll/pitch angles
    # Assuming platform is roughly horizontal at rest
    pitch = math.asin(-normal[0])  # rotation around Y
    roll = math.atan2(normal[1], normal[2])  # rotation around X
    
    return center, normal, roll, pitch

rest_center, rest_normal, rest_roll, rest_pitch = compute_platform_pose(platform_joints)
print(f"\nPlatform center (rest): {rest_center}")
print(f"Platform normal (rest): {rest_normal}")
print(f"Rest roll: {math.degrees(rest_roll):.2f}°, pitch: {math.degrees(rest_pitch):.2f}°")

# ============================================================
# 4. FORWARD KINEMATICS SOLVER
# ============================================================

def rotate_point(point, roll, pitch, yaw, center):
    """Rotate a point around the platform center"""
    # Translate to origin
    p = point - center
    
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: Rz * Ry * Rx (proper Euler angles ZYX)
    R = Rz @ Ry @ Rx
    rotated = R @ p
    
    # Translate back
    return rotated + center

def compute_platform_pts_from_pose(center, roll, pitch, yaw, ref_pts_at_origin):
    """Compute platform joint positions given a pose and reference points (centered at origin)"""
    rotated_pts = np.zeros_like(ref_pts_at_origin)
    for i in range(6):
        rotated_pts[i] = rotate_point(ref_pts_at_origin[i], roll, pitch, yaw, np.zeros(3))
    return rotated_pts + center

def forward_kinematics(base_pts, leg_lengths, ref_platform_pts_at_origin, 
                       max_iterations=1000, tolerance=1e-8):
    """
    Solve for platform pose given base positions and leg lengths.
    Uses gradient descent / Jacobian-based iterative method.
    
    Parameters:
    - base_pts: 6x3 array of base joint positions
    - leg_lengths: 6-element array of desired leg lengths (fixed)
    - ref_platform_pts_at_origin: 6x3 array of platform joint positions relative to center
    
    Returns:
    - (success, center_pos, roll, pitch, yaw)
    """
    
    # Initial guess: center at average of base points + offset, platform horizontal
    # The center should be roughly above the base in the Z direction
    center_z_guess = math.sqrt(leg_lengths[0]**2 - np.linalg.norm(base_pts[0, :2])**2)
    
    state = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])  # [x, y, z, roll, pitch, yaw]
    
    for iteration in range(max_iterations):
        # Current center and orientation
        center = state[:3]
        roll, pitch, yaw = state[3], state[4], state[5]
        
        # Compute current platform joint positions
        curr_platform = compute_platform_pts_from_pose(center, roll, pitch, yaw, ref_platform_pts_at_origin)
        
        # Compute current leg lengths
        curr_lengths = np.zeros(6)
        for i in range(6):
            diff = curr_platform[i] - base_pts[i]
            curr_lengths[i] = np.linalg.norm(diff)
        
        # Compute error (difference between current and desired leg lengths)
        error = curr_lengths - leg_lengths
        error_norm = np.linalg.norm(error)
        
        if error_norm < tolerance:
            return True, center, roll, pitch, yaw
        
        # Compute Jacobian (6x6): derivative of leg lengths w.r.t. state
        J = np.zeros((6, 6))
        
        for i in range(6):
            diff = curr_platform[i] - base_pts[i]
            leg_dir = diff / curr_lengths[i]  # unit vector along the leg
            
            # Derivative of leg i length w.r.t. platform center position
            J[i, :3] = leg_dir  # ∂len/∂center = leg_dir
            
            # For angular derivatives: ∂len/∂angle = (R' * leg_dir) × r_i
            # where r_i is the platform joint position relative to center
            r_i = curr_platform[i] - center
            
            # Compute cross product: r_i × leg_dir
            cross = np.cross(r_i, leg_dir)
            J[i, 3:6] = cross  # [∂len/∂roll, ∂len/∂pitch, ∂len/∂yaw]
        
        # Solve: Δstate = J^(-1) * (-error)
        try:
            delta_state = np.linalg.solve(J, -error)
        except np.linalg.LinAlgError:
            # Use pseudoinverse if singular
            delta_state = np.linalg.lstsq(J, -error, rcond=None)[0]
        
        # Apply damped update
        step_size = 0.5
        state = state + step_size * delta_state
        
        # Check for NaN
        if np.any(np.isnan(state)):
            return False, state[:3], state[3], state[4], state[5]
    
    # Check final error
    center = state[:3]
    roll, pitch, yaw = state[3], state[4], state[5]
    curr_platform = compute_platform_pts_from_pose(center, roll, pitch, yaw, ref_platform_pts_at_origin)
    curr_lengths = np.zeros(6)
    for i in range(6):
        diff = curr_platform[i] - base_pts[i]
        curr_lengths[i] = np.linalg.norm(diff)
    
    final_error = np.linalg.norm(curr_lengths - leg_lengths)
    
    if final_error < 1.0:  # Acceptable error threshold
        return True, center, roll, pitch, yaw
    else:
        return False, center, roll, pitch, yaw

# ============================================================
# 5. COLLAPSE DETECTION
# ============================================================

def check_collapse(base_pts, leg_lengths, ref_platform_pts_at_origin, 
                   max_tilt_deg=60.0, min_center_z=0.0):
    """
    Check if the platform has collapsed/slumped given base positions.
    
    Collapse criteria:
    1. Forward kinematics solver fails to converge
    2. Platform tilt exceeds max_tilt_deg
    3. Platform center Z drops below min_center_z
    """
    
    success, center, roll, pitch, yaw = forward_kinematics(
        base_pts, leg_lengths, ref_platform_pts_at_origin
    )
    
    if not success:
        return True, "Solver failed to converge - platform collapsed!"
    
    # Compute tilt angle (angle between platform normal and vertical)
    ref_pt = ref_platform_pts_at_origin  # at origin
    curr_platform = compute_platform_pts_from_pose(center, roll, pitch, yaw, ref_platform_pts_at_origin)
    _, normal, _, _ = compute_platform_pose(curr_platform)
    
    vertical = np.array([0, 0, 1])
    cos_tilt = np.clip(np.dot(normal, vertical), -1.0, 1.0)
    tilt_deg = math.degrees(math.acos(cos_tilt))
    
    reasons = []
    if tilt_deg > max_tilt_deg:
        reasons.append(f"Excessive tilt: {tilt_deg:.2f}° > {max_tilt_deg}°")
    
    if center[2] < min_center_z:
        reasons.append(f"Platform too low: Z={center[2]:.2f} < {min_center_z}")
    
    # Check if platform orientation is unrealistic
    if abs(roll) > math.radians(80) or abs(pitch) > math.radians(80):
        reasons.append(f"Unrealistic orientation: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°")
    
    if reasons:
        return True, "Collapsed: " + "; ".join(reasons)
    
    return False, f"Stable: tilt={tilt_deg:.2f}°, center=({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})"

# ============================================================
# 6. RANDOM BASE Z MOVEMENT GENERATION
# ============================================================

def generate_random_base_z(max_displacement=10.0):
    """Generate random Z displacements for all 6 base joints"""
    z_displacements = np.random.uniform(0, max_displacement, 6)
    return z_displacements

def apply_z_to_bases(z_displacements):
    """Apply Z displacements to base joints"""
    bases = deepcopy(base_joints_initial)
    for i in range(6):
        bases[i, 2] = z_displacements[i]
    return bases

# ============================================================
# 7. MAIN SIMULATION
# ============================================================

def run_simulation(num_tests=5000, max_displacement=10.0, max_tilt_deg=60.0):
    """Run the main simulation with random base movements"""
    
    # Reference platform points centered at origin
    platform_center_rest = np.mean(platform_joints, axis=0)
    ref_platform_at_origin = platform_joints - platform_center_rest
    
    results = {
        'stable': [],
        'collapsed': [],
        'total': 0,
        'collapse_reasons': {}
    }
    
    print(f"\n{'='*70}")
    print(f"Running simulation: {num_tests} random configurations")
    print(f"Max Z displacement: {max_displacement}")
    print(f"Max allowed tilt: {max_tilt_deg}°")
    print(f"{'='*70}\n")
    
    for test_idx in range(1, num_tests + 1):
        # Generate random Z displacements for base joints
        z_disp = generate_random_base_z(max_displacement)
        base_pts = apply_z_to_bases(z_disp)
        
        # Check for collapse
        collapsed, msg = check_collapse(
            base_pts, rest_leg_lengths, ref_platform_at_origin, max_tilt_deg
        )
        
        if collapsed:
            # Extract reason category
            if "Solver failed" in msg:
                reason = "Solver failed"
            elif "tilt" in msg.lower():
                reason = "Excessive tilt"
            elif "low" in msg.lower():
                reason = "Platform too low"
            else:
                reason = "Other"
            
            results['collapsed'].append({
                'test': test_idx,
                'z_displacements': z_disp,
                'base_positions': base_pts,
                'reason': msg
            })
            
            results['collapse_reasons'][reason] = results['collapse_reasons'].get(reason, 0) + 1
        else:
            results['stable'].append({
                'test': test_idx,
                'z_displacements': z_disp,
                'info': msg
            })
        
        results['total'] += 1
        
        # Progress indicator
        if test_idx % 500 == 0:
            print(f"  Progress: {test_idx}/{num_tests} tests completed "
                  f"({len(results['collapsed'])} collapses found so far)")
    
    return results

def analyze_results(results):
    """Analyze and display simulation results"""
    total = results['total']
    num_collapsed = len(results['collapsed'])
    num_stable = len(results['stable'])
    
    print(f"\n{'='*70}")
    print(f"SIMULATION RESULTS")
    print(f"{'='*70}")
    print(f"Total configurations tested: {total}")
    print(f"Stable configurations: {num_stable} ({100*num_stable/total:.1f}%)")
    print(f"Collapsed configurations: {num_collapsed} ({100*num_collapsed/total:.1f}%)")
    
    if results['collapse_reasons']:
        print(f"\nCollapse breakdown:")
        for reason, count in sorted(results['collapse_reasons'].items(), key=lambda x: -x[1]):
            print(f"  - {reason}: {count} cases ({100*count/num_collapsed:.1f}%)")
    
    # Show some collapse examples
    if num_collapsed > 0:
        print(f"\nSample collapsed configurations (first 5):")
        for i, collapse in enumerate(results['collapsed'][:5]):
            print(f"\n  --- Collapse #{i+1} (Test #{collapse['test']}) ---")
            print(f"  Reason: {collapse['reason']}")
            print(f"  Base Z displacements: {collapse['z_displacements']}")
    
    # Show some stable examples
    if num_stable > 0:
        print(f"\nSample stable configurations (first 3):")
        for i, stable in enumerate(results['stable'][:3]):
            print(f"\n  --- Stable #{i+1} (Test #{stable['test']}) ---")
            print(f"  {stable['info']}")
            print(f"  Base Z displacements: {stable['z_displacements']}")
    
    return num_collapsed, num_stable

# ============================================================
# 8. VISUALIZATION
# ============================================================

def visualize_configuration(base_pts, platform_pts, title="Platform Configuration", 
                           collapsed=False, filename=None):
    """Create a 3D visualization of the platform configuration"""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot base joints
    ax.scatter(base_pts[:, 0], base_pts[:, 1], base_pts[:, 2], 
               c='blue', s=100, marker='o', label='Base joints')
    
    # Plot platform joints
    ax.scatter(platform_pts[:, 0], platform_pts[:, 1], platform_pts[:, 2], 
               c='red' if collapsed else 'green', s=100, marker='^', 
               label='Platform joints')
    
    # Draw legs (connections between base i and platform i)
    for i in range(6):
        ax.plot([base_pts[i, 0], platform_pts[i, 0]], 
                [base_pts[i, 1], platform_pts[i, 1]], 
                [base_pts[i, 2], platform_pts[i, 2]], 
                'gray', linestyle='-', linewidth=2, alpha=0.7)
    
    # Draw platform edges (rigid connections between platform joints)
    for i in range(6):
        j = (i + 1) % 6
        ax.plot([platform_pts[i, 0], platform_pts[j, 0]], 
                [platform_pts[i, 1], platform_pts[j, 1]], 
                [platform_pts[i, 2], platform_pts[j, 2]], 
                'red' if collapsed else 'green', linestyle='-', linewidth=3)
    
    # Draw base connections (imaginary base platform)
    for i in range(6):
        j = (i + 1) % 6
        ax.plot([base_pts[i, 0], base_pts[j, 0]], 
                [base_pts[i, 1], base_pts[j, 1]], 
                [base_pts[i, 2], base_pts[j, 2]], 
                'blue', linestyle='--', linewidth=1, alpha=0.3)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    
    # Equal aspect ratio
    max_range = max([
        np.max(np.ptp(base_pts[:, 0])), np.max(np.ptp(base_pts[:, 1])), 
        np.max(np.ptp(platform_pts[:, 0])), np.max(np.ptp(platform_pts[:, 1])),
        np.max(np.ptp(base_pts[:, 2])) + 5
    ])
    
    mid_x = (np.mean(base_pts[:, 0]) + np.mean(platform_pts[:, 0])) / 2
    mid_y = (np.mean(base_pts[:, 1]) + np.mean(platform_pts[:, 1])) / 2
    mid_z = (rest_center[2]) / 2
    
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(-5, rest_center[2] + 10)
    
    ax.legend()
    
    if filename:
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {filename}")
    
    return fig, ax


def visualize_results(results, num_examples=4):
    """Visualize sample results from the simulation"""
    import matplotlib.pyplot as plt
    
    # Show collapsed examples
    collapsed = results['collapsed']
    stable = results['stable']
    
    # Pick examples
    collapse_samples = collapsed[:min(num_examples//2, len(collapsed))]
    stable_samples = stable[:min(num_examples//2, len(stable))]
    
    total_plots = len(collapse_samples) + len(stable_samples)
    
    if total_plots == 0:
        print("No configurations to visualize")
        return
    
    fig, axes = plt.subplots(2, max(2, (total_plots + 1) // 2), 
                            figsize=(16, 10), subplot_kw={'projection': '3d'})
    axes = axes.flatten()
    
    plot_idx = 0
    
    # Helper function to plot on an axis
    def plot_on_ax(ax, base_pts, platform_pts, title, is_collapsed):
        ax.scatter(base_pts[:, 0], base_pts[:, 1], base_pts[:, 2], 
                   c='blue', s=50, marker='o')
        ax.scatter(platform_pts[:, 0], platform_pts[:, 1], platform_pts[:, 2], 
                   c='red' if is_collapsed else 'green', s=50, marker='^')
        
        for i in range(6):
            ax.plot([base_pts[i, 0], platform_pts[i, 0]], 
                    [base_pts[i, 1], platform_pts[i, 1]], 
                    [base_pts[i, 2], platform_pts[i, 2]], 
                    'gray', linewidth=1.5, alpha=0.6)
        
        for i in range(6):
            j = (i + 1) % 6
            ax.plot([platform_pts[i, 0], platform_pts[j, 0]], 
                    [platform_pts[i, 1], platform_pts[j, 1]], 
                    [platform_pts[i, 2], platform_pts[j, 2]], 
                    'red' if is_collapsed else 'green', linewidth=2)
        
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    
    for c in collapse_samples:
        if plot_idx < len(axes):
            base_pts = c['base_positions']
            z_disp = c['z_displacements']
            # Reconstruct platform for visualization
            ref_platform_at_origin = platform_joints - np.mean(platform_joints, axis=0)
            success, center, roll, pitch, yaw = forward_kinematics(
                base_pts, rest_leg_lengths, ref_platform_at_origin
            )
            platform_pts = compute_platform_pts_from_pose(
                center, roll, pitch, yaw, ref_platform_at_origin
            )
            plot_on_ax(axes[plot_idx], base_pts, platform_pts, 
                      f"Collapse #{c['test']}", True)
            plot_idx += 1
    
    for s in stable_samples:
        if plot_idx < len(axes):
            base_pts = s['base_positions']
            ref_platform_at_origin = platform_joints - np.mean(platform_joints, axis=0)
            success, center, roll, pitch, yaw = forward_kinematics(
                base_pts, rest_leg_lengths, ref_platform_at_origin
            )
            platform_pts = compute_platform_pts_from_pose(
                center, roll, pitch, yaw, ref_platform_at_origin
            )
            plot_on_ax(axes[plot_idx], base_pts, platform_pts, 
                      f"Stable #{s['test']}", False)
            plot_idx += 1
    
    # Hide unused subplots
    for i in range(plot_idx, len(axes)):
        axes[i].set_visible(False)
    
    plt.suptitle("Platform Simulation Results - Stable vs Collapsed", fontsize=14)
    plt.tight_layout()
    plt.savefig('d:/final/platform_simulation_results.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"\nSaved results visualization to platform_simulation_results.png")


# ============================================================
# 9. RUN THE MAIN SIMULATION
# ============================================================

if __name__ == "__main__":
    np.random.seed(42)  # For reproducibility
    
    # Run the simulation
    results = run_simulation(num_tests=5000, max_displacement=10.0, max_tilt_deg=60.0)
    
    # Analyze results
    num_collapsed, num_stable = analyze_results(results)
    
    # Visualize the initial configuration
    print(f"\n{'='*70}")
    print(f"Visualizing initial (rest) configuration...")
    print(f"{'='*70}")
    
    fig, ax = visualize_configuration(
        base_joints_initial, platform_joints,
        title="Initial Configuration (Rest Position)",
        collapsed=False, 
        filename='d:/final/platform_initial_config.png'
    )
    
    # Visualize some collapse examples
    if num_collapsed > 0:
        visualize_results(results, num_examples=4)
    
    # Summary
    print(f"\n{'='*70}")
    print(f"SUMMARY")
    print(f"{'='*70}")
    print(f"The platform is a rigid hexagon with 6 legs connecting to movable base points.")
    print(f"When base points move randomly along Z axis (0-{10} units),")
    print(f"the platform {'IS LIKELY TO COLLAPSE' if num_collapsed > num_stable else 'is generally stable'}.")
    print(f"")
    print(f"Out of {results['total']} random configurations:")
    print(f"  - Collapsed: {num_collapsed} cases ({100*num_collapsed/results['total']:.1f}%)")
    print(f"  - Stable:    {num_stable} cases ({100*num_stable/results['total']:.1f}%)")
    
    if num_collapsed > 0:
        worst = results['collapsed'][0]
        print(f"\nWorst case: Test #{worst['test']} - {worst['reason']}")
        print(f"  Base Z displacements: {worst['z_displacements']}")
