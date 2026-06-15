"""
Stewart Platform Collapse Simulation (Optimized)
=================================================
- 6 platform joints form a rigid hexagon
- 6 base joints can each move along Z-axis (0 to 10)
- Base i connected to Platform i by fixed-length rigid link
- Detect collapse when geometry is impossible or unstable
"""

import numpy as np
import math
import random
import sys

# ============================================================
# GEOMETRY
# ============================================================
PLAT = np.array([
    [8.526279, -7.232051, 38.951552],
    [10.526279, -3.767949, 38.951552],
    [2.000000, 11.000000, 38.951552],
    [-2.000000, 11.000000, 38.951552],
    [-10.526279, -3.767949, 38.951552],
    [-8.526279, -7.232051, 38.951552],
])

BASE0 = np.array([
    [3.100000, -12.600000, 0.000000],
    [12.461920, 3.615321, 0.000000],
    [9.361920, 8.984679, 0.000000],
    [-9.361920, 8.984679, 0.000000],
    [-12.461920, 3.615321, 0.000000],
    [-3.100000, -12.600000, 0.000000],
])

# Fixed leg lengths (rigid links)
LEG_LEN = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])

# Platform geometry
PLAT_CENTER = np.mean(PLAT, axis=0)
REF_P = PLAT - PLAT_CENTER  # centered at origin

# Platform edge distances (rigid hexagon edges)
EDGE_DISTS = []
for i in range(6):
    j = (i + 1) % 6
    EDGE_DISTS.append(np.linalg.norm(PLAT[i] - PLAT[j]))

def rot_mat(r, p, y):
    """ZYX Euler angles to rotation matrix"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def plat_points(c, r, p, y):
    """Get platform joint positions from pose"""
    R = rot_mat(r, p, y)
    return c + (R @ REF_P.T).T

def solve_fk(bases, max_it=100, tol=1e-5):
    """
    Newton-Raphson forward kinematics.
    Returns (converged, center, [roll,pitch,yaw], error)
    """
    c = np.array([0.0, 0.0, 35.0])
    rpy = np.array([0.0, 0.0, 0.0])
    
    for _ in range(max_it):
        pts = plat_points(c, rpy[0], rpy[1], rpy[2])
        L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = L - LEG_LEN
        en = np.linalg.norm(e)
        if en < tol:
            return True, c, rpy, en
        
        J = np.zeros((6,6))
        for i in range(6):
            d = pts[i] - bases[i]
            ln = np.linalg.norm(d)
            if ln < 1e-10:
                return False, c, rpy, 999
            u = d / ln
            J[i,:3] = u
            ri = pts[i] - c
            J[i,3:] = np.cross(ri, u)
        
        try:
            delta = np.linalg.solve(J, -e)
        except:
            delta = np.linalg.lstsq(J, -e, rcond=None)[0]
        
        c += 0.3 * delta[:3]
        rpy += 0.3 * delta[3:]
        
        if np.any(np.isnan(c)) or np.any(np.isnan(rpy)):
            return False, np.zeros(3), np.zeros(3), 999
    
    # Final check
    pts = plat_points(c, rpy[0], rpy[1], rpy[2])
    L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
    return (np.linalg.norm(L - LEG_LEN) < 1.0), c, rpy, np.linalg.norm(L - LEG_LEN)


def check(bases):
    """Check if platform has collapsed. Returns (is_collapsed, reason_msg)"""
    ok, c, rpy, err = solve_fk(bases)
    
    if not ok:
        return True, f"SOLVER FAILED (err={err:.3f})"
    
    pts = plat_points(c, rpy[0], rpy[1], rpy[2])
    
    # Tilt angle
    R = rot_mat(rpy[0], rpy[1], rpy[2])
    normal = R @ np.array([0,0,1])
    tilt = math.degrees(math.acos(np.clip(normal[2], -1, 1)))
    
    reasons = []
    if tilt > 60:
        reasons.append(f"TILT={tilt:.1f}deg")
    if c[2] < 0:
        reasons.append(f"Zc={c[2]:.1f}")
    min_z = np.min(pts[:,2])
    if min_z < -2:
        reasons.append(f"Zmin={min_z:.1f}")
    if abs(rpy[0]) > 1.3 or abs(rpy[1]) > 1.3:  # >~75 deg
        reasons.append(f"EXTREME_ROT")
    
    if reasons:
        return True, "COLLAPSE: " + "; ".join(reasons)
    
    return False, (f"OK: Zc={c[2]:.1f} tilt={tilt:.1f}deg err={err:.5f}")


def run():
    """Run the simulation"""
    results_file = 'd:/final/results.txt'
    N = 2000
    MAX_Z = 10.0
    
    with open(results_file, 'w') as f:
        f.write("="*70 + "\n")
        f.write("STEWART PLATFORM COLLAPSE SIMULATION\n")
        f.write("="*70 + "\n")
        f.write(f"Configurations: {N}\n")
        f.write(f"Max Z movement: {MAX_Z}\n")
        f.write("\n")
        f.write(f"Leg lengths (fixed):\n")
        for i, l in enumerate(LEG_LEN):
            f.write(f"  Leg {i+1}: {l:.6f}\n")
        f.write("\n")
        f.write(f"Platform: rigid hexagon\n")
        f.write(f"  Edge distances: {np.round(EDGE_DISTS, 4)}\n")
        f.write(f"  Center at rest: {PLAT_CENTER}\n")
        f.write("\n")
        
        # Verify solver
        ok, c, rpy, err = solve_fk(BASE0)
        f.write(f"Solver test (rest config): ok={ok}, err={err:.8f}\n")
        f.write(f"  Rest center: {c}\n")
        f.write("\n")
        f.write("="*70 + "\n")
        f.write("SIMULATION RUNNING\n")
        f.write("="*70 + "\n")
        
        np.random.seed(42)
        
        collapsed = []
        stable = []
        
        for test in range(1, N+1):
            z = np.random.uniform(0, MAX_Z, 6)
            bases = BASE0.copy()
            bases[:,2] = z
            
            coll, msg = check(bases)
            if coll:
                collapsed.append((test, z, msg, bases.copy()))
            else:
                stable.append((test, z, msg, bases.copy()))
            
            if test % 200 == 0:
                f.write(f"  Test {test}/{N}: collapsed={len(collapsed)}, stable={len(stable)}\n")
        
        f.write("\n")
        f.write("="*70 + "\n")
        f.write("RESULTS\n")
        f.write("="*70 + "\n")
        
        total = len(collapsed) + len(stable)
        col_pct = 100 * len(collapsed) / total
        f.write(f"Total: {total}\n")
        f.write(f"Stable: {len(stable)} ({100*len(stable)/total:.1f}%)\n")
        f.write(f"Collapsed: {len(collapsed)} ({col_pct:.1f}%)\n")
        f.write("\n")
        
        # Analyze collapse types
        types = {}
        for _, _, msg, _ in collapsed:
            if "SOLVER" in msg:
                t = "Solver failed"
            elif "TILT" in msg:
                t = "Excessive tilt"
            elif "Zc" in msg and "TILT" not in msg:
                t = "Platform too low"
            else:
                t = msg.split(":")[1].strip()[:30] if ":" in msg else "Other"
            types[t] = types.get(t, 0) + 1
        
        f.write("\nCollapse breakdown:\n")
        for t, cnt in sorted(types.items(), key=lambda x: -x[1]):
            f.write(f"  {t}: {cnt} ({100*cnt/len(collapsed):.1f}%)\n")
        f.write("\n")
        
        # Collapse examples
        f.write("-"*70 + "\n")
        f.write("COLLAPSE EXAMPLES (up to 10):\n")
        f.write("-"*70 + "\n")
        for idx, (test, z, msg, bases) in enumerate(collapsed[:10]):
            f.write(f"\n  #{idx+1} Test {test}:\n")
            f.write(f"    {msg}\n")
            f.write(f"    Z: [{z[0]:.3f}, {z[1]:.3f}, {z[2]:.3f}, {z[3]:.3f}, {z[4]:.3f}, {z[5]:.3f}]\n")
            
            ok2, c2, rpy2, err2 = solve_fk(bases)
            if ok2:
                p2 = plat_points(c2, rpy2[0], rpy2[1], rpy2[2])
                f.write(f"    Center: ({c2[0]:.2f}, {c2[1]:.2f}, {c2[2]:.2f})\n")
                f.write(f"    Joint Z range: [{np.min(p2[:,2]):.2f}, {np.max(p2[:,2]):.2f}]\n")
                f.write(f"    Tilt: {math.degrees(math.acos(np.clip((rot_mat(rpy2[0],rpy2[1],rpy2[2])@np.array([0,0,1]))[2], -1, 1))):.1f}deg\n")
        
        # Stable examples  
        f.write("\n")
        f.write("-"*70 + "\n")
        f.write("STABLE EXAMPLES (up to 5):\n")
        f.write("-"*70 + "\n")
        for idx, (test, z, msg, bases) in enumerate(stable[:5]):
            f.write(f"\n  #{idx+1} Test {test}:\n")
            f.write(f"    {msg}\n")
            f.write(f"    Z: [{z[0]:.3f}, {z[1]:.3f}, {z[2]:.3f}, {z[3]:.3f}, {z[4]:.3f}, {z[5]:.3f}]\n")
            ok2, c2, rpy2, _ = solve_fk(bases)
            if ok2:
                p2 = plat_points(c2, rpy2[0], rpy2[1], rpy2[2])
                f.write(f"    Joint Z range: [{np.min(p2[:,2]):.2f}, {np.max(p2[:,2]):.2f}]\n")
        
        f.write("\n")
        f.write("="*70 + "\n")
        f.write("SUMMARY\n")
        f.write("="*70 + "\n")
        f.write(f"Out of {total} random base Z configurations:\n")
        f.write(f"  - Collapsed: {len(collapsed)} ({col_pct:.1f}%)\n")
        f.write(f"  - Stable: {len(stable)} ({100*len(stable)/total:.1f}%)\n")
        f.write("\n")
        
        if col_pct > 50:
            f.write("CONCLUSION: The platform is HIGHLY UNSTABLE - more than half of random\n")
            f.write("base Z movements cause collapse. The fixed leg lengths (~39.7 units)\n")
            f.write("combined with the rigid hexagon geometry make it difficult to accommodate\n")
            f.write("uneven base Z displacements.\n")
        elif col_pct > 10:
            f.write("CONCLUSION: The platform has MODERATE stability - some random base\n")
            f.write("Z movements cause collapse, especially when base displacements are\n")
            f.write("highly uneven.\n")
        else:
            f.write("CONCLUSION: The platform is generally STABLE under random base Z movement.\n")
    
    print(f"\nDone! Results saved to {results_file}")
    
    # Also print summary to console
    print(f"\n=== QUICK SUMMARY ===")
    print(f"Total: {total}")
    print(f"Stable: {len(stable)} ({100*len(stable)/total:.1f}%)")
    print(f"Collapsed: {len(collapsed)} ({col_pct:.1f}%)")
    for t, cnt in sorted(types.items(), key=lambda x: -x[1]):
        print(f"  {t}: {cnt}")

if __name__ == "__main__":
    run()
