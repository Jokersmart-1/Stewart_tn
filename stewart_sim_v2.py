"""
Stewart Platform Collapse Simulation v2
========================================
- 6 platform joints: rigid hexagon (1-2-3-4-5-6-1)
- 6 base joints: each can move along Z axis (0 to 10)
- Base i connected to Platform i by fixed-length rigid link (~39.69)
- Detect when platform collapses due to impossible geometry
"""

import numpy as np
import math
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

LEG = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])
C0 = np.mean(PLAT, axis=0)
REF = PLAT - C0

# Hexagon edge lengths
EDGES = [np.linalg.norm(PLAT[i] - PLAT[(i+1)%6]) for i in range(6)]

def R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,           cp*cr]])

def P(c, r, p, y):
    return c + (R(r,p,y) @ REF.T).T

def fk(bases, n=100, tol=1e-5):
    c = np.array([0.0, 0.0, 35.0])
    a = np.array([0.0, 0.0, 0.0])
    
    for _ in range(n):
        pts = P(c, a[0], a[1], a[2])
        L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = L - LEG
        en = np.linalg.norm(e)
        if en < tol:
            return True, c, a, en
        
        J = np.zeros((6,6))
        for i in range(6):
            d = pts[i] - bases[i]
            ln = np.linalg.norm(d)
            if ln < 1e-10:
                return False, c, a, 999
            u = d / ln
            J[i,:3] = u
            J[i,3:] = np.cross(pts[i]-c, u)
        
        try:
            delta = np.linalg.solve(J, -e)
        except:
            delta = np.linalg.lstsq(J, -e, rcond=None)[0]
        
        c += 0.3 * delta[:3]
        a += 0.3 * delta[3:]
        
        if np.any(np.isnan(c)) or np.any(np.isnan(a)):
            return False, np.zeros(3), np.zeros(3), 999
    
    pts = P(c, a[0], a[1], a[2])
    L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
    err = np.linalg.norm(L - LEG)
    return (err < 0.5), c, a, err


def test(bases, limit_tilt=60.0):
    ok, c, a, err = fk(bases)
    
    if not ok:
        return True, "SOLVER_FAIL"
    
    pts = P(c, a[0], a[1], a[2])
    n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
    tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
    minz = np.min(pts[:,2])
    
    if tilt > limit_tilt:
        return True, f"TILT_{tilt:.1f}"
    if minz < -2:
        return True, f"ZMIN_{minz:.1f}"
    if abs(a[0]) > 1.3 or abs(a[1]) > 1.3:
        return True, f"EXTREME_ROT"
    
    return False, f"OK_z{c[2]:.1f}_t{tilt:.1f}"


def run():
    N = 2000
    MAX_Z = 10.0
    np.random.seed(42)
    
    out = sys.stdout
    out.write("="*70 + "\n")
    out.write("STEWART PLATFORM COLLAPSE SIMULATION\n")
    out.write("="*70 + "\n")
    out.write(f"Configurations: {N}\n")
    out.write(f"Max base Z movement: {MAX_Z}\n")
    out.write("\n")
    out.write(f"Leg lengths (fixed): {np.round(LEG, 6)}\n")
    out.write(f"Platform edges (rigid): {np.round(EDGES, 4)}\n")
    out.write(f"Platform center at rest: {C0}\n")
    out.write("\n")
    
    # Verify solver
    ok, c, a, err = fk(BASE0)
    out.write(f"Solver test (rest config): ok={ok}, err={err:.8f}\n")
    out.write(f"  Solved center: ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})\n")
    out.write("\n")
    
    collapsed = []
    stable = []
    
    for test_idx in range(1, N+1):
        z = np.random.uniform(0, MAX_Z, 6)
        bases = BASE0.copy()
        bases[:,2] = z
        
        is_bad, msg = test(bases)
        if is_bad:
            collapsed.append((test_idx, z, msg, bases))
        else:
            stable.append((test_idx, z, msg, bases))
        
        if test_idx % 200 == 0:
            out.write(f"  Progress: {test_idx}/{N}  "
                      f"collapsed={len(collapsed)} stable={len(stable)}\n")
    
    # RESULTS
    total = len(collapsed) + len(stable)
    cpct = 100 * len(collapsed) / total
    
    out.write("\n" + "="*70 + "\n")
    out.write("RESULTS\n")
    out.write("="*70 + "\n")
    out.write(f"Total: {total}\n")
    out.write(f"Stable:   {len(stable):5d}  ({100*len(stable)/total:.1f}%)\n")
    out.write(f"Collapsed: {len(collapsed):5d}  ({cpct:.1f}%)\n")
    out.write("\n")
    
    # Collapse type breakdown
    types = {}
    for _, _, msg, _ in collapsed:
        t = msg.split("_")[0]
        types[t] = types.get(t, 0) + 1
    
    out.write("Collapse by type:\n")
    for t, n in sorted(types.items(), key=lambda x: -x[1]):
        out.write(f"  {t}: {n} ({100*n/len(collapsed):.1f}%)\n")
    out.write("\n")
    
    # Examples
    out.write("-"*70 + "\n")
    out.write("COLLAPSE EXAMPLES (first 8):\n")
    out.write("-"*70 + "\n")
    for idx, (ti, z, msg, bases) in enumerate(collapsed[:8]):
        ok2, c2, a2, err2 = fk(bases)
        tilt2 = math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2], -1, 1))) if ok2 else 0
        out.write(f"\n  #{idx+1} Test {ti}:\n")
        out.write(f"    {msg}\n")
        out.write(f"    Z: [{z[0]:.3f} {z[1]:.3f} {z[2]:.3f} {z[3]:.3f} {z[4]:.3f} {z[5]:.3f}]\n")
        if ok2:
            p2 = P(c2, a2[0], a2[1], a2[2])
            out.write(f"    Center: ({c2[0]:.1f}, {c2[1]:.1f}, {c2[2]:.1f})  Tilt: {tilt2:.1f}deg\n")
            out.write(f"    Joint Z range: [{np.min(p2[:,2]):.1f}, {np.max(p2[:,2]):.1f}]\n")
    
    out.write("\n")
    out.write("-"*70 + "\n")
    out.write("STABLE EXAMPLES (first 5):\n")
    out.write("-"*70 + "\n")
    for idx, (ti, z, msg, bases) in enumerate(stable[:5]):
        ok2, c2, a2, _ = fk(bases)
        tilt2 = math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2], -1, 1)))
        out.write(f"\n  #{idx+1} Test {ti}:\n")
        out.write(f"    Z: [{z[0]:.3f} {z[1]:.3f} {z[2]:.3f} {z[3]:.3f} {z[4]:.3f} {z[5]:.3f}]\n")
        out.write(f"    Center: ({c2[0]:.1f}, {c2[1]:.1f}, {c2[2]:.1f})  Tilt: {tilt2:.1f}deg\n")
    
    out.write("\n")
    out.write("="*70 + "\n")
    out.write("SUMMARY\n")
    out.write("="*70 + "\n")
    out.write(f"The platform has 6 rigid legs (length ~39.69) connecting base to platform.\n")
    out.write(f"The platform itself is a rigid hexagon (edges ~{EDGES[0]:.3f}).\n")
    out.write(f"\n")
    out.write(f"When {N} random base Z configurations were tested (Z=0 to {MAX_Z}):\n")
    out.write(f"  - {len(collapsed)} configurations LED TO COLLAPSE ({cpct:.1f}%)\n")
    out.write(f"  - {len(stable)} configurations STAYED STABLE ({100*len(stable)/total:.1f}%)\n")
    out.write(f"\n")
    
    if cpct > 0:
        out.write("CONCLUSION: The platform CAN COLLAPSE when base points move unevenly\n")
        out.write("along the Z axis. The rigid links cannot accommodate all random\n")
        out.write("configurations - the geometry becomes impossible and the platform\n")
        out.write("either cannot be positioned (solver fails), tilts excessively,\n")
        out.write("or hits the ground.\n")
    else:
        out.write("CONCLUSION: The platform is stable for all tested configurations.\n")


if __name__ == "__main__":
    run()
