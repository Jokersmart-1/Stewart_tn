"""
Stewart Platform - Comprehensive Simulation with Visualization
==============================================================
"""
import numpy as np
import math
import time
import random
import os

# ============================================================
# GEOMETRY & CONSTANTS
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

LEG_LENS = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])
C0 = np.mean(PLAT, axis=0)
REF = PLAT - C0

# Platform edge connections (rigid hexagon)
CONN = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)]

def R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,   cp*sr,           cp*cr]
    ])

def P(c, r, p, y):
    return c + (R(r, p, y) @ REF.T).T

def fk(bases, n=20, tol=1e-3):
    """Fast forward kinematics"""
    c = np.array([0., 0., 37.])
    a = np.array([0., 0., 0.])
    
    for _ in range(n):
        pts = P(c, a[0], a[1], a[2])
        L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = L - LEG_LENS
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
            J[i,3:] = np.cross(pts[i] - c, u)
        
        try:
            delta = np.linalg.solve(J, -e)
        except:
            delta = np.linalg.lstsq(J, -e, rcond=None)[0]
        
        c += delta[:3]
        a += delta[3:]
        
        if np.any(np.isnan(c)) or np.any(np.isnan(a)):
            return False, np.zeros(3), np.zeros(3), 999
    
    pts = P(c, a[0], a[1], a[2])
    L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
    err = np.linalg.norm(L - LEG_LENS)
    return (err < 1.0), c, a, err

def check_config(bases):
    """Returns (is_collapsed, details_dict)"""
    ok, c, a, err = fk(bases)
    
    if not ok:
        return True, {'reason': 'Solver failed', 'err': err}
    
    pts = P(c, a[0], a[1], a[2])
    n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
    tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
    minz = np.min(pts[:,2])
    
    reasons = []
    if tilt > 60:
        reasons.append(f"Excessive tilt ({tilt:.1f}°)")
    if c[2] < 0:
        reasons.append(f"Center below ground ({c[2]:.1f})")
    if minz < -2:
        reasons.append(f"Joint below ground ({minz:.1f})")
    
    if reasons:
        return True, {'reason': '; '.join(reasons), 'tilt': tilt, 'center_z': c[2], 'min_z': minz}
    
    return False, {'center': c, 'roll': a[0], 'pitch': a[1], 'yaw': a[2], 
                   'tilt': tilt, 'min_z': minz}


def run_comprehensive():
    """Run comprehensive simulation and write detailed report"""
    np.random.seed(42)
    
    # Test different max displacements
    displacements = [5.0, 10.0, 15.0, 20.0]
    all_results = {}
    
    for max_z in displacements:
        N = 2000
        coll = 0
        types = {}
        
        for _ in range(N):
            z = np.random.uniform(0, max_z, 6)
            bases = BASE0.copy()
            bases[:,2] = z
            bad, _ = check_config(bases)
            if bad:
                coll += 1
                # Check if it's solver fail or other
                ok, c, a, err = fk(bases)
                if not ok:
                    key = "Solver failed"
                else:
                    pts = P(c, a[0], a[1], a[2])
                    n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
                    tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
                    if tilt > 60:
                        key = "Excessive tilt"
                    elif np.min(pts[:,2]) < -2:
                        key = "Joint below ground"
                    else:
                        key = "Other"
                types[key] = types.get(key, 0) + 1
        
        all_results[max_z] = {
            'total': N,
            'collapsed': coll,
            'stable': N - coll,
            'pct': 100 * coll / N,
            'types': types
        }
        
    return all_results


def run_main_simulation():
    """Run the main 5000-config simulation and save results"""
    N = 5000
    MAX_Z = 10.0
    np.random.seed(42)
    
    out_lines = []
    def w(s=""):
        out_lines.append(s)
        print(s)
    
    w("="*80)
    w("STEWART PLATFORM - PHYSICAL COLLAPSE SIMULATION")
    w("="*80)
    w("")
    w("SYSTEM DESCRIPTION:")
    w("  - 6 Platform joints: rigid hexagon at Z=38.95")
    w("    Connections: 1-2-3-4-5-6-1")
    w(f"    Joints: {PLAT.tolist()}")
    w("")
    w("  - 6 Base joints: hexagon at Z=0, each can move along Z (0-10)")
    w(f"    Joints: {BASE0.tolist()}")
    w("")
    w("  - Base i is connected to Platform i by fixed-length rigid link")
    w(f"    Leg lengths: {np.round(LEG_LENS, 4)}")
    w(f"    Platform center at rest: {C0}")
    w("")
    
    # Verify solver
    ok, c, a, err = fk(BASE0)
    w(f"Solver verification (rest config):")
    w(f"  Converged: {ok}, Error: {err:.8f}")
    w(f"  Solved center: ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})")
    w("")
    
    t0 = time.time()
    
    collapsed = []
    stable = []
    
    for i in range(1, N+1):
        z = np.random.uniform(0, MAX_Z, 6)
        bases = BASE0.copy()
        bases[:,2] = z
        
        bad, info = check_config(bases)
        
        rec = {
            'test': i,
            'z': z.copy(),
            'bases': bases.copy(),
            'info': info
        }
        
        if bad:
            collapsed.append(rec)
        else:
            stable.append(rec)
        
        if i % 1000 == 0:
            w(f"  Progress: {i}/{N} ({(time.time()-t0):.1f}s) - "
              f"collapsed: {len(collapsed)}, stable: {len(stable)}")
    
    t1 = time.time()
    total = len(collapsed) + len(stable)
    cpct = 100 * len(collapsed) / total
    
    w("")
    w("="*80)
    w("RESULTS")
    w("="*80)
    w(f"Total configurations tested: {total}")
    w(f"Simulation time: {t1-t0:.1f}s")
    w(f"")
    w(f"  STABLE:   {len(stable):5d} configurations ({100*len(stable)/total:.2f}%)")
    w(f"  COLLAPSED: {len(collapsed):5d} configurations ({cpct:.2f}%)")
    w("")
    
    # Analyze collapse reasons
    reasons = {}
    for c in collapsed:
        r = c['info']['reason']
        if 'Solver' in r:
            cat = 'Geometric impossibility (solver failed)'
        elif 'tilt' in r:
            cat = 'Excessive tilt'
        elif 'ground' in r:
            cat = 'Joint below ground level'
        else:
            cat = r
        reasons[cat] = reasons.get(cat, 0) + 1
    
    w("COLLAPSE BREAKDOWN:")
    for r, n in sorted(reasons.items(), key=lambda x: -x[1]):
        w(f"  {r}: {n} cases ({100*n/len(collapsed):.1f}%)")
    w("")
    
    # Analyze Z displacement patterns in collapses vs stable
    w("PATTERN ANALYSIS:")
    stable_z = np.array([s['z'] for s in stable])
    coll_z = np.array([c['z'] for c in collapsed])
    
    if len(stable_z) > 0:
        w(f"  Stable configs - mean Z: {np.mean(stable_z):.3f}, std: {np.std(stable_z):.3f}")
        w(f"  Stable configs - max Z diff (range): {np.mean(np.max(stable_z,1)-np.min(stable_z,1)):.3f}")
    if len(coll_z) > 0:
        w(f"  Collapsed configs - mean Z: {np.mean(coll_z):.3f}, std: {np.std(coll_z):.3f}")
        w(f"  Collapsed configs - max Z diff (range): {np.mean(np.max(coll_z,1)-np.min(coll_z,1)):.3f}")
    w("")
    
    # Collapse examples
    w("-"*80)
    w("COLLAPSE EXAMPLES:")
    w("-"*80)
    for idx, c in enumerate(collapsed[:8]):
        w(f"\n  #{idx+1} Test {c['test']}:")
        w(f"    Reason: {c['info']['reason']}")
        w(f"    Z: [{c['z'][0]:.2f} {c['z'][1]:.2f} {c['z'][2]:.2f} {c['z'][3]:.2f} {c['z'][4]:.2f} {c['z'][5]:.2f}]")
        
        # Try FK anyway
        ok2, c2, a2, _ = fk(c['bases'])
        if ok2:
            p2 = P(c2, a2[0], a2[1], a2[2])
            n2 = R(a2[0], a2[1], a2[2]) @ np.array([0,0,1])
            t2 = math.degrees(math.acos(np.clip(n2[2], -1, 1)))
            w(f"    Center: ({c2[0]:.1f}, {c2[1]:.1f}, {c2[2]:.1f}), Tilt: {t2:.0f}°")
            w(f"    Joint Z range: [{np.min(p2[:,2]):.2f}, {np.max(p2[:,2]):.2f}]")
    
    # Stable examples
    w("\n" + "-"*80)
    w("STABLE EXAMPLES:")
    w("-"*80)
    for idx, s in enumerate(stable[:5]):
        c2 = s['info']['center']
        t2 = s['info']['tilt']
        p2 = P(c2, s['info']['roll'], s['info']['pitch'], s['info']['yaw'])
        w(f"\n  #{idx+1} Test {s['test']}:")
        w(f"    Z: [{s['z'][0]:.2f} {s['z'][1]:.2f} {s['z'][2]:.2f} {s['z'][3]:.2f} {s['z'][4]:.2f} {s['z'][5]:.2f}]")
        w(f"    Center: ({c2[0]:.1f}, {c2[1]:.1f}, {c2[2]:.1f}), Tilt: {t2:.0f}°")
        w(f"    Joint Z range: [{np.min(p2[:,2]):.2f}, {np.max(p2[:,2]):.2f}]")
    
    # Multi-displacement analysis
    w("\n" + "-"*80)
    w("SENSITIVITY ANALYSIS (varying max Z displacement):")
    w("-"*80)
    zm_results = run_comprehensive()
    for dz, res in sorted(zm_results.items()):
        w(f"  Max Z={dz:5.1f}: {res['stable']:5d} stable, {res['collapsed']:5d} collapsed "
          f"({res['pct']:.1f}% collapse rate)")
    
    # Final analysis
    w("\n" + "="*80)
    w("PHYSICAL ANALYSIS")
    w("="*80)
    w("")
    w("The Stewart platform is a parallel kinematic mechanism where:")
    w("  1. The platform is a RIGID BODY (hexagon, edges fixed)")
    w("  2. Each of 6 legs has a FIXED LENGTH (~39.69 units)")
    w("  3. Base joints can only move ALONG Z AXIS (0 to 10)")
    w("")
    w("For the platform to exist in a valid configuration:")
    w("  - The 6 spheres (radius = leg length) centered at each base joint")
    w("    must have a common intersection with the platform's rigid")
    w("    hexagon geometry")
    w("")
    w("When this intersection doesn't exist (geometric impossibility),")
    w("the platform is said to have COLLAPSED.")
    w("")
    
    if cpct < 5:
        stability = "HIGH - The platform is very robust to random base movements."
    elif cpct < 15:
        stability = "MODERATE - Some configurations cause collapse, especially with uneven displacement."
    elif cpct < 30:
        stability = "LOW - The platform frequently collapses under random base movement."
    else:
        stability = "VERY LOW - The platform is highly unstable under random base Z movement."
    
    w(f"STABILITY RATING: {stability}")
    w(f"")
    w(f"Collapse rate: {cpct:.1f}% ({len(collapsed)}/{total})")
    
    # Save to file
    with open('d:/final/stewart_complete_report.txt', 'w') as f:
        f.write('\n'.join(out_lines))
    
    w(f"\nFull report saved to d:/final/stewart_complete_report.txt")
    
    return collapsed, stable, out_lines


if __name__ == "__main__":
    collapsed, stable, _ = run_main_simulation()
    
    # ============================================================
    # VISUALIZATION
    # ============================================================
    print("\nGenerating visualization...")
    
    try:
        import matplotlib
        matplotlib.use('Agg')  # non-interactive backend
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        # FIGURE 1: Initial configuration
        fig1 = plt.figure(figsize=(12, 10))
        ax1 = fig1.add_subplot(111, projection='3d')
        
        # Platform
        ax1.scatter(PLAT[:,0], PLAT[:,1], PLAT[:,2], c='green', s=100, marker='^', label='Platform')
        for i,j in CONN:
            ax1.plot([PLAT[i,0], PLAT[j,0]], [PLAT[i,1], PLAT[j,1]], [PLAT[i,2], PLAT[j,2]], 
                    'green', linewidth=3)
        
        # Base
        ax1.scatter(BASE0[:,0], BASE0[:,1], BASE0[:,2], c='blue', s=80, marker='o', label='Base')
        for i,j in CONN:
            ax1.plot([BASE0[i,0], BASE0[j,0]], [BASE0[i,1], BASE0[j,1]], [BASE0[i,2], BASE0[j,2]], 
                    'blue', linewidth=1, alpha=0.3)
        
        # Legs
        for i in range(6):
            ax1.plot([BASE0[i,0], PLAT[i,0]], [BASE0[i,1], PLAT[i,1]], [BASE0[i,2], PLAT[i,2]], 
                    'gray', linewidth=2, alpha=0.7)
        
        ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
        ax1.set_title('Stewart Platform - Initial Configuration (Rest)')
        max_r = 20
        ax1.set_xlim(-max_r, max_r); ax1.set_ylim(-max_r, max_r); ax1.set_zlim(-5, 50)
        ax1.legend()
        fig1.savefig('d:/final/fig1_initial_config.png', dpi=150, bbox_inches='tight')
        print("  Saved fig1_initial_config.png")
        
        # FIGURE 2: Collapse example
        if len(collapsed) > 0:
            fig2 = plt.figure(figsize=(12, 10))
            ax2 = fig2.add_subplot(111, projection='3d')
            
            ex = collapsed[0]
            bases = ex['bases']
            
            # Try FK
            ok, c, a, _ = fk(bases)
            if ok:
                plat = P(c, a[0], a[1], a[2])
            else:
                # Use degenerate
                plat = PLAT.copy()
                plat[:,2] = 0
            
            ax2.scatter(bases[:,0], bases[:,1], bases[:,2], c='blue', s=80, marker='o', label='Base')
            ax2.scatter(plat[:,0], plat[:,1], plat[:,2], c='red', s=100, marker='^', label='Platform (failed)')
            
            for i in range(6):
                ax2.plot([bases[i,0], plat[i,0]], [bases[i,1], plat[i,1]], [bases[i,2], plat[i,2]], 
                        'gray', linewidth=2, alpha=0.7)
            
            for i,j in CONN:
                ax2.plot([plat[i,0], plat[j,0]], [plat[i,1], plat[j,1]], [plat[i,2], plat[j,2]], 
                        'red', linewidth=3)
            
            ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
            ax2.set_title(f'Collapse Example: {ex["info"]["reason"]}')
            ax2.set_xlim(-20, 20); ax2.set_ylim(-20, 20); ax2.set_zlim(-5, 50)
            ax2.legend()
            fig2.savefig('d:/final/fig2_collapse_example.png', dpi=150, bbox_inches='tight')
            print("  Saved fig2_collapse_example.png")
        
        # FIGURE 3: Stable example
        if len(stable) > 0:
            fig3 = plt.figure(figsize=(12, 10))
            ax3 = fig3.add_subplot(111, projection='3d')
            
            ex = stable[0]
            bases = ex['bases']
            inf = ex['info']
            c3 = inf['center']
            plat = P(c3, inf['roll'], inf['pitch'], inf['yaw'])
            
            ax3.scatter(bases[:,0], bases[:,1], bases[:,2], c='blue', s=80, marker='o', label='Base')
            ax3.scatter(plat[:,0], plat[:,1], plat[:,2], c='green', s=100, marker='^', label='Platform')
            
            for i in range(6):
                ax3.plot([bases[i,0], plat[i,0]], [bases[i,1], plat[i,1]], [bases[i,2], plat[i,2]], 
                        'gray', linewidth=2, alpha=0.7)
            
            for i,j in CONN:
                ax3.plot([plat[i,0], plat[j,0]], [plat[i,1], plat[j,1]], [plat[i,2], plat[j,2]], 
                        'green', linewidth=3)
            
            ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')
            ax3.set_title(f'Stable Example: Zc={c3[2]:.1f}, Tilt={inf["tilt"]:.0f}°')
            ax3.set_xlim(-20, 20); ax3.set_ylim(-20, 20); ax3.set_zlim(-5, 50)
            ax3.legend()
            fig3.savefig('d:/final/fig3_stable_example.png', dpi=150, bbox_inches='tight')
            print("  Saved fig3_stable_example.png")
        
        # FIGURE 4: Z displacement analysis
        fig4, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # Histogram of stable Z values
        if len(stable) > 0:
            stable_z = np.array([s['z'] for s in stable]).flatten()
            axes[0].hist(stable_z, bins=20, alpha=0.7, color='green', label='Stable')
        if len(collapsed) > 0:
            coll_z = np.array([c['z'] for c in collapsed]).flatten()
            axes[0].hist(coll_z, bins=20, alpha=0.7, color='red', label='Collapsed')
        axes[0].set_xlabel('Base Z displacement')
        axes[0].set_ylabel('Count')
        axes[0].set_title('Distribution of Base Z Displacements')
        axes[0].legend()
        
        # Bar chart
        categories = []
        counts = []
        for c in collapsed[:1]:
            pass
        # Collapse reasons
        reasons = {}
        for c in collapsed:
            r = c['info']['reason'].split('(')[0].strip()[:30]
            reasons[r] = reasons.get(r, 0) + 1
        
        cats = list(reasons.keys())[:5]
        vals = [reasons[k] for k in cats]
        axes[1].bar(cats, vals, color='red', alpha=0.7)
        axes[1].set_xlabel('Collapse Reason')
        axes[1].set_ylabel('Count')
        axes[1].set_title('Collapse Reasons')
        axes[1].tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        fig4.savefig('d:/final/fig4_analysis.png', dpi=150, bbox_inches='tight')
        print("  Saved fig4_analysis.png")
        
        # FIGURE 5: Sensitivity analysis
        fig5, ax5 = plt.subplots(figsize=(10, 6))
        zm_results = run_comprehensive()
        dzs = sorted(zm_results.keys())
        rates = [zm_results[d]['pct'] for d in dzs]
        
        ax5.plot(dzs, rates, 'ro-', linewidth=2, markersize=8)
        ax5.set_xlabel('Max Base Z Displacement')
        ax5.set_ylabel('Collapse Rate (%)')
        ax5.set_title('Collapse Rate vs Max Z Displacement')
        ax5.grid(True, alpha=0.3)
        
        for d, r in zip(dzs, rates):
            ax5.annotate(f'{r:.1f}%', (d, r), textcoords="offset points", xytext=(0,10), ha='center')
        
        fig5.savefig('d:/final/fig5_sensitivity.png', dpi=150, bbox_inches='tight')
        print("  Saved fig5_sensitivity.png")
        
        print("\nAll visualizations saved!")
        
    except ImportError:
        print("  matplotlib not available - skipping visualization")
    except Exception as e:
        print(f"  Visualization error: {e}")
