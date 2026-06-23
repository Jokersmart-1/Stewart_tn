"""
Stewart Platform - Mo phong voi tham so toi uu
=============================================
BEST PARAMETERS tim duoc:
  baseRadius=10, platformRadius=7
  baseDistance=6, platformDistance=2
  legLength=16
"""
import numpy as np, math, time, random
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.animation import FuncAnimation

# ============ GEOMETRY ============
def build_platform(pr, pd, z=0):
    po = math.atan2(pd/2, pr)
    R = math.sqrt(pr**2 + (pd/2)**2)
    return np.array([
        [R*math.sin(math.pi/3-po), -R*math.cos(math.pi/3-po), z],
        [R*math.sin(math.pi/3+po), -R*math.cos(math.pi/3+po), z],
        [pd/2, pr, z], [-pd/2, pr, z],
        [-R*math.sin(math.pi/3+po), -R*math.cos(math.pi/3+po), z],
        [-R*math.sin(math.pi/3-po), -R*math.cos(math.pi/3-po), z],
    ])

def build_base(br, bd):
    bo = math.atan2(bd/2, br)
    R = math.sqrt(br**2 + (bd/2)**2)
    return np.array([
        [bd/2, -br, 0],
        [R*math.sin(math.pi/3+bo), R*math.cos(math.pi/3+bo), 0],
        [R*math.sin(math.pi/3-bo), R*math.cos(math.pi/3-bo), 0],
        [-R*math.sin(math.pi/3-bo), R*math.cos(math.pi/3-bo), 0],
        [-R*math.sin(math.pi/3+bo), R*math.cos(math.pi/3+bo), 0],
        [-bd/2, -br, 0],
    ])

def Rm(rr,pp,yy):
    cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp); cy,sy=math.cos(yy),math.sin(yy)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr]])

def solve_platform(base_pts, plat_ref, L):
    """Newton-Raphson solver goc: tim (R, t) thoa leg length constraint"""
    rs = plat_ref
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 40.0])  # [rx,ry,rz, tx,ty,tz]
    for it in range(100):
        Rmat = Rm(x[0], x[1], x[2])
        t = x[3:6]
        q = (Rmat @ rs.T).T
        
        f = np.array([np.linalg.norm(t + q[i] - base_pts[i])**2 - L**2 for i in range(6)])
        err = np.sum(f**2)
        if err < 1e-6: break
        
        J = np.zeros((6,6))
        for i in range(6):
            v = t + q[i] - base_pts[i]
            dq_drx = Rm(x[0]+0.001, x[1], x[2]) @ rs[i] - q[i]
            dq_dry = Rm(x[0], x[1]+0.001, x[2]) @ rs[i] - q[i]
            dq_drz = Rm(x[0], x[1], x[2]+0.001) @ rs[i] - q[i]
            J[i,:3] = 2*v @ np.column_stack([dq_drx/0.001, dq_dry/0.001, dq_drz/0.001])
            J[i,3:] = 2*v
        
        try: dx = -np.linalg.lstsq(J, f, rcond=None)[0]
        except: break
        x += dx * 0.5
    
    Rmat = Rm(x[0], x[1], x[2])
    t = x[3:6]
    q = (Rmat @ rs.T).T
    err = np.sum([(np.linalg.norm(t+q[i]-base_pts[i])-L)**2 for i in range(6)])
    return err < 0.1, Rmat, t, err

# ============ THAM SO TOI UU ============
BR_OPT = 10; PR_OPT = 7; BD_OPT = 6; PD_OPT = 2; LL_OPT = 16
# Tham so MAC DINH (de so sanh)
BR_DEF = 14.7; PR_DEF = 8.5; BD_DEF = 11.2; PD_DEF = 4.8; LL_DEF = 18

print("="*70)
print("MO PHONG SAP - THAM SO TOI UU vs MAC DINH")
print("="*70)

# Xay dung hinh hoc
plat_opt = build_platform(PR_OPT, PD_OPT, 0)
base_opt = build_base(BR_OPT, BD_OPT)
ref_opt = plat_opt - np.mean(plat_opt, axis=0)
L_opt_rest = np.mean([np.linalg.norm(plat_opt[i]-base_opt[i]) for i in range(6)])

plat_def = build_platform(PR_DEF, PD_DEF, 0)
base_def = build_base(BR_DEF, BD_DEF)
ref_def = plat_def - np.mean(plat_def, axis=0)
L_def_rest = np.mean([np.linalg.norm(plat_def[i]-base_def[i]) for i in range(6)])

print(f"\nTham so TOI UU: BR={BR_OPT}, PR={PR_OPT}, BD={BD_OPT}, PD={PD_OPT}, LL={LL_OPT}")
print(f"  L_rest = {L_opt_rest:.3f}  |LL-L_rest|/L_rest = {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.1f}%")
print(f"\nTham so MAC DINH: BR={BR_DEF}, PR={PR_DEF}, BD={BD_DEF}, PD={PD_DEF}, LL={LL_DEF}")
print(f"  L_rest = {L_def_rest:.3f}  |LL-L_rest|/L_rest = {abs(LL_DEF-L_def_rest)/L_def_rest*100:.1f}%")

# ============ MO PHONG NHIEU LAN ============
np.random.seed(12345)
N = 2000
print(f"\nMo phong {N} cau hinh ngau nhien (Z random 0-10mm)...")

t0 = time.time()
col_opt = 0; col_def = 0
col_cases_opt = []; col_cases_def = []

for i in range(N):
    z = np.random.uniform(0, 10, 6)
    
    # TOI UU
    b_opt = base_opt.copy(); b_opt[:,2] = z
    ok_opt, _, _, _ = solve_platform(b_opt, ref_opt, LL_OPT)
    if not ok_opt: 
        col_opt += 1
        if len(col_cases_opt) < 5: col_cases_opt.append(z.copy())
    
    # MAC DINH
    b_def = base_def.copy(); b_def[:,2] = z
    ok_def, _, _, _ = solve_platform(b_def, ref_def, LL_DEF)
    if not ok_def: 
        col_def += 1
        if len(col_cases_def) < 5: col_cases_def.append(z.copy())
    
    if (i+1) % 200 == 0:
        print(f"  {i+1}/{N}... opt: {col_opt} coll, def: {col_def} coll")

print(f"\nKET QUA:")
print(f"  Tham so TOI UU: {col_opt}/{N} = {col_opt*100/N:.1f}% bi sap")
print(f"  Tham so MAC DINH: {col_def}/{N} = {col_def*100/N:.1f}% bi sap")
print(f"  Cai thien: {(col_def-col_opt)/max(col_def,1)*100:.0f}% giam sap")

# ============ VI DU SAP ============
print(f"\nVi du collapse (TOI UU):")
for i, z in enumerate(col_cases_opt[:3]):
    print(f"  Case {i+1}: Z={z}")
print(f"\nVi du collapse (MAC DINH):")
for i, z in enumerate(col_cases_def[:3]):
    print(f"  Case {i+1}: Z={z}")

print(f"\nThoi gian chay: {time.time()-t0:.1f}s")

# ============ VE BIEU DO ============
print(f"\nDang ve bieu do...")

# 1. SO SANH COLLAPSE RATE
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
fig.suptitle('Stewart Platform - So Sanh Tham So Toi Uu vs Mac Dinh', fontsize=16, fontweight='bold')

ax = axes[0,0]
bars = ax.bar(['Mac Dinh', 'Toi Uu'], [col_def*100/N, col_opt*100/N], 
              color=['#e74c3c', '#2ecc71'], width=0.5, edgecolor='black', linewidth=2)
ax.set_ylabel('Ty le sap (%)', fontsize=12)
ax.set_title('Ty le bi sap (2000 cau hinh)', fontsize=13, fontweight='bold')
ax.set_ylim(0, max(100, max(col_def, col_opt)*100/N*1.2))
for bar, val in zip(bars, [col_def*100/N, col_opt*100/N]):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{val:.1f}%', 
            ha='center', fontsize=12, fontweight='bold')
ax.grid(axis='y', alpha=0.3)

# 2. LEG LENGTH ANALYSIS
ax = axes[0,1]
legs_opt = [np.linalg.norm(plat_opt[i]-base_opt[i]) for i in range(6)]
legs_def = [np.linalg.norm(plat_def[i]-base_def[i]) for i in range(6)]
x = np.arange(6)
w = 0.35
ax.bar(x-w/2, legs_def, w, label='Mac Dinh', color='#e74c3c', edgecolor='black')
ax.bar(x+w/2, legs_opt, w, label='Toi Uu', color='#2ecc71', edgecolor='black')
ax.axhline(y=LL_DEF, color='#e74c3c', linestyle='--', alpha=0.5, label=f'LL={LL_DEF}')
ax.axhline(y=LL_OPT, color='#2ecc71', linestyle='--', alpha=0.5, label=f'LL={LL_OPT}')
ax.set_xlabel('Chan so', fontsize=11)
ax.set_ylabel('Chieu dai (mm)', fontsize=11)
ax.set_title('Do dai chan (rest pose)', fontsize=13, fontweight='bold')
ax.set_xticks(x); ax.set_xticklabels([f'{i+1}' for i in range(6)])
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# 3. RATIO ANALYSIS
ax = axes[0,2]
ratios = []
labels_data = []
for ll, Lr, label, color in [(LL_DEF, L_def_rest, 'Mac Dinh', '#e74c3c'), 
                               (LL_OPT, L_opt_rest, 'Toi Uu', '#2ecc71')]:
    ratio = abs(ll-Lr)/Lr*100
    ratios.append(ratio)
    labels_data.append(label)
bars = ax.bar(labels_data, ratios, color=['#e74c3c', '#2ecc71'], width=0.5, edgecolor='black', linewidth=2)
ax.set_ylabel('|LL - L_rest| / L_rest (%)', fontsize=11)
ax.set_title('Do lech giua LL va L_rest', fontsize=13, fontweight='bold')
for bar, val in zip(bars, ratios):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{val:.1f}%', 
            ha='center', fontsize=11, fontweight='bold')
ax.set_ylim(0, max(ratios)*1.3); ax.grid(axis='y', alpha=0.3)

# 4. 3D VIEW - TOP VIEW of both configs
from matplotlib.patches import Circle
ax = axes[1,0]
ax.set_aspect('equal')
# Base
theta = np.linspace(0, 2*np.pi, 100)
circle_base_def = Circle((0,0), BR_DEF, fill=False, color='#e74c3c', linestyle='--', linewidth=2, label=f'Base R={BR_DEF}')
circle_base_opt = Circle((0,0), BR_OPT, fill=False, color='#2ecc71', linestyle='-', linewidth=2, label=f'Base R={BR_OPT}')
ax.add_patch(circle_base_def); ax.add_patch(circle_base_opt)
# Platform
circle_plat_def = Circle((0,0), PR_DEF, fill=False, color='#c0392b', linestyle='--', linewidth=2, label=f'Plat R={PR_DEF}')
circle_plat_opt = Circle((0,0), PR_OPT, fill=False, color='#27ae60', linestyle='-', linewidth=2, label=f'Plat R={PR_OPT}')
ax.add_patch(circle_plat_def); ax.add_patch(circle_plat_opt)
ax.set_xlim(-22, 22); ax.set_ylim(-22, 22)
ax.set_xlabel('X'); ax.set_ylabel('Y')
ax.set_title('Kich thuoc Base vs Platform (top view)', fontsize=13, fontweight='bold')
ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3)
ax.axhline(0, color='gray', linewidth=0.5); ax.axvline(0, color='gray', linewidth=0.5)

# 5. 3D view of best configuration
ax = axes[1,1]
ax.remove()
ax = fig.add_subplot(2, 3, 5, projection='3d')
# Plot optimized structure at z=0
for i in range(6):
    ax.plot([base_opt[i,0], plat_opt[i,0]], 
            [base_opt[i,1], plat_opt[i,1]], 
            [base_opt[i,2], plat_opt[i,2]], 'g-', linewidth=1.5, alpha=0.7)
# Base points
ax.scatter(base_opt[:,0], base_opt[:,1], base_opt[:,2], color='blue', s=50, label='Base')
# Platform hexagon
pts = np.vstack([plat_opt, plat_opt[0]])
ax.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=2, label='Platform')
ax.scatter(plat_opt[:,0], plat_opt[:,1], plat_opt[:,2], color='red', s=50)
ax.set_title('Cau hinh TOI UU (rest pose)', fontsize=12, fontweight='bold')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.legend(fontsize=8)

# 6. So sanh collapse probability vs LL
ax = axes[1,2]
ll_vals = np.arange(6, 30, 2)
rates_opt = []; rates_def = []
for ll in ll_vals:
    c_o = 0; c_d = 0
    for _ in range(200):
        z = np.random.uniform(0, 10, 6)
        b_o = base_opt.copy(); b_o[:,2] = z
        ok_o, _, _, _ = solve_platform(b_o, ref_opt, ll)
        if not ok_o: c_o += 1
        b_d = base_def.copy(); b_d[:,2] = z
        ok_d, _, _, _ = solve_platform(b_d, ref_def, ll)
        if not ok_d: c_d += 1
    rates_opt.append(c_o/2)
    rates_def.append(c_d/2)
ax.plot(ll_vals, rates_def, 'r-o', label='Mac Dinh', linewidth=2)
ax.plot(ll_vals, rates_opt, 'g-s', label='Toi Uu', linewidth=2)
ax.axvline(x=LL_DEF, color='red', linestyle='--', alpha=0.5)
ax.axvline(x=LL_OPT, color='green', linestyle='--', alpha=0.5)
ax.set_xlabel('Leg Length (mm)'); ax.set_ylabel('Ty le sap (%)')
ax.set_title('Ty le sap theo leg length', fontsize=13, fontweight='bold')
ax.legend(fontsize=9); ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('d:/final/fig5_optimized.png', dpi=150, bbox_inches='tight')
print(f"  Da luu: fig5_optimized.png")
plt.close()

# ============ TIM MOT TRUONG HOP SAP CU THE (TOI UU) ============
print(f"\nTim mot truong hop sap cu the de minh hoa...")
found_col = False
for _ in range(5000):
    z = np.random.uniform(0, 10, 6)
    b = base_opt.copy(); b[:,2] = z
    ok, Rmat, t, err = solve_platform(b, ref_opt, LL_OPT)
    if not ok:
        found_col = True
        break

if found_col:
    print(f"  Tim thay truong hop sap!")
    print(f"  Z = {z}")
    print(f"  Residual = {err:.6f}")
    
    # Ve collapse case
    fig = plt.figure(figsize=(12, 5))
    
    # Before collapse (rest)
    ax1 = fig.add_subplot(121, projection='3d')
    b_rest = base_opt.copy()
    for i in range(6):
        ax1.plot([b_rest[i,0], plat_opt[i,0]], [b_rest[i,1], plat_opt[i,1]], [b_rest[i,2], plat_opt[i,2]], 
                 'g-', linewidth=1.5, alpha=0.7)
    ax1.scatter(b_rest[:,0], b_rest[:,1], b_rest[:,2], color='blue', s=40)
    pts = np.vstack([plat_opt, plat_opt[0]])
    ax1.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=2)
    ax1.scatter(plat_opt[:,0], plat_opt[:,1], plat_opt[:,2], color='red', s=40)
    ax1.set_title('Rest pose (OK)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_zlim(-5, 20)
    
    # After collapse (with Z displacement)
    ax2 = fig.add_subplot(122, projection='3d')
    # The platform CANNOT find valid position - so we show the best attempt or the disconnected legs
    plat_attempt = np.zeros_like(plat_opt)
    if ok:
        q = (Rmat @ ref_opt.T).T
        plat_attempt = t + q
    
    for i in range(6):
        ax2.plot([b[i,0], plat_attempt[i,0]], [b[i,1], plat_attempt[i,1]], [b[i,2], plat_attempt[i,2]], 
                 'r-', linewidth=1.5, alpha=0.5)
        # Show the target rest length violation
        actual_len = np.linalg.norm(plat_attempt[i]-b[i])
        if abs(actual_len - LL_OPT) > 1:
            ax2.scatter(plat_attempt[i,0], plat_attempt[i,1], plat_attempt[i,2], 
                       color='orange', s=60, marker='x', linewidth=2)
    
    ax2.scatter(b[:,0], b[:,1], b[:,2], color='blue', s=40)
    ax2.set_title(f'COLLAPSE (Z={z[0]:.1f}...{z[5]:.1f})', fontsize=12, fontweight='bold')
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.set_zlim(-5, 20)
    
    # Show Z displacement lines
    for i in range(6):
        ax2.plot([b[i,0], b[i,0]], [b[i,1], b[i,1]], [b[i,2], 0], 'b:', alpha=0.3)
    
    plt.suptitle('Stewart Platform - Collapse Example (Optimized Config)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('d:/final/fig6_collapse_optimized.png', dpi=150)
    print(f"  Da luu: fig6_collapse_optimized.png")
    plt.close()

print(f"\n{'='*70}")
print(f"TONG KET")
print(f"{'='*70}")
print(f"""
Tham so toi uu tim duoc:
  baseRadius={BR_OPT}, platformRadius={PR_OPT}
  baseDistance={BD_OPT}, platformDistance={PD_OPT}
  legLength={LL_OPT}

Hieu qua: {col_def*100/N:.1f}% (mac dinh) -> {col_opt*100/N:.1f}% (toi uu) bi sap
  Giam {(col_def-col_opt)/max(col_def,1)*100:.0f}% so voi mac dinh!

Nguyen nhan chinh:
  - Mac dinh: |LL-L_rest|/L_rest = {abs(LL_DEF-L_def_rest)/L_def_rest*100:.1f}% (lech nhieu)
  - Toi uu: |LL-L_rest|/L_rest = {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.1f}% (lech it)
""")
