"""
Stewart Platform - Mo phong voi tham so toi uu (fast version)
============================================================
Su dung analytical grid solver thay vi Newton-Raphson
"""
import numpy as np, math, time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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

def fast_check(bases, rs, L):
    best = 1e10
    for rr in np.linspace(-0.8, 0.8, 7):
        for pp in np.linspace(-0.8, 0.8, 7):
            Rmat = Rm(rr, pp, 0.0)
            q = (Rmat @ rs.T).T
            H = np.array([Rmat@(rs[0]-rs[1])-(bases[0]-bases[1]),
                          Rmat@(rs[0]-rs[2])-(bases[0]-bases[2]),
                          Rmat@(rs[0]-rs[3])-(bases[0]-bases[3])])
            if abs(np.linalg.det(H)) < 1e-8: continue
            d = lambda a,b: np.dot(a,b)
            s = np.array([
                d(q[0],bases[0])-d(q[1],bases[1])+(d(rs[1],rs[1])-d(rs[0],rs[0])+d(bases[1],bases[1])-d(bases[0],bases[0]))/2,
                d(q[0],bases[0])-d(q[2],bases[2])+(d(rs[2],rs[2])-d(rs[0],rs[0])+d(bases[2],bases[2])-d(bases[0],bases[0]))/2,
                d(q[0],bases[0])-d(q[3],bases[3])+(d(rs[3],rs[3])-d(rs[0],rs[0])+d(bases[3],bases[3])-d(bases[0],bases[0]))/2])
            try: t = np.linalg.solve(H, s)
            except: continue
            err = sum((np.linalg.norm(t+q[i]-bases[i])-L)**2 for i in range(6))
            if err < best: best = err
            if err < 0.5: return True, err
    for yy in np.linspace(-math.pi, math.pi, 8):
        Rmat = Rm(0, 0, yy)
        q = (Rmat @ rs.T).T
        H = np.array([Rmat@(rs[0]-rs[1])-(bases[0]-bases[1]),
                      Rmat@(rs[0]-rs[2])-(bases[0]-bases[2]),
                      Rmat@(rs[0]-rs[3])-(bases[0]-bases[3])])
        if abs(np.linalg.det(H)) < 1e-8: continue
        d = lambda a,b: np.dot(a,b)
        s = np.array([
            d(q[0],bases[0])-d(q[1],bases[1])+(d(rs[1],rs[1])-d(rs[0],rs[0])+d(bases[1],bases[1])-d(bases[0],bases[0]))/2,
            d(q[0],bases[0])-d(q[2],bases[2])+(d(rs[2],rs[2])-d(rs[0],rs[0])+d(bases[2],bases[2])-d(bases[0],bases[0]))/2,
            d(q[0],bases[0])-d(q[3],bases[3])+(d(rs[3],rs[3])-d(rs[0],rs[0])+d(bases[3],bases[3])-d(bases[0],bases[0]))/2])
        try: t = np.linalg.solve(H, s)
        except: continue
        err = sum((np.linalg.norm(t+q[i]-bases[i])-L)**2 for i in range(6))
        if err < best: best = err
        if err < 0.5: return True, err
    return best < 3.0, best

def coll_rate(br, pr, bd, pd, ll, n=500):
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    rs = plat - np.mean(plat, axis=0)
    np.random.seed(42)
    c = 0
    for _ in range(n):
        z = np.random.uniform(0, 10, 6)
        b = base.copy(); b[:,2] = z
        ok, _ = fast_check(b, rs, ll)
        if not ok: c += 1
    return c/n

# ============ THAM SO ============
# TOI UU (tu optimize_params.py)
BR_OPT=10; PR_OPT=7; BD_OPT=6; PD_OPT=2; LL_OPT=16
# MAC DINH
BR_DEF=14.7; PR_DEF=8.5; BD_DEF=11.2; PD_DEF=4.8; LL_DEF=18

print("="*70)
print("MO PHONG SAP STEWART PLATFORM")
print("="*70)

# Tinh toan thong so
plat_opt = build_platform(PR_OPT, PD_OPT, 0)
base_opt = build_base(BR_OPT, BD_OPT)
L_opt_rest = np.mean([np.linalg.norm(plat_opt[i]-base_opt[i]) for i in range(6)])

plat_def = build_platform(PR_DEF, PD_DEF, 0)
base_def = build_base(BR_DEF, BD_DEF)
L_def_rest = np.mean([np.linalg.norm(plat_def[i]-base_def[i]) for i in range(6)])

print(f"\nTham so TOI UU: BR={BR_OPT}, PR={PR_OPT}, BD={BD_OPT}, PD={PD_OPT}, LL={LL_OPT}")
print(f"  L_rest = {L_opt_rest:.3f}, |LL-L_rest|/L_rest = {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.1f}%")
print(f"\nTham so MAC DINH: BR={BR_DEF}, PR={PR_DEF}, BD={BD_DEF}, PD={PD_DEF}, LL={LL_DEF}")
print(f"  L_rest = {L_def_rest:.3f}, |LL-L_rest|/L_rest = {abs(LL_DEF-L_def_rest)/L_def_rest*100:.1f}%")

# Mo phong
print(f"\nDang mo phong...")
t0 = time.time()
r_opt = coll_rate(BR_OPT, PR_OPT, BD_OPT, PD_OPT, LL_OPT, 2000)
t1 = time.time()
r_def = coll_rate(BR_DEF, PR_DEF, BD_DEF, PD_DEF, LL_DEF, 2000)
t2 = time.time()

print(f"\n{'='*70}")
print("KET QUA MO PHONG (2000 cau hinh ngau nhien)")
print(f"{'='*70}")
print(f"\n  Tham so TOI UU: collapse rate = {r_opt*100:.2f}% (chay {t1-t0:.1f}s)")
print(f"  Tham so MAC DINH: collapse rate = {r_def*100:.2f}% (chay {t2-t1:.1f}s)")
print(f"\n  => Giam collapse: {(r_def-r_opt)/max(r_def,0.001)*100:.0f}%")

# ============ VE BIEU DO ============
print(f"\nDang ve bieu do...")

fig = plt.figure(figsize=(16, 10))
fig.suptitle('Stewart Platform - So sanh tham so TOI UU vs MAC DINH', fontsize=16, fontweight='bold')

# 1. Collapse rate comparison
ax1 = fig.add_subplot(2, 3, 1)
bars = ax1.bar(['Tham so MAC DINH', 'Tham so TOI UU'], 
               [r_def*100, r_opt*100],
               color=['#e74c3c', '#2ecc71'], width=0.5, edgecolor='black', linewidth=2)
ax1.set_ylabel('Ty le sap (%)', fontsize=12)
ax1.set_title('Ty le bi sap (2000 cau hinh)', fontsize=13, fontweight='bold')
ax1.set_ylim(0, max(100, max(r_def, r_opt)*100*1.3))
for bar, val in zip(bars, [r_def*100, r_opt*100]):
    ax1.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.5, f'{val:.1f}%', 
            ha='center', fontsize=14, fontweight='bold')
ax1.grid(axis='y', alpha=0.3)

# 2. Leg length comparison
ax2 = fig.add_subplot(2, 3, 2)
x = np.arange(6)
w = 0.35
legs_def = [np.linalg.norm(plat_def[i]-base_def[i]) for i in range(6)]
legs_opt = [np.linalg.norm(plat_opt[i]-base_opt[i]) for i in range(6)]
ax2.bar(x-w/2, legs_def, w, label='Mac Dinh', color='#e74c3c', edgecolor='black')
ax2.bar(x+w/2, legs_opt, w, label='Toi Uu', color='#2ecc71', edgecolor='black')
ax2.axhline(y=LL_DEF, color='#e74c3c', linestyle='--', alpha=0.7, linewidth=2, label=f'LL={LL_DEF}')
ax2.axhline(y=LL_OPT, color='#2ecc71', linestyle='--', alpha=0.7, linewidth=2, label=f'LL={LL_OPT}')
ax2.set_xlabel('Chan so', fontsize=11)
ax2.set_ylabel('Do dai (mm)', fontsize=11)
ax2.set_title('Do dai chan tai rest pose', fontsize=13, fontweight='bold')
ax2.set_xticks(x); ax2.set_xticklabels([f'{i+1}' for i in range(6)])
ax2.legend(fontsize=9); ax2.grid(alpha=0.3)

# 3. |LL-L_rest|/L_rest ratio
ax3 = fig.add_subplot(2, 3, 3)
ratios = [abs(LL_DEF-L_def_rest)/L_def_rest*100, abs(LL_OPT-L_opt_rest)/L_opt_rest*100]
bars = ax3.bar(['Mac Dinh', 'Toi Uu'], ratios, color=['#e74c3c', '#2ecc71'], 
               width=0.5, edgecolor='black', linewidth=2)
ax3.set_ylabel('|LL - L_rest| / L_rest (%)', fontsize=11)
ax3.set_title('Do lech giua legLength va L_rest', fontsize=13, fontweight='bold')
ax3.set_ylim(0, max(ratios)*1.3)
for bar, val in zip(bars, ratios):
    ax3.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{val:.1f}%', 
            ha='center', fontsize=12, fontweight='bold')
ax3.grid(axis='y', alpha=0.3)

# 4. Top view comparison
ax4 = fig.add_subplot(2, 3, 4)
from matplotlib.patches import Circle
ax4.set_aspect('equal')
c1 = Circle((0,0), BR_DEF, fill=False, color='#e74c3c', linestyle='--', linewidth=2, label=f'Base R={BR_DEF}')
c2 = Circle((0,0), BR_OPT, fill=False, color='#2ecc71', linestyle='-', linewidth=2, label=f'Base R={BR_OPT}')
c3 = Circle((0,0), PR_DEF, fill=False, color='#c0392b', linestyle='--', linewidth=2, label=f'Plat R={PR_DEF}')
c4 = Circle((0,0), PR_OPT, fill=False, color='#27ae60', linestyle='-', linewidth=2, label=f'Plat R={PR_OPT}')
ax4.add_patch(c1); ax4.add_patch(c2); ax4.add_patch(c3); ax4.add_patch(c4)
ax4.set_xlim(-22, 22); ax4.set_ylim(-22, 22)
ax4.set_xlabel('X'); ax4.set_ylabel('Y')
ax4.set_title('Kich thuoc top view', fontsize=13, fontweight='bold')
ax4.legend(fontsize=8); ax4.grid(alpha=0.3); ax4.axhline(0, color='gray', linewidth=0.5); ax4.axvline(0, color='gray', linewidth=0.5)

# 5. 3D view optimized
ax5 = fig.add_subplot(2, 3, 5, projection='3d')
for i in range(6):
    ax5.plot([base_opt[i,0], plat_opt[i,0]], [base_opt[i,1], plat_opt[i,1]], 
             [base_opt[i,2], plat_opt[i,2]], 'g-', linewidth=1.5, alpha=0.7)
ax5.scatter(base_opt[:,0], base_opt[:,1], base_opt[:,2], color='blue', s=40, label='Base')
pts = np.vstack([plat_opt, plat_opt[0]])
ax5.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=2, label='Platform')
ax5.scatter(plat_opt[:,0], plat_opt[:,1], plat_opt[:,2], color='red', s=40)
ax5.set_title('Cau hinh TOI UU', fontsize=12, fontweight='bold')
ax5.set_xlabel('X'); ax5.set_ylabel('Y'); ax5.set_zlabel('Z')
ax5.legend(fontsize=8)

# 6. Text summary
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')
text = f"""KET QUA TONG HOP

Tham so TOI UU:
BR={BR_OPT}, PR={PR_OPT}
BD={BD_OPT}, PD={PD_OPT}
LL={LL_OPT}

Sap: {r_opt*100:.1f}%

Tham so MAC DINH:
BR={BR_DEF}, PR={PR_DEF}
BD={BD_DEF}, PD={PD_DEF}
LL={LL_DEF}

Sap: {r_def*100:.1f}%

CAI THIEN:
{(r_def-r_opt)/max(r_def,0.001)*100:.0f}% giam collapse

Nguyen nhan: |LL-L_rest|/L_rest
Mac Dinh: {abs(LL_DEF-L_def_rest)/L_def_rest*100:.0f}%
Toi Uu: {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.0f}%"""
ax6.text(0.1, 0.9, text, transform=ax6.transAxes, fontsize=11, 
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('d:/final/fig7_optimized_simulation.png', dpi=150, bbox_inches='tight')
print(f"\n  Da luu: fig7_optimized_simulation.png")
plt.close()

# Extra plot: Z random samples that cause collapse
print("\nVe them bieu do phan tich...")

fig2, axes = plt.subplots(2, 2, figsize=(12, 10))
fig2.suptitle('Phan tich chi tiet collapse', fontsize=14, fontweight='bold')

# 4.1: LL vs collapse rate (sweep)
ax = axes[0,0]
ll_range = np.arange(6, 30, 2)
r_opt_vs_ll = [coll_rate(BR_OPT, PR_OPT, BD_OPT, PD_OPT, ll, 200) for ll in ll_range]
r_def_vs_ll = [coll_rate(BR_DEF, PR_DEF, BD_DEF, PD_DEF, ll, 200) for ll in ll_range]
ax.plot(ll_range, [r*100 for r in r_def_vs_ll], 'r-o', label='Mac Dinh', linewidth=2)
ax.plot(ll_range, [r*100 for r in r_opt_vs_ll], 'g-s', label='Toi Uu', linewidth=2)
ax.axvline(x=LL_DEF, color='red', linestyle='--', alpha=0.5)
ax.axvline(x=LL_OPT, color='green', linestyle='--', alpha=0.5)
ax.axvline(x=L_def_rest, color='red', linestyle=':', alpha=0.3, label=f'L_rest(md)={L_def_rest:.1f}')
ax.axvline(x=L_opt_rest, color='green', linestyle=':', alpha=0.3, label=f'L_rest(tu)={L_opt_rest:.1f}')
ax.set_xlabel('Leg Length (mm)'); ax.set_ylabel('Ty le sap (%)')
ax.set_title('Ty le sap theo leg length'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

# 4.2: L_rest comparison
ax = axes[0,1]
x = ['Mac Dinh', 'Toi Uu']
y = [L_def_rest, L_opt_rest]
bars = ax.bar(x, y, color=['#e74c3c', '#2ecc71'], width=0.5, edgecolor='black')
ax.axhline(y=LL_DEF, color='red', linestyle='--', linewidth=2, label=f'LL={LL_DEF}')
ax.axhline(y=LL_OPT, color='green', linestyle='--', linewidth=2, label=f'LL={LL_OPT}')
ax.set_ylabel('L_rest (mm)'); ax.set_title('L_rest vs LegLength')
ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
for bar, val in zip(bars, y):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.3, f'{val:.2f}', 
            ha='center', fontsize=11, fontweight='bold')

# 4.3: Base/Platform radius ratio
ax = axes[1,0]
ratios = [BR_DEF/PR_DEF, BR_OPT/PR_OPT]
bars = ax.bar(['Mac Dinh', 'Toi Uu'], ratios, color=['#e74c3c', '#2ecc71'], 
              width=0.5, edgecolor='black')
ax.set_ylabel('Ty le BR/PR'); ax.set_title('Ti le baseRadius/platformRadius')
for bar, val in zip(bars, ratios):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.02, f'{val:.2f}', 
            ha='center', fontsize=12, fontweight='bold')
ax.grid(axis='y', alpha=0.3)

# 4.4: Comparison table
ax = axes[1,1]
ax.axis('off')
tbl = f"""THONG SO        MAC DINH    TOI UU
baseRadius      {BR_DEF:<8} {BR_OPT:<8}
platformRadius  {PR_DEF:<8} {PR_OPT:<8}
baseDistance    {BD_DEF:<8} {BD_OPT:<8}
platformDist    {PD_DEF:<8} {PD_OPT:<8}
legLength       {LL_DEF:<8} {LL_OPT:<8}
L_rest          {L_def_rest:<8.3f} {L_opt_rest:<8.3f}
Sap rate        {r_def*100:<7.1f}% {r_opt*100:<7.1f}%
TI LE:
BR/PR           {BR_DEF/PR_DEF:<8.2f} {BR_OPT/PR_OPT:<8.2f}
|LL-Lr|/Lr      {abs(LL_DEF-L_def_rest)/L_def_rest*100:<7.1f}% {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:<7.1f}%"""
ax.text(0.1, 0.9, tbl, transform=ax.transAxes, fontsize=11, fontfamily='monospace',
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('d:/final/fig8_analysis_optimized.png', dpi=150, bbox_inches='tight')
print(f"  Da luu: fig8_analysis_optimized.png")
plt.close()

print(f"\n{'='*70}")
print("TONG KET CUOI CUNG")
print(f"{'='*70}")
print(f"""
PHAN TICH STEWART PLATFORM COLLAPSE
====================================

Cau hinh MAC DINH (tu MATLAB):
  BR={BR_DEF}, PR={PR_DEF}, BD={BD_DEF}, PD={PD_DEF}, LL={LL_DEF}
  L_rest={L_def_rest:.3f} -> |LL-L_rest|/L_rest={abs(LL_DEF-L_def_rest)/L_def_rest*100:.1f}%
  Collapse rate: {r_def*100:.1f}%

Cau hinh TOI UU tim duoc:
  BR={BR_OPT}, PR={PR_OPT}, BD={BD_OPT}, PD={PD_OPT}, LL={LL_OPT}
  L_rest={L_opt_rest:.3f} -> |LL-L_rest|/L_rest={abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.1f}%
  Collapse rate: {r_opt*100:.1f}%

CAI THIEN: {(r_def-r_opt)/max(r_def,0.001)*100:.0f}% giam collapse
  (tu {r_def*100:.1f}% xuong {r_opt*100:.1f}%)

Nguyen nhan chinh:
  - Collapse xay ra khi LL khong khop voi L_rest
  - Tham so toi uu can dam bao L_rest ~ LL de chân co du biên do Z
  - Gia tri L_rest phu thuoc vao hinh hoc (BR, PR, BD, PD)

Anh huong toa do goc:
  - Plat joints o Z=38.95, Base joints o Z=0 -> L_rest=39.69
  - Can chon LL=40 cho cau hinh nay
  - Neu dung LL=18 -> |18-39.7|/39.7=55% -> sap ~100%
""")
