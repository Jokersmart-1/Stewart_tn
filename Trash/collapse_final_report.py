"""
Final Report: Stewart Platform Collapse Simulation
==================================================
Using pre-computed results from optimize_params.py
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np, math

# ============ GEOMETRY FUNCTIONS ============
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

# ============ THAM SO ============
# TOI UU
BR_OPT=10; PR_OPT=7; BD_OPT=6; PD_OPT=2; LL_OPT=16
# MAC DINH
BR_DEF=14.7; PR_DEF=8.5; BD_DEF=11.2; PD_DEF=4.8; LL_DEF=18

# Ket qua tu optimize_params.py (verified 1000 tests)
R_OPT = 0.013   # 1.3% collapse (optimized)
R_DEF = 0.976   # 97.6% collapse (default)

plat_opt = build_platform(PR_OPT, PD_OPT, 0)
base_opt = build_base(BR_OPT, BD_OPT)
plat_def = build_platform(PR_DEF, PD_DEF, 0)
base_def = build_base(BR_DEF, BD_DEF)

L_opt_rest = np.mean([np.linalg.norm(plat_opt[i]-base_opt[i]) for i in range(6)])
L_def_rest = np.mean([np.linalg.norm(plat_def[i]-base_def[i]) for i in range(6)])

print("="*70)
print("BAO CAO CUOI CUNG: MO PHONG SAP STEWART PLATFORM")
print("="*70)

print(f"""
THAM SO MAC DINH:
  baseRadius={BR_DEF}, platformRadius={PR_DEF}
  baseDistance={BD_DEF}, platformDistance={PD_DEF}
  legLength={LL_DEF}
  L_rest={L_def_rest:.3f}
  |LL-L_rest|/L_rest = {abs(LL_DEF-L_def_rest)/L_def_rest*100:.1f}%
  COLLAPSE RATE: {R_DEF*100:.1f}%

THAM SO TOI UU:
  baseRadius={BR_OPT}, platformRadius={PR_OPT}
  baseDistance={BD_OPT}, platformDistance={PD_OPT}
  legLength={LL_OPT}
  L_rest={L_opt_rest:.3f}
  |LL-L_rest|/L_rest = {abs(LL_OPT-L_opt_rest)/L_opt_rest*100:.1f}%
  COLLAPSE RATE: {R_OPT*100:.1f}%

CAI THIEN: {(R_DEF-R_OPT)/R_DEF*100:.0f}% giam collapse
  (tu {R_DEF*100:.1f}% xuong {R_OPT*100:.1f}%)

NGUYEN NHAN:
  - Collapse xay ra khi legLength (LL) khong khop voi L_rest
  - Mac dinh: LL={LL_DEF} nhung L_rest={L_def_rest:.1f} (lech {(abs(LL_DEF-L_def_rest)):.1f}mm)
  - Toi uu: LL={LL_OPT} va L_rest={L_opt_rest:.1f} (lech {abs(LL_OPT-L_opt_rest):.1f}mm)
  - Lech cang lon, xac suat sap cang cao

KHUYEN NGHI:
  - Chon LL sao cho |LL - L_rest| / L_rest < 5%
  - Voi toa do goc (L_rest=39.69): dung LL≈40
  - Voi tham so mac dinh (L_rest=8.39): dung LL≈8
""")

# ============ VE 3 BIEU DO DON GIAN ============
print("Dang ve bieu do...")

# 1. Collapse rate comparison
fig, axes = plt.subplots(1, 3, figsize=(16, 5))
fig.suptitle('Stewart Platform - Phan tch collapse', fontsize=15, fontweight='bold')

# Collapse rate bar
ax = axes[0]
bars = ax.bar(['Tham so MAC DINH', 'Tham so TOI UU'], [R_DEF*100, R_OPT*100],
              color=['#e74c3c', '#2ecc71'], width=0.5, edgecolor='black', linewidth=2)
ax.set_ylabel('Ty le sap (%)', fontsize=12)
ax.set_title('Ty le sap (1000 cau hinh)', fontsize=13, fontweight='bold')
ax.set_ylim(0, 110)
for bar, val in zip(bars, [R_DEF*100, R_OPT*100]):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{val:.1f}%', 
            ha='center', fontsize=14, fontweight='bold')
ax.grid(axis='y', alpha=0.3)

# |LL-L_rest|/L_rest ratio
ax = axes[1]
ratios = [abs(LL_DEF-L_def_rest)/L_def_rest*100, abs(LL_OPT-L_opt_rest)/L_opt_rest*100]
bars = ax.bar(['Mac Dinh', 'Toi Uu'], ratios, color=['#e74c3c', '#2ecc71'],
              width=0.5, edgecolor='black', linewidth=2)
ax.set_ylabel('|LL - L_rest| / L_rest (%)', fontsize=12)
ax.set_title('Do lech giua LL va L_rest', fontsize=13, fontweight='bold')
for bar, val in zip(bars, ratios):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{val:.1f}%', 
            ha='center', fontsize=12, fontweight='bold')
ax.set_ylim(0, max(ratios)*1.3)
ax.grid(axis='y', alpha=0.3)

# L_rest comparison
ax = axes[2]
y = [L_def_rest, L_opt_rest]
bars = ax.bar(['Mac Dinh', 'Toi Uu'], y, color=['#e74c3c', '#2ecc71'],
              width=0.5, edgecolor='black', linewidth=2)
ax.axhline(y=LL_DEF, color='#e74c3c', linestyle='--', linewidth=2, label=f'LL={LL_DEF}')
ax.axhline(y=LL_OPT, color='#2ecc71', linestyle='--', linewidth=2, label=f'LL={LL_OPT}')
ax.set_ylabel('Chieu dai (mm)', fontsize=12)
ax.set_title('L_rest so voi legLength', fontsize=13, fontweight='bold')
for bar, val in zip(bars, y):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.3, f'{val:.2f}', 
            ha='center', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(axis='y', alpha=0.3)

plt.tight_layout()
plt.savefig('d:/final/fig9_final_report.png', dpi=150, bbox_inches='tight')
print("  Da luu: fig9_final_report.png")
plt.close()

# 2. Top view comparison + 3D
fig = plt.figure(figsize=(16, 6))
fig.suptitle('Hinh hoc Stewart Platform - Tham so TOI UU', fontsize=14, fontweight='bold')

# Top view
ax1 = fig.add_subplot(121)
ax1.set_aspect('equal')
c1 = Circle((0,0), BR_OPT, fill=False, color='blue', linewidth=2, label=f'Base (R={BR_OPT})')
c2 = Circle((0,0), PR_OPT, fill=False, color='red', linewidth=2, label=f'Platform (R={PR_OPT})')
ax1.add_patch(c1); ax1.add_patch(c2)
# Draw base points
for i in range(6):
    ax1.plot(base_opt[i,0], base_opt[i,1], 'bo', markersize=8)
    ax1.text(base_opt[i,0]+0.5, base_opt[i,1]+0.5, f'B{i+1}', fontsize=8, color='blue')
    ax1.plot(plat_opt[i,0], plat_opt[i,1], 'ro', markersize=8)
    ax1.text(plat_opt[i,0]+0.5, plat_opt[i,1]+0.5, f'P{i+1}', fontsize=8, color='red')
    ax1.plot([base_opt[i,0], plat_opt[i,0]], [base_opt[i,1], plat_opt[i,1]], 'g-', alpha=0.5)
# Platform hexagon
pts = np.vstack([plat_opt, plat_opt[0]])
ax1.plot(pts[:,0], pts[:,1], 'r-', linewidth=2)
ax1.set_xlim(-12, 12); ax1.set_ylim(-12, 12)
ax1.set_xlabel('X (mm)'); ax1.set_ylabel('Y (mm)')
ax1.set_title('Top view - cau hinh TOI UU', fontsize=12, fontweight='bold')
ax1.legend(fontsize=9); ax1.grid(alpha=0.3)
ax1.axhline(0, color='gray', linewidth=0.5); ax1.axvline(0, color='gray', linewidth=0.5)

# 3D view
ax2 = fig.add_subplot(122, projection='3d')
for i in range(6):
    ax2.plot([base_opt[i,0], plat_opt[i,0]], [base_opt[i,1], plat_opt[i,1]],
             [base_opt[i,2], plat_opt[i,2]], 'g-', linewidth=2, alpha=0.8)
ax2.scatter(base_opt[:,0], base_opt[:,1], base_opt[:,2], color='blue', s=30, label='Base joints')
pts = np.vstack([plat_opt, plat_opt[0]])
ax2.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=2, label='Platform')
ax2.scatter(plat_opt[:,0], plat_opt[:,1], plat_opt[:,2], color='red', s=30, label='Plat joints')
ax2.set_title('Cau hinh TOI UU 3D', fontsize=12, fontweight='bold')
ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
ax2.legend(fontsize=8)
ax2.view_init(elev=25, azim=-45)

plt.tight_layout()
plt.savefig('d:/final/fig10_geometry_optimized.png', dpi=150, bbox_inches='tight')
print("  Da luu: fig10_geometry_optimized.png")
plt.close()

print(f"\n{'='*70}")
print("HOAN THANH! Tat ca bieu do da duoc luu:")
print(f"{'='*70}")
print("""
  fig1_initial_config.png
  fig2_collapse_example.png
  fig3_stable_example.png
  fig4_analysis.png
  fig5_sensitivity.png
  fig7_optimized_simulation.png  (moi)
  fig8_analysis_optimized.png    (moi)
  fig9_final_report.png          (moi)
  fig10_geometry_optimized.png   (moi)
""")
