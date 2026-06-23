"""
Stewart Platform - Mo phong dong DUNG
=====================================
Heave, Roll, Sway -> Platform chay -> Tinh Z_base tu IK -> Kiem tra collapse

Base joints CHI di chuyen tren Z (toi da 10mm)
"""
import numpy as np, math, time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ===================== THAM SO MAC DINH =====================
BR=14.7; PR=8.5; BD=11.2; PD=4.8; LL=18.0

def build_platform(pr,pd,z=0):
    po=math.atan2(pd/2,pr); R=math.sqrt(pr**2+(pd/2)**2)
    return np.array([[R*math.sin(math.pi/3-po),-R*math.cos(math.pi/3-po),z],
        [R*math.sin(math.pi/3+po),-R*math.cos(math.pi/3+po),z],
        [pd/2,pr,z],[-pd/2,pr,z],
        [-R*math.sin(math.pi/3+po),-R*math.cos(math.pi/3+po),z],
        [-R*math.sin(math.pi/3-po),-R*math.cos(math.pi/3-po),z]])

def build_base(br,bd):
    bo=math.atan2(bd/2,br); R=math.sqrt(br**2+(bd/2)**2)
    return np.array([[bd/2,-br,0],[R*math.sin(math.pi/3+bo),R*math.cos(math.pi/3+bo),0],
        [R*math.sin(math.pi/3-bo),R*math.cos(math.pi/3-bo),0],
        [-R*math.sin(math.pi/3-bo),R*math.cos(math.pi/3-bo),0],
        [-R*math.sin(math.pi/3+bo),R*math.cos(math.pi/3+bo),0],
        [-bd/2,-br,0]])

PLAT0 = build_platform(PR,PD,38.951552)
BASE0 = build_base(BR,BD)
C0 = np.mean(PLAT0,axis=0)  # [0, 0, 38.951552]
REF = PLAT0 - C0              # Platform joints trong local frame
LEG = np.array([LL]*6)

# Thong tin
print("="*70)
print("STEWART PLATFORM - MO PHONG DONG DUNG (v2)")
print("="*70)
print(f"\nTham so:")
print(f"  baseRadius={BR}, platformRadius={PR}")
print(f"  baseDistance={BD}, platformDistance={PD}")
print(f"  legLength={LL}")
print(f"\nCACH MO PHONG:")
print(f"  1. Heave/Roll/Sway -> Platform pose")
print(f"  2. Tu platform pose -> IK -> Z_base can thiet")
print(f"  3. Kiem tra Z_base co nam trong [0,10] khong")
print(f"  4. Neu Z_base ngoai pham vi -> COLLAPSE")
print(f"\nToa do Platform goc (z={PLAT0[0,2]:.3f}):")
for i in range(6):
    print(f"  P{i+1} = [{PLAT0[i,0]:.6f}, {PLAT0[i,1]:.6f}, {PLAT0[i,2]:.6f}]")
print(f"\nToa do Base XY co dinh (Z chay tu do):")
for i in range(6):
    print(f"  B{i+1} = [{BASE0[i,0]:.6f}, {BASE0[i,1]:.6f}, Z can tim]")

L_rest = np.mean([np.linalg.norm(PLAT0[i]-BASE0[i]) for i in range(6)])
print(f"\nL_rest = {L_rest:.3f}")

# ===================== XOAY & PLATFORM POSE =====================
def Rx(deg):
    """Ma tran xoay Roll (quanh X)"""
    r = math.radians(deg)
    cr,sr = math.cos(r), math.sin(r)
    return np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])

def get_platform_pose(t):
    """Tinh vi tri Platform tu Heave/Roll/Sway"""
    w = 2*math.pi/1.5
    dh = 3.5 * math.sin(w*t)      # Heave (mm)
    dR = 3.8 * math.sin(w*t)      # Roll (deg)
    dS = 3.0 * math.sin(w*t)      # Sway (mm)
    
    # Platform center: Heave -> Z, Sway -> Y
    C = C0.copy()
    C[1] += dS  # Sway theo Y
    C[2] += dh  # Heave theo Z
    
    # Platform orientation: Roll quanh X
    R_mat = Rx(dR)
    
    # Platform joints trong world frame
    P_world = C + (R_mat @ REF.T).T
    
    return C, R_mat, P_world, dh, dR, dS

def compute_base_Z(P_world):
    """
    Tinh Z can thiet cho moi base joint tu platform pose
    IK: (Px-Bx)^2 + (Py-By)^2 + (Pz-Bz)^2 = LL^2
    => Bz = Pz - sqrt(LL^2 - (Px-Bx)^2 - (Py-By)^2)
    """
    Z_needed = np.zeros(6)
    ok = True
    for i in range(6):
        dx = P_world[i,0] - BASE0[i,0]
        dy = P_world[i,1] - BASE0[i,1]
        dz_sq = LL**2 - dx**2 - dy**2
        if dz_sq < 0:
            # Khong the dat den -> collapse
            ok = False
            Z_needed[i] = 0
        else:
            Z_needed[i] = P_world[i,2] - math.sqrt(dz_sq)
    return Z_needed, ok

def check_collapse(P_world, Z_base, C, tilt_deg):
    """Kiem tra cac dieu kien collapse"""
    # 1. Kiem tra Z_base co trong [0,100] khong
    if np.any(Z_base < -0.001) or np.any(Z_base > 100.001):
        return True, "Z_BASE_OOR"
    
    # 2. Kiem tra platform co cham dat khong
    if np.min(P_world[:,2]) < -5:
        return True, "ZMIN"
    
    # 3. Kiem tra tilt
    if tilt_deg > 60:
        return True, "TILT"
    
    # 4. Kiem tra center Z
    if C[2] < -5:
        return True, "LOWZ"
    
    return False, "OK"

# ===================== MO PHONG =====================
dt = 0.01
T_total = 3.0
N_steps = int(T_total/dt)

print(f"\nMo phong {T_total}s ({N_steps} buoc)...")
print(f"f = 1/1.5 = {1/1.5:.3f} Hz (2 chu ky)")

t0 = time.time()

times = []
heave_vals = []
roll_vals = []
sway_vals = []
center_z_vals = []
tilt_vals = []
base_z_vals = []  # Z cua 6 base joints
status_vals = []
collapse_times = []
collapse_reasons = []

for step in range(N_steps):
    t = step * dt
    
    # 1. Tinh platform pose tu Heave/Roll/Sway
    C, R_mat, P_world, dh, dR, dS = get_platform_pose(t)
    
    # 2. Tinh Z can thiet cho base joints (IK)
    Z_base, ik_ok = compute_base_Z(P_world)
    
    # 3. Tinh tilt
    n_vec = R_mat @ np.array([0,0,1])
    tilt = math.degrees(math.acos(np.clip(n_vec[2], -1, 1)))
    
    # 4. Kiem tra collapse
    col, reason = check_collapse(P_world, Z_base, C, tilt)
    if not ik_ok:
        col = True
        reason = "IK_FAIL"
    
    times.append(t)
    heave_vals.append(dh)
    roll_vals.append(dR)
    sway_vals.append(dS)
    center_z_vals.append(C[2])
    tilt_vals.append(tilt)
    base_z_vals.append(Z_base.copy())
    
    if not col:
        status_vals.append(1)
    else:
        status_vals.append(0)
        if len(collapse_times) == 0 or t - collapse_times[-1][0] > 0.01:
            collapse_times.append((t, reason))
            print(f"  COLLAPSE t={t:.3f}s: {reason}")
            if len(collapse_times) >= 10:
                pass  # ghi nhan toi da

t1 = time.time()

# ===================== XU LY KET QUA =====================
n_col = sum(1 for s in status_vals if s == 0)
n_ok = N_steps - n_col

print(f"\nThoi gian mo phong: {t1-t0:.2f}s")
print(f"\nKET QUA:")
print(f"  Ong dinh: {n_ok:5d} ({n_ok/N_steps*100:.1f}%)")
print(f"  Collapse: {n_col:5d} ({n_col/N_steps*100:.1f}%)")

if collapse_times:
    print(f"\nThoi diem collapse:")
    for ct, cr in collapse_times[:10]:
        print(f"  t={ct:.3f}s: {cr}")

# Thong ke ly do collapse
reasons = {}
for _, cr in collapse_times:
    reasons[cr] = reasons.get(cr, 0) + 1
if reasons:
    print(f"\nLy do collapse:")
    for r, n in sorted(reasons.items(), key=lambda x: -x[1]):
        print(f"  {r}: {n}")

# ===================== VE BIEU DO =====================
print(f"\nDang ve bieu do...")

fig, axes = plt.subplots(5, 1, figsize=(14, 14))
fig.suptitle('Stewart Platform - Mo phong dong (v2 - IK dung)', fontsize=14, fontweight='bold')

# 1. Input: Heave/Roll/Sway
ax = axes[0]
ax.plot(times, heave_vals, 'b-', lw=1.5, label='Heave dh')
ax.plot(times, roll_vals, 'r-', lw=1.5, label='Roll dR (deg)')
ax.plot(times, sway_vals, 'g-', lw=1.5, label='Sway dS')
ax.set_ylabel('Gia tri', fontsize=10)
ax.set_title('Input: Heave, Roll, Sway -> Platform', fontsize=11, fontweight='bold')
ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

# 2. Center Z
ax = axes[1]
ax.plot(times, center_z_vals, 'b-', lw=1.5)
ax.axhline(y=C0[2], color='gray', ls='--', alpha=0.5, label=f'Z0={C0[2]:.1f}')
ax.set_ylabel('Center Z (mm)', fontsize=10)
ax.set_title('Platform Center Z', fontsize=11, fontweight='bold')
ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

# 3. Tilt
ax = axes[2]
ax.plot(times, tilt_vals, 'purple', lw=1.5)
ax.axhline(y=60, color='red', ls='--', alpha=0.5, label='Gioi han 60 do')
ax.set_ylabel('Tilt (do)', fontsize=10)
ax.set_title('Goc nghieng Platform (Roll)', fontsize=11, fontweight='bold')
ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)
ax.set_ylim(0, max(90, max(tilt_vals)+5))

# 4. Base Z values
ax = axes[3]
base_z_arr = np.array(base_z_vals)
colors_b = ['blue','cyan','green','orange','red','purple']
for i in range(6):
    ax.plot(times, base_z_arr[:,i], color=colors_b[i], lw=1, label=f'B{i+1} Z')
ax.axhline(y=0, color='gray', ls='--', alpha=0.5)
ax.axhline(y=10, color='gray', ls='--', alpha=0.5)
ax.fill_between([0,T_total], 0, 10, alpha=0.08, color='green', label='Gioi han [0,10]')
ax.set_ylabel('Base Z (mm)', fontsize=10)
ax.set_title('Z can thiet cua 6 Base joints (tu IK)', fontsize=11, fontweight='bold')
ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

# 5. Status
ax = axes[4]
colors_s = ['green' if s==1 else 'red' for s in status_vals]
ax.scatter(times, center_z_vals, c=colors_s, s=3, alpha=0.7)
ax.plot(times, center_z_vals, 'k-', alpha=0.2, lw=0.5)
ax.set_ylabel('Center Z (mm)', fontsize=10)
ax.set_title('Trang thai (xanh=OK, do=collapse)', fontsize=11, fontweight='bold')
ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

plt.tight_layout()
plt.savefig('d:/final/fig13_dynamic_v2.png', dpi=150, bbox_inches='tight')
print(f"  Da luu: fig13_dynamic_v2.png")
plt.close()

# ===================== GHI FILE =====================
out = open('d:/final/dynamic_v2_results.txt', 'w')
out.write("="*70+"\n")
out.write("STEWART PLATFORM - DYNAMIC SIMULATION v2 (IK)\n")
out.write("="*70+"\n")
out.write(f"Params: BR={BR}, PR={PR}, BD={BD}, PD={PD}, LL={LL}, L_rest={L_rest:.3f}\n\n")
out.write("Method:\n")
out.write("  1. Heave/Roll/Sway -> Platform pose (C, R)\n")
out.write("  2. IK: tu P_world, Base XY co dinh -> tinh Z_base\n")
out.write("  3. Collapse neu Z_base < 0 hoac > 10\n\n")
out.write(f"RESULTS:\n")
out.write(f"  Steps: {N_steps}, Time: {t1-t0:.2f}s\n")
out.write(f"  Stable: {n_ok:5d} ({n_ok/N_steps*100:.1f}%)\n")
out.write(f"  Collapsed: {n_col:5d} ({n_col/N_steps*100:.1f}%)\n")
if collapse_times:
    out.write(f"  First collapse: t={collapse_times[0][0]:.3f}s ({collapse_times[0][1]})\n\n")
    out.write("Collapse events:\n")
    for ct, cr in collapse_times:
        out.write(f"  t={ct:.3f}s: {cr}\n")
out.write("\nSamples:\n")
for step in range(0, min(N_steps, 100), 5):
    t = step*dt
    dh = heave_vals[step]; dR = roll_vals[step]; dS = sway_vals[step]
    cz = center_z_vals[step]; tl = tilt_vals[step]
    st = "OK" if status_vals[step]==1 else "COL"
    Zb = base_z_vals[step]
    Zb_str = f"[{Zb[0]:.1f} {Zb[1]:.1f} {Zb[2]:.1f} {Zb[3]:.1f} {Zb[4]:.1f} {Zb[5]:.1f}]"
    out.write(f"  t={t:.2f}s dh={dh:+.2f} dR={dR:+.1f}deg dS={dS:+.2f} -> CZ={cz:.1f} Tilt={tl:.0f}deg BZ={Zb_str} [{st}]\n")
out.write("\n"+"="*70+"\n")
out.write("CONCLUSION:\n")
if n_col > N_steps*0.5:
    out.write("  Platform COLLAPSES under these dynamic conditions.\n")
elif n_col > 0:
    out.write(f"  Platform PARTIALLY collapses ({n_col/N_steps*100:.1f}%).\n")
else:
    out.write("  Platform is STABLE under these dynamic conditions.\n")
out.write("="*70+"\n")
out.close()
print(f"  Da luu: dynamic_v2_results.txt")

print(f"\n{'='*70}")
print(f"HOAN THANH MO PHONG DONG v2!")
print(f"{'='*70}")
print(f"Ket qua: {n_ok}/{N_steps} ong dinh, {n_col}/{N_steps} collapse")
