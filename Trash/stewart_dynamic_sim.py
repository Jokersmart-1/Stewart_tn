"""
Stewart Platform - Mo phong dong voi tham so MAC DINH
=====================================================
Displacement:
  Heave: ∆h = 3.5*sin(2*pi/1.5*t)
  Roll:  ∆R = 3.8*sin(2*pi/1.5*t) (do)
  Sway:  ∆S = 3.0*sin(2*pi/1.5*t)
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

# Hinh hoc
PLAT0 = build_platform(PR,PD,38.951552)  # Z goc cua platform
BASE0 = build_base(BR,BD)                 # Base o Z=0
C0 = np.mean(PLAT0,axis=0)
REF = PLAT0 - C0
LEG = np.array([LL]*6)

print("="*70)
print("STEWART PLATFORM - MO PHONG DONG (MAC DINH)")
print("="*70)
print(f"\nTham so:")
print(f"  baseRadius={BR}, platformRadius={PR}")
print(f"  baseDistance={BD}, platformDistance={PD}")
print(f"  legLength={LL}")
print(f"\nDieu kien dong:")
print(f"  Heave: dh = 3.5*sin(2*pi/1.5*t)")
print(f"  Roll:  dR = 3.8*sin(2*pi/1.5*t) deg")
print(f"  Sway:  dS = 3.0*sin(2*pi/1.5*t)")
print(f"\nToa do Platform goc (z={PLAT0[0,2]:.3f}):")
for i in range(6):
    print(f"  P{i+1} = [{PLAT0[i,0]:.4f}, {PLAT0[i,1]:.4f}, {PLAT0[i,2]:.4f}]")
print(f"\nToa do Base (z=0):")
for i in range(6):
    print(f"  B{i+1} = [{BASE0[i,0]:.4f}, {BASE0[i,1]:.4f}, {BASE0[i,2]:.4f}]")

L_rest = np.mean([np.linalg.norm(PLAT0[i]-BASE0[i]) for i in range(6)])
print(f"\nL_rest = {L_rest:.3f}  |LL-L_rest|/L_rest = {abs(LL-L_rest)/L_rest*100:.1f}%")

# ===================== FK SOLVER =====================
def R(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])

def P(c,r,p,y):
    return c+(R(r,p,y)@REF.T).T

def fk(bases, iters=30, tol=1e-3):
    c=np.array([0.,0.,38.95])
    a=np.array([0.,0.,0.])
    for _ in range(iters):
        pts=P(c,a[0],a[1],a[2])
        L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
        e=L-LEG; en=np.linalg.norm(e)
        if en<tol: return True,c,a,en
        J=np.zeros((6,6))
        for i in range(6):
            d=pts[i]-bases[i]; ln=np.linalg.norm(d)
            if ln<1e-10: return False,c,a,999
            u=d/ln; J[i,:3]=u; J[i,3:]=np.cross(pts[i]-c,u)
        try: delta=np.linalg.solve(J,-e)
        except: delta=np.linalg.lstsq(J,-e,rcond=None)[0]
        c+=delta[:3]; a+=delta[3:]
        if np.any(np.isnan(c))or np.any(np.isnan(a)):
            return False,np.zeros(3),np.zeros(3),999
    pts=P(c,a[0],a[1],a[2]); L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
    err=np.linalg.norm(L-LEG)
    return (err<1.0),c,a,err

def check_collapse(bases):
    ok,c,a,err=fk(bases)
    if not ok: return True,"SOLVER_FAIL",err,0
    pts=P(c,a[0],a[1],a[2])
    n=R(a[0],a[1],a[2])@np.array([0,0,1])
    tilt=math.degrees(math.acos(np.clip(n[2],-1,1)))
    minz=np.min(pts[:,2])
    if tilt>60: return True,f"TILT_{tilt:.0f}",tilt,tilt
    if minz<-5: return True,f"ZMIN_{minz:.1f}",minz,tilt
    if c[2]<-5: return True,f"LOWZ_{c[2]:.1f}",c[2],tilt
    return False,f"OK",0,tilt

# ===================== APPLY DISPLACEMENT =====================
def apply_displacement(t, base_pts):
    """Apply heave+roll+sway to base joints, return new base positions"""
    w = 2*math.pi/1.5  # tan so goc
    dh = 3.5*math.sin(w*t)     # Heave (mm)
    dR = math.radians(3.8*math.sin(w*t))  # Roll (rad)
    dS = 3.0*math.sin(w*t)     # Sway (mm)
    
    # Rotation matrix for roll (quay quanh X)
    cr,sr = math.cos(dR), math.sin(dR)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    
    # Apply to each base point: rotate + translate
    new_bases = base_pts.copy()
    for i in range(6):
        p = base_pts[i]
        # Roll rotation around X axis, then heave + sway
        p_rot = Rx @ p
        p_rot[1] += dS  # sway
        p_rot[2] += dh  # heave
        new_bases[i] = p_rot
    return new_bases

# ===================== MO PHONG =====================
dt = 0.01
T_total = 3.0  # 3 seconds = 2 chu ky
N_steps = int(T_total/dt)

print(f"\nMo phong {T_total}s ({N_steps} buoc, dt={dt}s)...")
print(f"Tan so: f=1/1.5={1/1.5:.3f} Hz, 2 chu ky")

t0 = time.time()

times = []
heave_vals = []
roll_vals = []
sway_vals = []
center_z_vals = []
tilt_vals = []
leg_lengths = []
collapse_times = []
status_vals = []

coll_count = 0
collapsed = False
last_ok = True

for step in range(N_steps):
    t = step*dt
    w = 2*math.pi/1.5
    dh = 3.5*math.sin(w*t)
    dR = 3.8*math.sin(w*t)
    dS = 3.0*math.sin(w*t)
    
    # Apply displacement to base (base joints only on Z)
    # Heave: z_i += dh
    # Roll:  z_i += y_i * sin(dR_rad)  (khong thay doi X,Y)
    # Sway:  y_i += dS
    bases = BASE0.copy()
    dR_rad = math.radians(dR)
    for i in range(6):
        x,y,z = BASE0[i]
        # Heave: z += dh
        # Roll:  z += y * sin(dR_rad)
        # Sway:  y += dS
        bases[i] = [x, y + dS, y * math.sin(dR_rad) + dh]
    
    bad,msg,det,tilt = check_collapse(bases)
    
    times.append(t)
    heave_vals.append(dh)
    roll_vals.append(dR)
    sway_vals.append(dS)
    
    if not bad:
        ok,c,a,err = fk(bases)
        center_z_vals.append(c[2])
        tilt_vals.append(tilt)
        coll_count = 0
        collapsed = False
        status_vals.append(1)  # 1 = stable
    else:
        center_z_vals.append(0)
        tilt_vals.append(90)
        coll_count += 1
        if coll_count == 1:
            collapse_times.append(t)
            print(f"  COLLAPSE tai t={t:.3f}s: {msg}")
        collapsed = True
        status_vals.append(0)  # 0 = collapsed
    
    if (step+1) % 200 == 0:
        print(f"  t={t:.2f}s: dh={dh:.2f}, dR={dR:.1f}deg, dS={dS:.2f}, "
              f"status={'OK' if not bad else 'COLLAPSE'}")

t1 = time.time()

print(f"\nThoi gian mo phong: {t1-t0:.2f}s")
print(f"So lan collapse: {len(collapse_times)}")

# Xu ly ket qua
if collapse_times:
    first_col = collapse_times[0]
    stable_periods = []
    in_stable = True
    stable_start = 0
    for t in collapse_times:
        if in_stable:
            stable_periods.append((stable_start, t))
            in_stable = False
        else:
            in_stable = True
            stable_start = t
    stable_pct = sum(end-st for st,end in stable_periods)/T_total*100 if stable_periods else 0
else:
    first_col = None
    stable_pct = 100

n_col = sum(1 for s in status_vals if s==0)
n_ok = N_steps - n_col

print(f"\nKET QUA:")
print(f"  Tong so buoc: {N_steps}")
print(f"  Ong dinh: {n_ok} ({n_ok/N_steps*100:.1f}%)")
print(f"  Collapse: {n_col} ({n_col/N_steps*100:.1f}%)")
if first_col:
    print(f"  Lan collapse dau tien: t={first_col:.3f}s")
print(f"  Thoi gian ong dinh: {n_ok*dt:.2f}s / {T_total:.2f}s ({n_ok/N_steps*100:.1f}%)")

# ===================== VE BIEU DO =====================
print(f"\nDang ve bieu do...")

fig, axes = plt.subplots(4, 1, figsize=(14, 12))
fig.suptitle('Stewart Platform - Mo phong dong (tham so MAC DINH)', fontsize=14, fontweight='bold')

# 1. Displacement inputs
ax = axes[0]
ax.plot(times, heave_vals, 'b-', linewidth=1.5, label='Heave dh=3.5*sin(wt)')
ax.plot(times, roll_vals, 'r-', linewidth=1.5, label='Roll dR=3.8*sin(wt) deg')
ax.plot(times, sway_vals, 'g-', linewidth=1.5, label='Sway dS=3.0*sin(wt)')
ax.set_ylabel('Displacement', fontsize=11)
ax.set_title('Input: Heave, Roll, Sway', fontsize=12, fontweight='bold')
ax.legend(fontsize=9); ax.grid(alpha=0.3)
ax.set_xlim(0, T_total)

# 2. Center Z position
ax = axes[1]
colors = ['green' if s==1 else 'red' for s in status_vals]
ax.scatter(times, center_z_vals, c=colors, s=5, alpha=0.7)
ax.plot(times, center_z_vals, 'k-', alpha=0.3, linewidth=0.5)
ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax.set_ylabel('Center Z (mm)', fontsize=11)
ax.set_title('Vi tri Z cua platform center', fontsize=12, fontweight='bold')
ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

# 3. Tilt angle
ax = axes[2]
ax.plot(times, tilt_vals, 'purple', linewidth=1.5, alpha=0.8)
ax.axhline(y=60, color='red', linestyle='--', alpha=0.5, label='Gioi han 60 do')
ax.set_ylabel('Tilt (do)', fontsize=11)
ax.set_title('Goc nghieng platform', fontsize=12, fontweight='bold')
ax.legend(fontsize=9); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)
ax.set_ylim(0, 100)

# 4. Status indicator
ax = axes[3]
ax.fill_between(times, 0, status_vals, color='green', alpha=0.5, label='Ong dinh')
ax.fill_between(times, 0, [1-s for s in status_vals], color='red', alpha=0.5, label='Collapse')
ax.set_ylabel('Status', fontsize=11)
ax.set_title('Trang thai (1=Ong dinh, 0=Collapse)', fontsize=12, fontweight='bold')
ax.set_yticks([0,1]); ax.set_yticklabels(['Collapse','OK'])
ax.set_ylim(-0.1, 1.1)
ax.legend(fontsize=9, loc='upper right'); ax.grid(alpha=0.3); ax.set_xlim(0, T_total)

plt.tight_layout()
plt.savefig('d:/final/fig11_dynamic_sim.png', dpi=150, bbox_inches='tight')
print(f"  Da luu: fig11_dynamic_sim.png")
plt.close()

# 2. Extra plot: displacement vs status
fig2 = plt.figure(figsize=(14, 8))
fig2.suptitle('Phan tich collapse theo displacement', fontsize=14, fontweight='bold')

ax1 = fig2.add_subplot(221)
sc = ax1.scatter(heave_vals, roll_vals, c=status_vals, cmap='RdYlGn', s=3, alpha=0.6)
ax1.set_xlabel('Heave (mm)'); ax1.set_ylabel('Roll (deg)')
ax1.set_title('Heave vs Roll (mau=status)')
plt.colorbar(sc, ax=ax1, label='1=OK,0=Collapse')
ax1.grid(alpha=0.3)

ax2 = fig2.add_subplot(222)
sc2 = ax2.scatter(heave_vals, sway_vals, c=status_vals, cmap='RdYlGn', s=3, alpha=0.6)
ax2.set_xlabel('Heave (mm)'); ax2.set_ylabel('Sway (mm)')
ax2.set_title('Heave vs Sway (mau=status)')
plt.colorbar(sc2, ax=ax2, label='1=OK,0=Collapse')
ax2.grid(alpha=0.3)

ax3 = fig2.add_subplot(223)
sc3 = ax3.scatter(roll_vals, sway_vals, c=status_vals, cmap='RdYlGn', s=3, alpha=0.6)
ax3.set_xlabel('Roll (deg)'); ax3.set_ylabel('Sway (mm)')
ax3.set_title('Roll vs Sway (mau=status)')
plt.colorbar(sc3, ax=ax3, label='1=OK,0=Collapse')
ax3.grid(alpha=0.3)

ax4 = fig2.add_subplot(224)
ax4.axis('off')
txt = f"KET QUA TONG HOP\n\n"
txt += f"Tham so MAC DINH:\n"
txt += f"BR={BR}, PR={PR}, BD={BD}, PD={PD}\n"
txt += f"LL={LL}, L_rest={L_rest:.2f}\n\n"
txt += f"Mo phong: {T_total}s, {N_steps} buoc\n"
txt += f"Tan so: {1/1.5:.3f}Hz (2 chu ky)\n\n"
txt += f"Ong dinh: {n_ok} ({n_ok/N_steps*100:.1f}%)\n"
txt += f"Collapse: {n_col} ({n_col/N_steps*100:.1f}%)\n"
if first_col:
    txt += f"\nCollapse lan dau:\nt={first_col:.3f}s"
txt += f"\n\nBien do:\nHeave=3.5mm\nRoll=3.8deg\nSway=3.0mm"
ax4.text(0.1, 0.9, txt, transform=ax4.transAxes, fontsize=11,
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('d:/final/fig12_dynamic_analysis.png', dpi=150, bbox_inches='tight')
print(f"  Da luu: fig12_dynamic_analysis.png")
plt.close()

# Ghi file ket qua
out=open('d:/final/dynamic_sim_results.txt','w')
out.write("="*70+"\n")
out.write("STEWART PLATFORM - DYNAMIC SIMULATION\n")
out.write("="*70+"\n")
out.write(f"Parameters (DEFAULT):\n")
out.write(f"  baseRadius={BR}, platformRadius={PR}\n")
out.write(f"  baseDistance={BD}, platformDistance={PD}\n")
out.write(f"  legLength={LL}, L_rest={L_rest:.3f}\n\n")
out.write(f"Input:\n")
out.write(f"  Heave: dh = 3.5*sin(2*pi/1.5*t)\n")
out.write(f"  Roll:  dR = 3.8*sin(2*pi/1.5*t) deg\n")
out.write(f"  Sway:  dS = 3.0*sin(2*pi/1.5*t)\n")
out.write(f"  Period = 1.5s, Simulation = {T_total}s\n\n")
out.write(f"RESULTS:\n")
out.write(f"  Total steps: {N_steps}\n")
out.write(f"  Stable: {n_ok:5d} ({n_ok/N_steps*100:.1f}%)\n")
out.write(f"  Collapsed: {n_col:5d} ({n_col/N_steps*100:.1f}%)\n")
if first_col:
    out.write(f"  First collapse at t={first_col:.3f}s\n")
out.write(f"\nCollapse events:\n")
for ct in collapse_times:
    out.write(f"  t={ct:.3f}s\n")
out.write("\nSamples:\n")
for step in range(0, min(N_steps, 100), 5):
    t = step*dt
    w = 2*math.pi/1.5
    dh = 3.5*math.sin(w*t)
    dR = 3.8*math.sin(w*t)
    dS = 3.0*math.sin(w*t)
    s = "OK" if status_vals[step]==1 else "COLLAPSE"
    cz = center_z_vals[step] if status_vals[step]==1 else 0
    tl = tilt_vals[step]
    out.write(f"  t={t:.2f}s dh={dh:+.2f} dR={dR:+.1f}deg dS={dS:+.2f} -> Z={cz:.1f} Tilt={tl:.0f}deg [{s}]\n")
out.write("\n"+"="*70+"\n")
out.write("CONCLUSION:\n")
if n_col > N_steps*0.5:
    out.write("  Platform COLLAPSES under these dynamic conditions.\n")
elif n_col > 0:
    out.write(f"  Platform PARTIALLY collapses ({n_col/N_steps*100:.1f}% of time).\n")
else:
    out.write("  Platform is STABLE under these dynamic conditions.\n")
out.write("="*70+"\n")
out.close()
print(f"  Da luu: dynamic_sim_results.txt")

print(f"\n{'='*70}")
print(f"HOAN THANH MO PHONG DONG!")
print(f"{'='*70}")
print(f"\nTong ket: {n_ok}/{N_steps} ong dinh, {n_col}/{N_steps} collapse")
if first_col:
    print(f"Collapse lan dau tai t={first_col:.3f}s")
print(f"Xem ket qua chi tiet: dynamic_sim_results.txt")
