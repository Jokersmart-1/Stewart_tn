"""
Stewart Platform - GIF Animation voi Heave/Roll/Sway
=====================================================
Heave: dh = 3.5*sin(2*pi/1.5*t)
Roll:  dR = 3.8*sin(2*pi/1.5*t) deg
Sway:  dS = 3.0*sin(2*pi/1.5*t)
"""
import numpy as np, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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
C0 = np.mean(PLAT0,axis=0)
REF = PLAT0 - C0
LEG = np.array([LL]*6)

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

# ===================== TINH TRUOC CAC FRAME =====================
T_total = 3.0
N_frames = 60  # 60 frame cho 3 giay ~ 20fps
dt = T_total/N_frames
fps = N_frames/T_total

print(f"Tinh toan {N_frames} frame...")

all_bases = []
all_ok = []
all_center = []
all_plat = []
all_tilt = []
all_dh = []
all_dR = []
all_dS = []

for frame in range(N_frames):
    t = frame * dt
    w = 2*math.pi/1.5
    dh = 3.5*math.sin(w*t)
    dRdeg = 3.8*math.sin(w*t)
    dS = 3.0*math.sin(w*t)
    
    bases = BASE0.copy()
    dR_rad = math.radians(dRdeg)
    for i in range(6):
        x,y,z = BASE0[i]
        # Heave: z += dh
        # Roll:  z += y * sin(dR_rad)  (chi anh huong Z)
        # Sway:  y += dS
        bases[i] = [x, y + dS, y * math.sin(dR_rad) + dh]
    
    ok,c,a,err = fk(bases)
    all_bases.append(bases)
    all_ok.append(ok)
    all_dh.append(dh)
    all_dR.append(dRdeg)
    all_dS.append(dS)
    
    if ok:
        plat_pts = P(c,a[0],a[1],a[2])
        n = R(a[0],a[1],a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2],-1,1)))
        all_center.append(c)
        all_plat.append(plat_pts)
        all_tilt.append(tilt)
    else:
        all_center.append(np.array([0,0,0]))
        all_plat.append(PLAT0.copy())
        all_tilt.append(90)
    
    if (frame+1)%10==0:
        print(f"  Frame {frame+1}/{N_frames} (t={t:.2f}s): dh={dh:.2f}, dR={dRdeg:.1f}deg, ok={ok}")

print(f"Bat dau ve animation...")

# ===================== TAO GIF =====================
fig = plt.figure(figsize=(14, 8))
fig.suptitle('Stewart Platform - Dynamic Simulation (Default Parameters)', fontsize=14, fontweight='bold')

ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122)
ax2.axis('equal')

# Gioi han truc
ax1.set_xlim(-20, 20); ax1.set_ylim(-20, 20); ax1.set_zlim(-5, 50)
ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')

def init():
    ax1.clear()
    ax2.clear()
    ax1.set_xlim(-20, 20); ax1.set_ylim(-20, 20); ax1.set_zlim(-5, 50)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax2.set_xlim(-22, 22); ax2.set_ylim(-22, 22)
    ax2.set_xlabel('X (mm)'); ax2.set_ylabel('Y (mm)')
    ax2.grid(alpha=0.3)
    ax2.axhline(0, color='gray', linewidth=0.5)
    ax2.axvline(0, color='gray', linewidth=0.5)
    return []

def update(frame):
    ax1.clear()
    ax2.clear()
    
    ax1.set_xlim(-20, 20); ax1.set_ylim(-20, 20); ax1.set_zlim(-5, 50)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax2.set_xlim(-22, 22); ax2.set_ylim(-22, 22)
    ax2.set_xlabel('X (mm)'); ax2.set_ylabel('Y (mm)')
    ax2.grid(alpha=0.3)
    ax2.axhline(0, color='gray', linewidth=0.5)
    ax2.axvline(0, color='gray', linewidth=0.5)
    
    t = frame * dt
    bases = all_bases[frame]
    ok = all_ok[frame]
    dh = all_dh[frame]
    dR = all_dR[frame]
    dS = all_dS[frame]
    plat_pts = all_plat[frame]
    center = all_center[frame]
    tilt = all_tilt[frame]
    
    # ---- 3D View ----
    ax1.set_title(f't={t:.2f}s | Heave={dh:+.1f}mm Roll={dR:+.1f}deg Sway={dS:+.1f}mm', fontsize=10)
    
    # Base points
    ax1.scatter(bases[:,0], bases[:,1], bases[:,2], color='blue', s=30, label='Base joints')
    for i in range(6):
        ax1.plot([bases[i,0], bases[i,0]], [bases[i,1], bases[i,1]], [0, bases[i,2]], 
                 'b:', alpha=0.3, linewidth=0.5)
    
    # Legs
    for i in range(6):
        if ok:
            color = 'green'
        else:
            color = 'red'
        ax1.plot([bases[i,0], plat_pts[i,0]], [bases[i,1], plat_pts[i,1]], 
                 [bases[i,2], plat_pts[i,2]], color=color, linewidth=1.5, alpha=0.7)
    
    # Platform
    pts = np.vstack([plat_pts, plat_pts[0]])
    if ok:
        ax1.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=2, label='Platform')
        ax1.scatter(plat_pts[:,0], plat_pts[:,1], plat_pts[:,2], color='red', s=25)
        ax1.scatter(center[0], center[1], center[2], color='orange', s=50, marker='o', label=f'Center Z={center[2]:.1f}')
    else:
        ax1.plot(pts[:,0], pts[:,1], pts[:,2], 'r--', linewidth=1, alpha=0.3)
        ax1.text2D(0.5, 0.5, 'COLLAPSED!', transform=ax1.transAxes, fontsize=20, 
                   color='red', fontweight='bold', ha='center')
    
    ax1.legend(fontsize=7, loc='upper left')
    ax1.view_init(elev=25, azim=-45)
    
    # ---- Top View ----
    ax2.set_title(f'Top View | Center Z={center[2]:.1f}mm Tilt={tilt:.0f}deg | Status={"OK" if ok else "COLLAPSE"}', 
                  fontsize=10, color='green' if ok else 'red')
    
    # Base
    ax2.scatter(bases[:,0], bases[:,1], color='blue', s=30, label='Base')
    for i in range(6):
        ax2.text(bases[i,0]+0.5, bases[i,1]+0.5, f'B{i+1}', fontsize=7, color='blue')
    
    # Platform
    ax2.scatter(plat_pts[:,0], plat_pts[:,1], color='red', s=30, label='Platform')
    for i in range(6):
        ax2.text(plat_pts[i,0]+0.5, plat_pts[i,1]+0.5, f'P{i+1}', fontsize=7, color='red')
    
    # Legs
    for i in range(6):
        ax2.plot([bases[i,0], plat_pts[i,0]], [bases[i,1], plat_pts[i,1]], 
                 'g-' if ok else 'r-', alpha=0.5, linewidth=1)
    
    # Hexagon
    pts2 = np.vstack([plat_pts, plat_pts[0]])
    ax2.plot(pts2[:,0], pts2[:,1], 'r-', linewidth=2 if ok else 1)
    
    ax2.legend(fontsize=8)
    
    fig.suptitle(f'Stewart Platform - Dynamic Simulation (Default Parameters) t={t:.2f}s/{T_total:.1f}s', 
                 fontsize=14, fontweight='bold')
    
    return []

print(f"Dang tao animation GIF...")
ani = FuncAnimation(fig, update, frames=N_frames, init_func=init, blit=False, repeat=True)

ani.save('d:/final/dynamic_animation.gif', writer='pillow', fps=fps, dpi=100)
print(f"  Da luu: dynamic_animation.gif")

plt.close()
print("Hoan thanh!")
