"""
Stewart Platform - GIF Animation v2 (IK dung)
=============================================
Heave/Roll/Sway -> Platform pose -> IK tinh Z_base -> 3 goc nhin
"""
import numpy as np, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

def Rx(deg):
    r=math.radians(deg)
    cr,sr=math.cos(r),math.sin(r)
    return np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])

def get_platform_pose(t):
    w=2*math.pi/1.5
    dh=3.5*math.sin(w*t); dR=3.8*math.sin(w*t); dS=3.0*math.sin(w*t)
    C=C0.copy(); C[1]+=dS; C[2]+=dh
    R_mat=Rx(dR)
    P_world=C+(R_mat@REF.T).T
    return C,R_mat,P_world,dh,dR,dS

def compute_base_Z(P_world):
    Z=np.zeros(6); ok=True
    for i in range(6):
        dx=P_world[i,0]-BASE0[i,0]; dy=P_world[i,1]-BASE0[i,1]
        dz_sq=LL**2-dx**2-dy**2
        if dz_sq<0: ok=False; Z[i]=0
        else: Z[i]=P_world[i,2]-math.sqrt(dz_sq)
    return Z,ok

# Tinh toan cac frame
T_total=3.0; N_frames=45
dt=T_total/N_frames
fps=N_frames/T_total

print(f"Tinh {N_frames} frame...")

all_bases=[]; all_ok=[]; all_center=[]; all_plat=[]; all_tilt=[]; all_dh=[]; all_dR=[]; all_dS=[]
all_base_z=[]

for frame in range(N_frames):
    t=frame*dt
    C,R_mat,P_world,dh,dR,dS=get_platform_pose(t)
    Z_base,ik_ok=compute_base_Z(P_world)
    
    # Kiem tra Z_base co trong [0,100]
    col=ik_ok==False or np.any(Z_base<-0.001) or np.any(Z_base>100.001)
    
    n_vec=R_mat@np.array([0,0,1])
    tilt=math.degrees(math.acos(np.clip(n_vec[2],-1,1)))
    
    base_pts=BASE0.copy()
    for i in range(6): base_pts[i,2]=Z_base[i]
    
    all_bases.append(base_pts); all_ok.append(not col)
    all_dh.append(dh); all_dR.append(dR); all_dS.append(dS)
    all_base_z.append(Z_base)
    
    if not col:
        all_center.append(C); all_plat.append(P_world); all_tilt.append(tilt)
    else:
        all_center.append(C); all_plat.append(P_world); all_tilt.append(tilt)
    
    if (frame+1)%10==0:
        print(f"  Frame {frame+1}/{N_frames}: t={t:.2f}s {'OK' if not col else 'COL'} Z_base=[{Z_base[0]:.0f}..{Z_base[5]:.0f}]")

print("Tao animation 3 goc nhin...")

fig=plt.figure(figsize=(16,6))
fig.suptitle('Stewart Platform - Dynamic Simulation (IK method)', fontsize=14,fontweight='bold')

# 3 goc nhin
angs=[(25,-45),(20,30),(10,120)]
titles=['Goc nhin 1 (azim=-45)','Goc nhin 2 (azim=30)','Goc nhin 3 (azim=120)']
pos=[131,132,133]
axs=[]

for i in range(3):
    ax=fig.add_subplot(pos[i],projection='3d')
    ax.set_xlim(-20,20); ax.set_ylim(-20,20); ax.set_zlim(-5,50)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.view_init(elev=angs[i][0],azim=angs[i][1])
    ax.set_title(titles[i],fontsize=10)
    axs.append(ax)

def init():
    for ax in axs:
        ax.clear()
        ax.set_xlim(-20,20); ax.set_ylim(-20,20); ax.set_zlim(-5,50)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    return []

def update(frame):
    t=frame*dt
    bases=all_bases[frame]; ok=all_ok[frame]
    dh=all_dh[frame]; dR=all_dR[frame]; dS=all_dS[frame]
    plat_pts=all_plat[frame]; center=all_center[frame]; tilt=all_tilt[frame]
    
    for i in range(3):
        ax=axs[i]
        ax.clear()
        ax.set_xlim(-20,20); ax.set_ylim(-20,20); ax.set_zlim(-5,50)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.view_init(elev=angs[i][0],azim=angs[i][1])
        ax.set_title(f'{titles[i]} | t={t:.2f}s',fontsize=9)
        
        # Base
        ax.scatter(bases[:,0],bases[:,1],bases[:,2],color='blue',s=25,label='Base')
        for j in range(6):
            ax.plot([bases[j,0],bases[j,0]],[bases[j,1],bases[j,1]],[0,bases[j,2]],'b:',alpha=0.3,lw=0.5)
            ax.text(bases[j,0],bases[j,1],bases[j,2]+1,f'B{j+1}',fontsize=7,color='blue')
        
        # Legs
        for j in range(6):
            c='green' if ok else 'red'
            ax.plot([bases[j,0],plat_pts[j,0]],[bases[j,1],plat_pts[j,1]],
                     [bases[j,2],plat_pts[j,2]],color=c,lw=1.5,alpha=0.7)
        
        # Platform
        pts=np.vstack([plat_pts,plat_pts[0]])
        if ok:
            ax.plot(pts[:,0],pts[:,1],pts[:,2],'r-',lw=2)
            ax.scatter(plat_pts[:,0],plat_pts[:,1],plat_pts[:,2],color='red',s=20)
        else:
            ax.plot(pts[:,0],pts[:,1],pts[:,2],'r--',lw=1,alpha=0.4)
        
        for j in range(6):
            ax.text(plat_pts[j,0],plat_pts[j,1],plat_pts[j,2]+1,f'P{j+1}',fontsize=7,color='red')
        
        ax.scatter(center[0],center[1],center[2],color='orange',s=40,marker='o')
        
        # Info text
        status='OK' if ok else 'COL'
        info=f'{status} | Heave={dh:+.1f} Roll={dR:+.1f}deg Sway={dS:+.1f}\nZc={center[2]:.1f} Tilt={tilt:.0f}deg\nBZ=[{bases[0,2]:.0f}..{bases[5,2]:.0f}]'
        ax.text2D(0.02,0.98,info,transform=ax.transAxes,fontsize=7,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round',facecolor='wheat',alpha=0.7))
        
        if not ok:
            ax.text2D(0.5,0.5,'COLLAPSE',transform=ax.transAxes,fontsize=18,
                       color='red',fontweight='bold',ha='center')
        
        ax.legend(fontsize=6,loc='lower right')
    
    fig.suptitle(f'Stewart Platform - IK Dynamic Sim | t={t:.2f}s/{T_total:.1f}s',fontsize=14,fontweight='bold')
    return []

ani=FuncAnimation(fig,update,frames=N_frames,init_func=init,blit=False,repeat=True)
ani.save('d:/final/dynamic_animation_v2.gif',writer='pillow',fps=fps,dpi=90)
print(f"  Da luu: dynamic_animation_v2.gif")
plt.close()
print("Hoan thanh!")
