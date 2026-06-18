"""
Stewart Platform - Mo phong voi tham so TOI UU
==============================================
baseRadius=10, platformRadius=7, baseDistance=6, platformDistance=2, legLength=16
Su dung thuat toan FK tu stewart_fast.py
"""
import numpy as np, math, sys, time

# ===================== GEOMETRY VOI THAM SO TOI UU =====================
BR=10; PR=7; BD=6; PD=2; LL=16

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

# Tao hinh hoc
PLAT = build_platform(PR,PD,0)
BASE0 = build_base(BR,BD)
LEG = np.array([LL]*6)  # Fix leg length = 16
C0 = np.mean(PLAT,axis=0)
REF = PLAT - C0

print("="*70)
print("STEWART PLATFORM - MO PHONG VOI THAM SO TOI UU")
print("="*70)
print(f"\nTham so:")
print(f"  baseRadius={BR}, platformRadius={PR}")
print(f"  baseDistance={BD}, platformDistance={PD}")
print(f"  legLength={LL}")
print(f"\nToa do Base joints (z=0):")
for i in range(6):
    print(f"  B{i+1} = [{BASE0[i,0]:.4f}, {BASE0[i,1]:.4f}, {BASE0[i,2]:.4f}]")
print(f"\nToa do Platform joints (z=0):")
for i in range(6):
    print(f"  P{i+1} = [{PLAT[i,0]:.4f}, {PLAT[i,1]:.4f}, {PLAT[i,2]:.4f}]")
print(f"\nKhoang cach Plat edges:")
for i in range(6):
    d=np.linalg.norm(PLAT[i]-PLAT[(i+1)%6])
    print(f"  P{i+1}-P{(i+1)%6+1} = {d:.4f}")
print(f"Khoang cach Base edges:")
for i in range(6):
    d=np.linalg.norm(BASE0[i]-BASE0[(i+1)%6])
    print(f"  B{i+1}-B{(i+1)%6+1} = {d:.4f}")

# ===================== FK SOLVER (tu stewart_fast.py) =====================
def R(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])

def P(c,r,p,y):
    return c+(R(r,p,y)@REF.T).T

def fk(bases, iters=20, tol=1e-3):
    c=np.array([0.,0.,37.])
    a=np.array([0.,0.,0.])
    
    for _ in range(iters):
        pts=P(c,a[0],a[1],a[2])
        L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
        e=L-LEG
        en=np.linalg.norm(e)
        if en<tol:
            return True,c,a,en
        
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
    
    pts=P(c,a[0],a[1],a[2])
    L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
    err=np.linalg.norm(L-LEG)
    return (err<1.0),c,a,err

def check_collapse(bases):
    """Return (is_collapsed, reason, detail)"""
    ok,c,a,err=fk(bases)
    if not ok: return True,"SOLVER_FAIL",err
    pts=P(c,a[0],a[1],a[2])
    n=R(a[0],a[1],a[2])@np.array([0,0,1])
    tilt=math.degrees(math.acos(np.clip(n[2],-1,1)))
    minz=np.min(pts[:,2])
    if tilt>60: return True,f"TILT_{tilt:.0f}",tilt
    if minz<-2: return True,f"ZMIN_{minz:.1f}",minz
    if c[2]<-5: return True,f"LOWZ_{c[2]:.1f}",c[2]
    return False,f"OK_Z{c[2]:.0f}_T{tilt:.0f}",tilt

# ===================== MO PHONG =====================
N=5000
np.random.seed(42)
t0=time.time()

coll,stab=[],[]

print(f"\nDang mo phong {N} cau hinh ngau nhien...")
for i in range(N):
    z=np.random.uniform(0,10,6)
    bases=BASE0.copy(); bases[:,2]=z
    bad,msg,det=check_collapse(bases)
    if bad: coll.append((i,z,msg,det,bases))
    else: stab.append((i,z,msg,det,bases))
    if (i+1)%1000==0:
        cp=100*len(coll)/(i+1)
        print(f"  {i+1}/{N}... collapse: {len(coll)} ({cp:.1f}%)")

t1=time.time()
tot=len(coll)+len(stab); cp=100*len(coll)/tot

print("\n"+"="*70)
print(f"KET QUA: {len(coll)}/{tot} collapse ({cp:.2f}%)")
print(f"        {len(stab)}/{tot} stable ({100*len(stab)/tot:.2f}%)")
print(f"Thoi gian: {t1-t0:.1f}s")
print("="*70)

# Phan loai collapse
types={}
for _,_,m,_,_ in coll: t=m.split("_")[0]; types[t]=types.get(t,0)+1
print("\nChi tiet collapse:")
for t,n in sorted(types.items(),key=lambda x:-x[1]):
    print(f"  {t}: {n} ({100*n/len(coll):.1f}%)")

# Vi du collapse
print("\nVi du collapse:")
for idx,(ti,z,msg,det,bases) in enumerate(coll[:5]):
    print(f"\n  #{idx+1} Test {ti}: {msg} (det={det:.2f})")
    print(f"    Z=[{z[0]:.2f} {z[1]:.2f} {z[2]:.2f} {z[3]:.2f} {z[4]:.2f} {z[5]:.2f}]")
    ok2,c2,a2,_=fk(bases)
    if ok2:
        p2=P(c2,a2[0],a2[1],a2[2])
        tilt2=math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2],-1,1)))
        print(f"    Center:({c2[0]:.1f},{c2[1]:.1f},{c2[2]:.1f}) Tilt:{tilt2:.0f}deg")
        print(f"    Joint Z: [{np.min(p2[:,2]):.1f}, {np.max(p2[:,2]):.1f}]")

# Ghi file
out=open('d:/final/optimized_sim_results.txt','w')
out.write("="*70+"\n")
out.write("STEWART PLATFORM - OPTIMIZED PARAMETERS\n")
out.write("="*70+"\n")
out.write(f"baseRadius={BR}, platformRadius={PR}\n")
out.write(f"baseDistance={BD}, platformDistance={PD}\n")
out.write(f"legLength={LL}\n\n")
out.write(f"Simulation: {N} random configs, base Z in [0,10]\n")
out.write(f"Time: {t1-t0:.1f}s\n\n")
out.write(f"STABLE: {len(stab):5d} ({100*len(stab)/tot:.2f}%)\n")
out.write(f"COLLAPSED: {len(coll):5d} ({cp:.2f}%)\n\n")
out.write("Collapse types:\n")
for t,n in sorted(types.items(),key=lambda x:-x[1]):
    out.write(f"  {t}: {n} ({100*n/len(coll):.1f}%)\n")

if coll:
    out.write("\nCollapse examples:\n")
    for idx,(ti,z,msg,det,bases) in enumerate(coll[:10]):
        out.write(f"\n#{idx+1} Test {ti}: {msg}\n")
        out.write(f"  Z=[{z[0]:.2f} {z[1]:.2f} {z[2]:.2f} {z[3]:.2f} {z[4]:.2f} {z[5]:.2f}]\n")
        
if stab:
    out.write("\nStable examples:\n")
    for idx,(ti,z,msg,det,bases) in enumerate(stab[:5]):
        ok2,c2,a2,_=fk(bases)
        tilt2=math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2],-1,1)))
        out.write(f"\n#{idx+1} Test {ti}: Center Z={c2[2]:.1f}, Tilt={tilt2:.0f}deg\n")

out.write("\n"+"="*70+"\n")
out.write(f"CONCLUSION: Collapse rate = {cp:.2f}%\n")
out.write("="*70+"\n")
out.close()

print(f"\nDa luu: d:/final/optimized_sim_results.txt")
