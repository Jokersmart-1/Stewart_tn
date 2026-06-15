"""
Stewart Platform Collapse Simulation (FAST)
============================================
Hexagon platform with rigid links to movable base points.
Detects collapse when geometry is impossible.
"""
import numpy as np, math, sys, time

# Geometry
PLAT = np.array([[8.526279,-7.232051,38.951552],[10.526279,-3.767949,38.951552],
                 [2.0,11.0,38.951552],[-2.0,11.0,38.951552],
                 [-10.526279,-3.767949,38.951552],[-8.526279,-7.232051,38.951552]])
BASE0= np.array([[3.1,-12.6,0],[12.46192,3.615321,0],[9.36192,8.984679,0],
                 [-9.36192,8.984679,0],[-12.46192,3.615321,0],[-3.1,-12.6,0]])
LEG = np.array([np.linalg.norm(PLAT[i]-BASE0[i]) for i in range(6)])
C0 = np.mean(PLAT,axis=0)
REF = PLAT - C0

def R(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def P(c,r,p,y):
    return c+(R(r,p,y)@REF.T).T

def fk(bases, iters=20, tol=1e-3):
    """Fast forward kinematics with fewer iterations and looser tolerance"""
    c=np.array([0.,0.,37.])
    a=np.array([0.,0.,0.])
    J=np.zeros((6,6))
    
    for _ in range(iters):
        pts=P(c,a[0],a[1],a[2])
        L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
        e=L-LEG
        en=np.linalg.norm(e)
        if en<tol:
            return True,c,a,en
        
        for i in range(6):
            d=pts[i]-bases[i]; ln=np.linalg.norm(d)
            if ln<1e-10: return False,c,a,999
            u=d/ln; J[i,:3]=u; J[i,3:]=np.cross(pts[i]-c,u)
        
        try: delta=np.linalg.solve(J,-e)
        except: delta=np.linalg.lstsq(J,-e,rcond=None)[0]
        
        c+=delta[:3]; a+=delta[3:]  # No damping for speed
        if np.any(np.isnan(c))or np.any(np.isnan(a)):
            return False,np.zeros(3),np.zeros(3),999
    
    pts=P(c,a[0],a[1],a[2])
    L=np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
    err=np.linalg.norm(L-LEG)
    return (err<1.0),c,a,err

def test(bases):
    ok,c,a,err=fk(bases)
    if not ok: return True,"SOLVER_FAIL"
    pts=P(c,a[0],a[1],a[2])
    n=R(a[0],a[1],a[2])@np.array([0,0,1])
    tilt=math.degrees(math.acos(np.clip(n[2],-1,1)))
    minz=np.min(pts[:,2])
    if tilt>60: return True,f"TILT_{tilt:.0f}"
    if minz<-2: return True,f"ZMIN_{minz:.1f}"
    if c[2]<0: return True,f"LOWZ_{c[2]:.1f}"
    return False,f"Z{c[2]:.0f}_T{tilt:.0f}"

def run():
    N=2000; np.random.seed(42)
    t0=time.time()
    
    coll,stab=[],[]
    for i in range(1,N+1):
        z=np.random.uniform(0,10,6)
        bases=BASE0.copy(); bases[:,2]=z
        bad,msg=test(bases)
        if bad: coll.append((i,z,msg,bases))
        else: stab.append((i,z,msg,bases))
    
    t1=time.time()
    tot=len(coll)+len(stab); cp=100*len(coll)/tot
    
    out=open('d:/final/sim_results.txt','w')
    out.write("="*70+"\nSTEWART PLATFORM SIMULATION RESULTS\n"+"="*70+"\n")
    out.write(f"Configs: {N}, Time: {t1-t0:.1f}s\n")
    out.write(f"Leg lengths: {np.round(LEG,4)}\n")
    out.write(f"Platform edges: {np.round([np.linalg.norm(PLAT[i]-PLAT[(i+1)%6]) for i in range(6)],4)}\n\n")
    
    out.write(f"RESULTS:\n")
    out.write(f"  Stable:   {len(stab):4d} ({100*len(stab)/tot:.1f}%)\n")
    out.write(f"  Collapsed: {len(coll):4d} ({cp:.1f}%)\n\n")
    
    types={}
    for _,_,m,_ in coll: t=m.split("_")[0]; types[t]=types.get(t,0)+1
    out.write("Breakdown:\n")
    for t,n in sorted(types.items(),key=lambda x:-x[1]):
        out.write(f"  {t}: {n} ({100*n/len(coll):.1f}%)\n")
    
    out.write("\nCOLLAPSE EXAMPLES:\n")
    for idx,(ti,z,msg,bases) in enumerate(coll[:6]):
        out.write(f"\n  #{idx+1} Test {ti}: {msg}\n")
        out.write(f"    Z=[{z[0]:.2f} {z[1]:.2f} {z[2]:.2f} {z[3]:.2f} {z[4]:.2f} {z[5]:.2f}]\n")
        ok2,c2,a2,_=fk(bases)
        if ok2:
            p2=P(c2,a2[0],a2[1],a2[2])
            tilt2=math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2],-1,1)))
            out.write(f"    Center:({c2[0]:.1f},{c2[1]:.1f},{c2[2]:.1f}) Tilt:{tilt2:.0f}deg\n")
            out.write(f"    Joint Z:[{np.min(p2[:,2]):.1f},{np.max(p2[:,2]):.1f}]\n")
    
    out.write("\nSTABLE EXAMPLES:\n")
    for idx,(ti,z,msg,bases) in enumerate(stab[:4]):
        ok2,c2,a2,_=fk(bases)
        tilt2=math.degrees(math.acos(np.clip((R(a2[0],a2[1],a2[2])@np.array([0,0,1]))[2],-1,1)))
        out.write(f"\n  #{idx+1} Test {ti}:\n")
        out.write(f"    Z=[{z[0]:.2f} {z[1]:.2f} {z[2]:.2f} {z[3]:.2f} {z[4]:.2f} {z[5]:.2f}]\n")
        out.write(f"    Center Z:{c2[2]:.1f} Tilt:{tilt2:.0f}deg\n")
    
    out.write("\n"+"="*70+"\nSUMMARY\n"+"="*70+"\n")
    out.write(f"Platform: rigid hexagon, leg length ~{LEG[0]:.1f}\n")
    out.write(f"Base Z moves 0-10, random.\n")
    out.write(f"Collapse rate: {cp:.1f}% ({len(coll)}/{tot})\n")
    out.write(f"Stability: {'HIGH' if cp<5 else 'MODERATE' if cp<30 else 'LOW' if cp<60 else 'VERY LOW'}\n")
    out.write("\nAnalysis: The Stewart platform collapses when base Z displacements\n")
    out.write("create geometric incompatibility. Fixed leg lengths constrain\n")
    out.write("the platform position; when bases move unevenly, the rigid\n")
    out.write("hexagon cannot satisfy all 6 leg length constraints simultaneously.\n")
    out.close()
    print(f"Done! {t1-t0:.1f}s, {len(coll)} collapsed ({cp:.1f}%), {len(stab)} stable")
    print(f"Results saved to sim_results.txt")

if __name__=="__main__":
    run()
