"""Stewart param optimizer - find best params to minimize collapse"""
import numpy as np, math, time

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
    """7x7x10 grid search for feasible (R,t), return best residual"""
    best = 1e10
    for rr in np.linspace(-0.8, 0.8, 7):
        for pp in np.linspace(-0.8, 0.8, 7):
            Rmat = Rm(rr, pp, 0.0)  # yaw=0 first
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
            if err < 0.5: return 0.0
    # Also check with yaw variation
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
        if err < 0.5: return 0.0
    return best

def coll_rate(br, pr, bd, pd, ll, n=200):
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    rs = plat - np.mean(plat, axis=0)
    np.random.seed(42)
    c = 0
    for _ in range(n):
        z = np.random.uniform(0, 10, 6)
        b = base.copy(); b[:,2] = z
        if fast_check(b, rs, ll) > 3.0: c += 1
    return c/n

print("="*70)
print("TIM THAM SO TOI UU - STEWART PLATFORM")
print("="*70)

DEFAULT_BR=14.7; DEFAULT_PR=8.5; DEFAULT_BD=11.2; DEFAULT_PD=4.8; DEFAULT_LL=18.0

# Default
t0=time.time()
r0 = coll_rate(DEFAULT_BR, DEFAULT_PR, DEFAULT_BD, DEFAULT_PD, DEFAULT_LL, 500)
print(f"\nDefault: BR={DEFAULT_BR} PR={DEFAULT_PR} BD={DEFAULT_BD} PD={DEFAULT_PD} LL={DEFAULT_LL}")
print(f"  Collapse rate = {r0*100:.1f}% (took {time.time()-t0:.1f}s)")

# Individual sweeps
print("\n" + "-"*70)
print("INDIVIDUAL SWEEPS")
print("-"*70)

def sweep(pname, vals):
    for v in vals:
        br,pr,bd,pd,ll = DEFAULT_BR,DEFAULT_PR,DEFAULT_BD,DEFAULT_PD,DEFAULT_LL
        if pname=='baseRadius': br=v
        elif pname=='platformRadius': pr=v
        elif pname=='baseDistance': bd=v
        elif pname=='platformDistance': pd=v
        elif pname=='legLength': ll=v
        r = coll_rate(br,pr,bd,pd,ll,200)
        m = "OK" if r==0 else ("WARN" if r<0.05 else "BAD")
        print(f"  {pname}={v}: {r*100:.1f}% [{m}]")

for p,vs in [("baseRadius",[10,12,14,16,18,20]),
             ("platformRadius",[5,7,9,11,13]),
             ("baseDistance",[4,6,8,10,12]),
             ("platformDistance",[2,4,6,8,10]),
             ("legLength",[14,16,18,20,22,24])]:
    print(f"\n{p}:")
    sweep(p, vs)

# Combo sweep
print("\n" + "-"*70)
print("COMBO SWEEP")
print("-"*70)

brs=[10,12,14,16,18]; prs=[5,7,9,11,13]; bds=[4,6,8,10,12]
pds=[2,4,6,8]; lls=[16,18,20,22]
best=1.0; bp=None; total=len(brs)*len(prs)*len(bds)*len(pds)*len(lls)
cnt=0; ts=time.time()

for br in brs:
    for pr in prs:
        for bd in bds:
            for pd in pds:
                for ll in lls:
                    cnt+=1; r=coll_rate(br,pr,bd,pd,ll,100)
                    if cnt%50==0:
                        el=time.time()-ts; rem=(total-cnt)*el/max(cnt,1)
                        print(f"\r  {cnt}/{total} ({cnt*100//total}%) est:{rem:.0f}s best={best*100:.1f}%",end="")
                    if r<best:
                        best=r; bp=(br,pr,bd,pd,ll)
                        print(f"\n  NEW BEST: {r*100:.1f}% BR={br} PR={pr} BD={bd} PD={pd} LL={ll}")
                        if r==0:
                            r2=coll_rate(br,pr,bd,pd,ll,500)
                            if r2==0: print("  CONFIRMED ZERO!"); break
                if best==0: break
            if best==0: break
        if best==0: break
    if best==0: break

print(f"\n\nTotal: {time.time()-ts:.0f}s")
print("\n"+"="*70)
print("RESULTS")
print("="*70)
if bp:
    br,pr,bd,pd,ll = bp
    print(f"\nBEST: BR={br} PR={pr} BD={bd} PD={pd} LL={ll}")
    print(f"Collapse: {best*100:.1f}%")
    rv=coll_rate(br,pr,bd,pd,ll,1000)
    print(f"Verified(1000): {rv*100:.2f}%")
    print(f"\nDefault: {r0*100:.1f}% -> Best: {best*100:.1f}%")
else:
    print("No improvement found")
