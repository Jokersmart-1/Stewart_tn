"""
Stewart Platform - Parameter Optimization
==========================================
Tim tham so toi uu de platform KHONG BI SAP hoac it sap nhat
"""
import numpy as np, math, time

DEFAULT_BR = 14.7; DEFAULT_PR = 8.5; DEFAULT_BD = 11.2; DEFAULT_PD = 4.8; DEFAULT_LL = 18.0

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

def fk(bases, plat_ref, L, c0, tol=1e-3, max_iter=30):
    def R(rr,pp,yy):
        cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp); cy,sy=math.cos(yy),math.sin(yy)
        return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    def P(c,rr,pp,yy):
        return c + (R(rr,pp,yy) @ plat_ref.T).T
    c = c0.copy(); a = np.array([0.,0.,0.])
    for _ in range(max_iter):
        pts = P(c,a[0],a[1],a[2])
        le = np.array([np.linalg.norm(pts[i]-bases[i]) for i in range(6)])
        e = le - L; en = np.linalg.norm(e)
        if en < tol: return True
        J = np.zeros((6,6))
        for i in range(6):
            d = pts[i]-bases[i]; ln = np.linalg.norm(d)
            if ln < 1e-10: return False
            u = d/ln; J[i,:3] = u; J[i,3:] = np.cross(pts[i]-c,u)
        try: delta = np.linalg.solve(J,-e)
        except: delta = np.linalg.lstsq(J,-e,rcond=None)[0]
        c += delta[:3]; a += delta[3:]
        if np.any(np.isnan(c))or np.any(np.isnan(a)): return False
    return en < 1.0

def test_config(bases, plat_ref, L, n=200, zmax=10):
    c0 = np.array([0,0,np.mean(bases[:,2])+L*0.95])
    coll = 0
    for _ in range(n):
        z = np.random.uniform(0,zmax,6)
        b = bases.copy(); b[:,2] = z
        if not fk(b, plat_ref, L, c0): coll += 1
    return coll/n

def rest_leg_len(plat, bases):
    return np.mean([np.linalg.norm(plat[i]-bases[i]) for i in range(6)])

print("=" * 70)
print("STEWART PLATFORM - TOI UU THAM SO CHONG SAP")
print("=" * 70)

# Default
plat0 = build_platform(DEFAULT_PR, DEFAULT_PD, 0)
base0 = build_base(DEFAULT_BR, DEFAULT_BD)
L0 = rest_leg_len(plat0, base0)
ref0 = plat0 - np.mean(plat0, axis=0)

print(f"\nDefault: BR={DEFAULT_BR}, PR={DEFAULT_PR}, BD={DEFAULT_BD}, PD={DEFAULT_PD}")
print(f"  Leg length (rest)={L0:.3f}, target LL={DEFAULT_LL}")
np.random.seed(42)
rate0 = test_config(base0, ref0, L0, 500)
print(f"  Default collapse rate: {rate0*100:.1f}%")

# Individual sweep
print("\n" + "-" * 70)
print("INDIVIDUAL PARAMETER SWEEP")
print("-" * 70)

def sweep_one(pname, vals):
    for v in vals:
        _br, _pr, _bd, _pd, _ll = DEFAULT_BR, DEFAULT_PR, DEFAULT_BD, DEFAULT_PD, DEFAULT_LL
        if pname == "baseRadius": _br = v
        elif pname == "platformRadius": _pr = v
        elif pname == "baseDistance": _bd = v
        elif pname == "platformDistance": _pd = v
        elif pname == "legLength": _ll = v
        
        plat = build_platform(_pr, _pd, 0)
        base = build_base(_br, _bd)
        L = _ll if pname == "legLength" else rest_leg_len(plat, base)
        ref = plat - np.mean(plat, axis=0)
        np.random.seed(42)
        rate = test_config(base, ref, L, 300)
        m = "OK" if rate==0 else ("WARN" if rate<0.05 else "BAD")
        print(f"    {pname}={v:.1f}: rate={rate*100:.1f}% [{m}]")

for pname, vals in [("baseRadius", [10,12,14,16,18,20]),
                    ("platformRadius", [5,7,9,11,13]),
                    ("baseDistance", [4,6,8,10,12]),
                    ("platformDistance", [2,4,6,8,10]),
                    ("legLength", [14,16,18,20,22,24])]:
    print(f"\n  {pname}:")
    sweep_one(pname, vals)

# Combo sweep
print("\n" + "-" * 70)
print("COMBO SWEEP (quick)")
print("-" * 70)

brs = [10,12,14,16,18]
prs = [5,7,9,11,13]
bds = [4,6,8,10,12]
pds = [2,4,6,8]
lls = [16,18,20,22]

best_r = 1.0; best_p = None
total = len(brs)*len(prs)*len(bds)*len(pds)*len(lls)
cnt = 0

for br in brs:
    for pr in prs:
        for bd in bds:
            for pd in pds:
                for ll in lls:
                    cnt += 1
                    if cnt % 200 == 0:
                        print(f"  Progress: {cnt}/{total} ({100*cnt/total:.0f}%)")
                    
                    try:
                        plat = build_platform(pr, pd, 0)
                        base = build_base(br, bd)
                        ref = plat - np.mean(plat, axis=0)
                        np.random.seed(42)
                        rate = test_config(base, ref, ll, 150)
                        if rate < best_r:
                            best_r = rate
                            Lr = rest_leg_len(plat, base)
                            best_p = (br, pr, bd, pd, ll, Lr)
                            print(f"\n  NEW BEST: rate={rate*100:.1f}%")
                            print(f"    BR={br}, PR={pr}, BD={bd}, PD={pd}, LL={ll} (rest={Lr:.2f})")
                            if rate == 0:
                                rate2 = test_config(base, ref, ll, 500)
                                if rate2 == 0:
                                    print(f"    CONFIRMED ZERO (500 tests)")
                                    break
                    except:
                        pass
                if best_r == 0: break
            if best_r == 0: break
        if best_r == 0: break
    if best_r == 0: break

print()
print("="*70)
print("RESULTS")
print("="*70)

if best_p:
    br, pr, bd, pd, ll, Lr = best_p
    print(f"\n  BEST PARAMETERS:")
    print(f"    baseRadius = {br}")
    print(f"    platformRadius = {pr}")
    print(f"    baseDistance = {bd}")
    print(f"    platformDistance = {pd}")
    print(f"    legLength = {ll}")
    print(f"    Rest leg length: {Lr:.2f}")
    print(f"    Collapse rate: {best_r*100:.1f}%")
    
    # Verify
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    ref = plat - np.mean(plat, axis=0)
    np.random.seed(123)
    rv = test_config(base, ref, ll, 1000)
    print(f"  Verified (1000 tests): {rv*100:.2f}%")
    print(f"\n  Comparison: Default={rate0*100:.1f}% vs Optimized={best_r*100:.1f}%")
    if rate0 > 0:
        print(f"  Improvement: {(rate0-best_r)/rate0*100:.0f}% reduction")

print("\n" + "="*70)
print("KEY INSIGHT")
print("="*70)
print("""
The key ratio: legLength / restLength.
- If legLength >> restLength: legs too long, platform slides too much
- If legLength << restLength: legs too short, can't reach
- Best: legLength ≈ restLength (nominal position is relaxed)

Also: base and platform should have similar size for stability.
""")
