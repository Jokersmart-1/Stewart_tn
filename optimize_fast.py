"""
Stewart Platform - Fast Parameter Optimization (khong dung Newton-Raphson)
======================================================================
Dung tieu chuan hinh hoc danh gia collapse nhanh hon 100x
"""
import numpy as np, math, time, itertools, sys

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
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def is_feasible_analytical(bases, plat_ref, L):
    """
    Kiem tra nhanh xem co ton tai (R,t) khong
    Dung grid R co muc tieu + pseudo-inverse
    """
    best_err = 1e10
    rs = plat_ref  # body-frame joint positions
    
    # Thu 3 grid resolutions
    for rr in np.linspace(-0.8, 0.8, 7):
        for pp in np.linspace(-0.8, 0.8, 7):
            for yy in np.linspace(-math.pi, math.pi, 10):
                Rmat = Rm(rr, pp, yy)
                q = (Rmat @ rs.T).T
                
                # Solve t tu (i=1,2,3)
                H = np.array([Rmat@(rs[0]-rs[1])-(bases[0]-bases[1]),
                              Rmat@(rs[0]-rs[2])-(bases[0]-bases[2]),
                              Rmat@(rs[0]-rs[3])-(bases[0]-bases[3])])
                if abs(np.linalg.det(H)) < 1e-8: continue
                
                def dot(a,b): return np.dot(a,b)
                s = np.array([
                    dot(q[0],bases[0])-dot(q[1],bases[1])+(dot(rs[1],rs[1])-dot(rs[0],rs[0])+dot(bases[1],bases[1])-dot(bases[0],bases[0]))/2,
                    dot(q[0],bases[0])-dot(q[2],bases[2])+(dot(rs[2],rs[2])-dot(rs[0],rs[0])+dot(bases[2],bases[2])-dot(bases[0],bases[0]))/2,
                    dot(q[0],bases[0])-dot(q[3],bases[3])+(dot(rs[3],rs[3])-dot(rs[0],rs[0])+dot(bases[3],bases[3])-dot(bases[0],bases[0]))/2,
                ])
                try: t_vec = np.linalg.solve(H, s)
                except: continue
                
                # Check all 6 legs
                err = sum((np.linalg.norm(t_vec+q[i]-bases[i])-L)**2 for i in range(6))
                if err < best_err: best_err = err
                if err < 0.5: return True, err
    
    return best_err < 3.0, best_err

def test_config_fast(bases, plat_ref, L, n=200, zmax=10):
    """Test collapse rate using analytical check (100x faster than Newton)"""
    coll = 0
    for _ in range(n):
        z = np.random.uniform(0, zmax, 6)
        b = bases.copy(); b[:,2] = z
        ok, _ = is_feasible_analytical(b, plat_ref, L)
        if not ok: coll += 1
    return coll / n

def rest_leg_len(plat, bases):
    return np.mean([np.linalg.norm(plat[i]-bases[i]) for i in range(6)])

print("=" * 70)
print("STEWART PLATFORM - TOI UU THAM SO (FAST)")
print("=" * 70)

# Default
DEFAULT_BR=14.7; DEFAULT_PR=8.5; DEFAULT_BD=11.2; DEFAULT_PD=4.8; DEFAULT_LL=18.0

plat0 = build_platform(DEFAULT_PR, DEFAULT_PD, 0)
base0 = build_base(DEFAULT_BR, DEFAULT_BD)
L0 = rest_leg_len(plat0, base0)
ref0 = plat0 - np.mean(plat0, axis=0)

print(f"\nDefault: BR={DEFAULT_BR}, PR={DEFAULT_PR}, BD={DEFAULT_BD}, PD={DEFAULT_PD}, LL={DEFAULT_LL}")
print(f"  Rest leg length={L0:.3f}")
np.random.seed(42)
t0=time.time()
rate0 = test_config_fast(base0, ref0, DEFAULT_LL, 500)
t1=time.time()
print(f"  Default collapse rate: {rate0*100:.1f}% (took {t1-t0:.1f}s)")

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
        ref = plat - np.mean(plat, axis=0)
        np.random.seed(42)
        rate = test_config_fast(base, ref, _ll, 300)
        m = "OK" if rate==0 else ("WARN" if rate<0.05 else "BAD")
        print(f"    {pname}={v:.1f}: rate={rate*100:.1f}% [{m}]")

for pname, vals in [("baseRadius", [10,12,14,16,18,20]),
                    ("platformRadius", [5,7,9,11,13]),
                    ("baseDistance", [4,6,8,10,12]),
                    ("platformDistance", [2,4,6,8,10]),
                    ("legLength", [14,16,18,20,22,24])]:
    print(f"\n  {pname}:")
    sweep_one(pname, vals)

# PAIRWISE check analysis
print("\n" + "-" * 70)
print("PAIRWISE CHECK ANALYSIS (legLength=18)")
print("-" * 70)
print("\n  Kiem tra pairwise cho default config khi Z thay doi:")
plat = build_platform(DEFAULT_PR, DEFAULT_PD, 0)
base = build_base(DEFAULT_BR, DEFAULT_BD)
L = DEFAULT_LL
np.random.seed(42)
pairwise_fail = 0
for _ in range(500):
    z = np.random.uniform(0, 10, 6)
    b = base.copy(); b[:,2] = z
    
    # Check pairwise: can edges exist?
    edges = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)]
    d_ij = {e: np.linalg.norm(plat[e[0]]-plat[e[1]]) for e in edges}
    ok = True
    for i,j in edges:
        bd = np.linalg.norm(b[i]-b[j])
        d = d_ij[(i,j)]
        if d < max(0, bd-2*L)-1e-6 or d > bd+2*L+1e-6:
            ok = False
            break
    if not ok: pairwise_fail += 1
print(f"  Pairwise fail: {pairwise_fail}/500 ({100*pairwise_fail/500:.1f}%)")

# COMBO SWEEP
print("\n" + "-" * 70)
print("COMBO SWEEP (finding best)")
print("-" * 70)

brs = [10,12,14,16,18]
prs = [5,7,9,11,13]
bds = [4,6,8,10,12]
pds = [2,4,6,8]
lls = [16,18,20,22]

best_r = 1.0; best_p = None
total = len(brs)*len(prs)*len(bds)*len(pds)*len(lls)
cnt = 0; t_start = time.time()

for br in brs:
    for pr in prs:
        for bd in bds:
            for pd in pds:
                for ll in lls:
                    cnt += 1
                    if cnt % 100 == 0:
                        elapsed = time.time() - t_start
                        rate_est = cnt/elapsed if elapsed>0 else 0
                        remain = (total-cnt)/rate_est if rate_est>0 else 0
                        print(f"  Progress: {cnt}/{total} ({cnt*100//total}%) est.remain:{remain:.0f}s")
                    
                    try:
                        plat = build_platform(pr, pd, 0)
                        base = build_base(br, bd)
                        ref = plat - np.mean(plat, axis=0)
                        np.random.seed(42)
                        rate = test_config_fast(base, ref, ll, 100)
                        if rate < best_r:
                            best_r = rate
                            Lr = rest_leg_len(plat, base)
                            best_p = (br, pr, bd, pd, ll, Lr)
                            print(f"\n  NEW BEST: rate={rate*100:.1f}%")
                            print(f"    BR={br} PR={pr} BD={bd} PD={pd} LL={ll} (rest={Lr:.2f})")
                            if rate == 0:
                                rate2 = test_config_fast(base, ref, ll, 500)
                                if rate2 == 0:
                                    print(f"    CONFIRMED ZERO (500 tests)")
                        sys.stdout.flush()
                    except Exception as e:
                        pass
                if best_r == 0: break
            if best_r == 0: break
        if best_r == 0: break
    if best_r == 0: break

elapsed_total = time.time() - t_start
print(f"\n\nTotal time: {elapsed_total:.0f}s")

print("\n" + "="*70)
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
    print(f"\n  Verification with 1000 tests...")
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    ref = plat - np.mean(plat, axis=0)
    np.random.seed(123)
    t0 = time.time()
    rv = test_config_fast(base, ref, ll, 1000)
    print(f"    Collapse rate: {rv*100:.2f}% (took {time.time()-t0:.1f}s)")
    print(f"\n  Comparison:")
    print(f"    Default: {rate0*100:.1f}%")
    print(f"    Optimized: {best_r*100:.1f}%")
    if rate0 > 0:
        print(f"    Improvement: {(rate0-best_r)/rate0*100:.0f}% reduction")

# Also test the original given joint coordinates
print("\n\n" + "="*70)
print("TEST WITH ORIGINAL GIVEN JOINT COORDINATES")
print("="*70)
print("""
Original Plat joints:
  P1 = [8.526, -7.232, 38.952]
  P2 = [10.526, -3.768, 38.952]
  P3 = [2.000, 11.000, 38.952]
  P4 = [-2.000, 11.000, 38.952]
  P5 = [-10.526, -3.768, 38.952]
  P6 = [-8.526, -7.232, 38.952]

Original Base joints:
  B1 = [3.100, -12.600, 0]
  B2 = [12.462, 3.615, 0]
  B3 = [9.362, 8.985, 0]
  B4 = [-9.362, 8.985, 0]
  B5 = [-12.462, 3.615, 0]
  B6 = [-3.100, -12.600, 0]
""")

plat_orig = np.array([
    [8.526279, -7.232051, 38.951552],
    [10.526279, -3.767949, 38.951552],
    [2.0, 11.0, 38.951552],
    [-2.0, 11.0, 38.951552],
    [-10.526279, -3.767949, 38.951552],
    [-8.526279, -7.232051, 38.951552],
])
base_orig = np.array([
    [3.1, -12.6, 0],
    [12.46192, 3.615321, 0],
    [9.36192, 8.984679, 0],
    [-9.36192, 8.984679, 0],
    [-12.46192, 3.615321, 0],
    [-3.1, -12.6, 0],
])
L_orig = np.mean([np.linalg.norm(plat_orig[i]-base_orig[i]) for i in range(6)])
ref_orig = plat_orig - np.mean(plat_orig, axis=0)

print(f"  Original leg length (rest) = {L_orig:.4f}")
print(f"  (Target LL should be close to this for stability)")
print()

for test_ll in [16, 18, 20, 22, L_orig]:
    np.random.seed(42)
    rate = test_config_fast(base_orig, ref_orig, test_ll, 300)
    print(f"  LL={test_ll:.1f}: collapse rate={rate*100:.1f}%")
