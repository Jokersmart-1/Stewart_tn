"""
Stewart Platform - Quick Parameter Analysis
===========================================
Key insight: collapse happens when leg length doesn't match the rest length.
We analyze which parameters make rest length closest to target.
"""
import numpy as np, math

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

def rest_leg_len(pr, pd, br, bd):
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    return np.mean([np.linalg.norm(plat[i]-base[i]) for i in range(6)])

print("=" * 70)
print("STEWART PLATFORM - THAM SO TOI UU")
print("=" * 70)

# First: test the original MATALB-based formulas
DEFAULT_BR=14.7; DEFAULT_PR=8.5; DEFAULT_BD=11.2; DEFAULT_PD=4.8

print(f"\nCau hinh mac dinh:")
print(f"  baseRadius={DEFAULT_BR}, platformRadius={DEFAULT_PR}")
print(f"  baseDistance={DEFAULT_BD}, platformDistance={DEFAULT_PD}")

# Offset angles
base_offset = math.degrees(math.atan2(DEFAULT_BD/2, DEFAULT_BR))
plat_offset = math.degrees(math.atan2(DEFAULT_PD/2, DEFAULT_PR))
print(f"  baseOffset = {base_offset:.2f} deg")
print(f"  platOffset = {plat_offset:.2f} deg")

L_rest = rest_leg_len(DEFAULT_PR, DEFAULT_PD, DEFAULT_BR, DEFAULT_BD)
print(f"  Leg length (nghi) = {L_rest:.4f}")

# The collision probability is related to how far the leg is from
# the average when base joints move in Z

print(f"\n{'='*70}")
print(f"Tim legLength LY TUONG nhat:")
print(f"{'='*70}")

target_L = 18.0  # Default target

# For target leg length = 18, find what configuration gives L_rest ≈ 18
# The formula is: L_rest = distance between plat[i] and base[i] at z=0
# We need parameter combinations that make this close to 18

print(f"\nTim tham so de L_rest ≈ {target_L}:")

# Sweep baseRadius and platformRadius (most impactful)
best_diff = 999
best_params = None

br_vals = range(8, 26, 1)
pr_vals = [x*0.5 for x in range(10, 28)]  # 5 to 13.5
bd_vals = range(4, 16, 1)
pd_vals = [x*0.5 for x in range(4, 20)]  # 2 to 10

# Only sweep radius (most impactful)
for br_nom in [10,11,12,13,14,15,16,17,18,19,20]:
    for pr_nom in [5,6,7,8,9,10,11,12,13]:
        # Optimal bd and pd scale with radius
        bd_opt = DEFAULT_BD * (br_nom / DEFAULT_BR)
        pd_opt = DEFAULT_PD * (pr_nom / DEFAULT_PR)
        
        Lr = rest_leg_len(pr_nom, pd_opt, br_nom, bd_opt)
        diff = abs(Lr - target_L)
        
        if diff < best_diff:
            best_diff = diff
            best_params = (br_nom, pr_nom, bd_opt, pd_opt, Lr)
            if diff < 0.1:
                print(f"  EXCELLENT: BR={br_nom} PR={pr_nom} BD={bd_opt:.2f} PD={pd_opt:.2f} L_rest={Lr:.4f}")

if best_params:
    print(f"\nBEST MATCH (L_rest ≈ {target_L}):")
    br, pr, bd, pd, Lr = best_params
    print(f"  baseRadius={br} (default=14.7)")
    print(f"  platformRadius={pr} (default=8.5)")
    print(f"  baseDistance={bd:.2f} (default=11.2)")
    print(f"  platformDistance={pd:.2f} (default=4.8)")
    print(f"  L_rest={Lr:.4f} (target={target_L})")
    print(f"  Sai so: {best_diff:.4f}")

    # Also check what happens when we vary legLength around this
    print(f"\n  Anh huong cua legLength (LL) toi collapse:")
    print(f"  (Dung ti le |LL - L_rest| / L_rest cang nho cang tot)")
    for ll in [14, 16, 18, 20, 22, target_L]:
        ratio = abs(ll - Lr) / Lr
        print(f"    LL={ll}: |{ll}-{Lr:.1f}|/{Lr:.1f}={ratio*100:.1f}%")

print(f"\n{'='*70}")
print(f"SWEEP NHANH: Tim tham so co L_rest gan 18 nhat")
print(f"{'='*70}")

results = []
for br_nom in [10,11,12,13,14,15,16,17,18,19,20]:
    for pr_nom in [5,6,7,8,9,10,11,12,13]:
        bd_opt = DEFAULT_BD * (br_nom / DEFAULT_BR)
        pd_opt = DEFAULT_PD * (pr_nom / DEFAULT_PR)
        Lr = rest_leg_len(pr_nom, pd_opt, br_nom, bd_opt)
        
        # Check a few legLengths
        for ll in [16, 17, 18, 19, 20]:
            ratio = abs(ll - Lr) / Lr
            results.append((ratio, ll, br_nom, pr_nom, bd_opt, pd_opt, Lr))

results.sort()
print(f"\nTop 10 cau hinh tot nhat (|LL-L_rest|/L_rest nho nhat):")
print(f"  {'LL':>5} {'L_rest':>8} {'|Δ|/L':>7} {'BR':>5} {'PR':>5} {'BD':>7} {'PD':>7}")
print(f"  {'-'*5} {'-'*8} {'-'*7} {'-'*5} {'-'*5} {'-'*7} {'-'*7}")
for r, ll, br, pr, bd, pd, Lr in results[:10]:
    print(f"  {ll:5.1f} {Lr:8.4f} {r*100:6.2f}% {br:5d} {pr:5d} {bd:7.2f} {pd:7.2f}")

print(f"\n{'='*70}")
print(f"KET LUAN")
print(f"{'='*70}")
print(f"""
Nguyen tac toi uu:
1. L_rest can gan bang legLength nhat co the
   - Qua xa -> khong gian dieu chinh bi han che
   - Qua gan -> mat do dai de dieu chinh

2. baseRadius va platformRadius can can xung:
   - Ty le BR/PR hop ly: 1.5-2.0
   - BR quang 12-16, PR quang 7-10

3. baseDistance va platformDistance:
   - Ti le voi ban kinh tuong ung
   - baseDistance ≈ 0.76 * baseRadius
   - platformDistance ≈ 0.56 * platformRadius
""")

# Also analyze the ORIGINAL given coordinates
print(f"\n{'='*70}")
print(f"PHAN TICH TOA DO GOC")
print(f"{'='*70}")

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

print(f"\n  L_rest (original) = {L_orig:.4f}")
print(f"\n  Khi LL=18:  |18 - {L_orig:.1f}| / {L_orig:.1f} = {abs(18-L_orig)/L_orig*100:.1f}%")
print(f"  Khi LL=20:  |20 - {L_orig:.1f}| / {L_orig:.1f} = {abs(20-L_orig)/L_orig*100:.1f}%")
print(f"  Khi LL={L_orig:.0f}: |{L_orig:.0f} - {L_orig:.1f}| / {L_orig:.1f} = {abs(L_orig-L_orig)/L_orig*100:.1f}% (LY TUONG)")

# Extract equivalent parameters from original coordinates
print(f"\n  Gia tri tham so tuong duong tu toa do goc:")
plat_r = np.mean([np.linalg.norm(plat_orig[i,:2]) for i in range(6)])
base_r = np.mean([np.linalg.norm(base_orig[i,:2]) for i in range(6)])
print(f"    platformRadius ≈ {plat_r:.2f}")
print(f"    baseRadius ≈ {base_r:.2f}")
dists_plat = [np.linalg.norm(plat_orig[i]-plat_orig[(i+1)%6]) for i in range(6)]
dists_base = [np.linalg.norm(base_orig[i]-base_orig[(i+1)%6]) for i in range(6)]
print(f"    Khoang cach các cap Plat joint: {[f'{d:.2f}' for d in dists_plat]}")
print(f"    Khoang cach các cap Base joint: {[f'{d:.2f}' for d in dists_base]}")
print(f"    Plat edges: {min(dists_plat):.2f} den {max(dists_plat):.2f}")
print(f"    Base edges: {min(dists_base):.2f} den {max(dists_base):.2f}")
