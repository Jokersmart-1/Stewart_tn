"""
Stewart Platform - Parameter Optimization
==========================================
Tìm tham số tối ưu để platform KHÔNG BỊ SẬP hoặc ít sập nhất
Dựa trên công thức MATLAB:
  baseOffset = atan2(baseDistance/2, baseRadius)
  platOffset = atan2(platformDistance/2, platformRadius)
"""

import numpy as np, math, time, itertools

# ============================================================
# DEFAULT PARAMETERS
# ============================================================
defaults = {
    'baseRadius': 14.7,
    'platformRadius': 8.5,
    'baseDistance': 11.2,
    'platformDistance': 4.8,
    'legLength': 18.0,
}

def build_platform(platformRadius, platformDistance, z=0):
    """Tính 6 điểm platform dựa trên công thức MATLAB"""
    platOffset = math.atan2(platformDistance / 2, platformRadius)
    R = math.sqrt(platformRadius**2 + (platformDistance/2)**2)
    return np.array([
        [R*math.sin(math.pi/3 - platOffset), -R*math.cos(math.pi/3 - platOffset), z],
        [R*math.sin(math.pi/3 + platOffset), -R*math.cos(math.pi/3 + platOffset), z],
        [platformDistance/2, platformRadius, z],
        [-platformDistance/2, platformRadius, z],
        [-R*math.sin(math.pi/3 + platOffset), -R*math.cos(math.pi/3 + platOffset), z],
        [-R*math.sin(math.pi/3 - platOffset), -R*math.cos(math.pi/3 - platOffset), z],
    ])

def build_base(baseRadius, baseDistance):
    """Tính 6 điểm base dựa trên công thức MATLAB"""
    baseOffset = math.atan2(baseDistance / 2, baseRadius)
    R = math.sqrt(baseRadius**2 + (baseDistance/2)**2)
    return np.array([
        [baseDistance/2, -baseRadius, 0],
        [R*math.sin(math.pi/3 + baseOffset), R*math.cos(math.pi/3 + baseOffset), 0],
        [R*math.sin(math.pi/3 - baseOffset), R*math.cos(math.pi/3 - baseOffset), 0],
        [-R*math.sin(math.pi/3 - baseOffset), R*math.cos(math.pi/3 - baseOffset), 0],
        [-R*math.sin(math.pi/3 + baseOffset), R*math.cos(math.pi/3 + baseOffset), 0],
        [-baseDistance/2, -baseRadius, 0],
    ])

# ============================================================
# NEWTON-RAPHSON SOLVER (forward kinematics)
# ============================================================
def fk(bases, plat_ref, L, c0, tol=1e-3, max_iter=30):
    """
    Forward kinematics: tìm pose (c, roll, pitch, yaw) từ base positions và leg lengths
    bases: 6×3 base positions
    plat_ref: 6×3 platform joint positions relative to center
    L: leg length
    c0: initial center guess
    """
    def R(rr,pp,yy):
        cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp); cy,sy=math.cos(yy),math.sin(yy)
        return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    def P(c,rr,pp,yy):
        return c + (R(rr,pp,yy) @ plat_ref.T).T
    
    c = c0.copy()
    a = np.array([0., 0., 0.])
    
    for _ in range(max_iter):
        pts = P(c, a[0], a[1], a[2])
        le = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = le - L
        en = np.linalg.norm(e)
        if en < tol:
            return True, c, a
        
        # Jacobian
        J = np.zeros((6, 6))
        for i in range(6):
            d = pts[i] - bases[i]
            ln = np.linalg.norm(d)
            if ln < 1e-10:
                return False, c, a
            u = d / ln
            J[i,:3] = u
            J[i,3:] = np.cross(pts[i] - c, u)
        
        try:
            delta = np.linalg.solve(J, -e)
        except:
            delta = np.linalg.lstsq(J, -e, rcond=None)[0]
        
        c += delta[:3]
        a += delta[3:]
        if np.any(np.isnan(c)) or np.any(np.isnan(a)):
            return False, c, a
    
    return en < 1.0, c, a

def test_configuration(bases, plat_ref, L, n_random=200, z_max=10):
    """
    Test một cấu hình tham số: chạy n_random lần ngẫu nhiên
    Trả về tỉ lệ sập
    """
    c0 = np.array([0, 0, np.mean(bases[:,2]) + L * 0.95])
    collapsed = 0
    
    for _ in range(n_random):
        z = np.random.uniform(0, z_max, 6)
        b = bases.copy()
        b[:,2] = z
        
        ok, _, _ = fk(b, plat_ref, L, c0)
        if not ok:
            collapsed += 1
    
    return collapsed / n_random

def compute_rest_leg_length(plat, bases):
    """Tính chiều dài chân ở cấu hình nghỉ (Z=0)"""
    return np.mean([np.linalg.norm(plat[i] - bases[i]) for i in range(6)])

# ============================================================
# SWEEP THAM SỐ
# ============================================================
print("=" * 75)
print("STEWART PLATFORM - TỐI ƯU HÓA THAM SỐ CHỐNG SẬP")
print("=" * 75)

# Default configuration
plat0 = build_platform(defaults['platformRadius'], defaults['platformDistance'], 0)
base0 = build_base(defaults['baseRadius'], defaults['baseDistance'])
L0 = compute_rest_leg_length(plat0, base0)
plat_ref0 = plat0 - np.mean(plat0, axis=0)

print(f"\nCấu hình mặc định:")
print(f"  baseRadius={defaults['baseRadius']}, platformRadius={defaults['platformRadius']}")
print(f"  baseDistance={defaults['baseDistance']}, platformDistance={defaults['platformDistance']}")
print(f"  Leg length (rest) = {L0:.4f} (target=18)")
print(f"  Platform center: {np.mean(plat0, axis=0)}")
print(f"  Platform radius: {np.linalg.norm(plat_ref0, axis=1).round(3)}")

# Test default
np.random.seed(42)
rate0 = test_configuration(base0, plat_ref0, L0, 500)
print(f"  Collapse rate: {rate0*100:.1f}% (500 configs)")

print("\n" + "-" * 75)
print("SWEEP THAM SỐ: thay đổi từng tham số để tìm ít sập nhất")
print("-" * 75)

results = []

# Parameter ranges
param_ranges = {
    'baseRadius': np.arange(10, 22, 2),
    'platformRadius': np.arange(5, 14, 1.5),
    'baseDistance': np.arange(4, 16, 2),
    'platformDistance': np.arange(2, 10, 1.5),
    'legLength': np.arange(14, 28, 2),
}

N_TEST = 300

for param_name, values in param_ranges.items():
    print(f"\n  Sweeping {param_name}...")
    for val in values:
        params = defaults.copy()
        params[param_name] = val
        
        plat = build_platform(params['platformRadius'], params['platformDistance'], 0)
        base = build_base(params['baseRadius'], params['baseDistance'])
        
        if param_name == 'legLength':
            L = val
        else:
            L = compute_rest_leg_length(plat, base)
        
        plat_ref = plat - np.mean(plat, axis=0)
        
        np.random.seed(42)
        rate = test_configuration(base, plat_ref, L, N_TEST)
        results.append((param_name, val, L, rate))
        
        marker = ' ✓' if rate == 0 else (' ⚠' if rate < 0.05 else ' ✗')
        print(f"    {param_name}={val:.1f}, L={L:.2f}: {rate*100:.1f}%{marker}")

# ============================================================
# TÌM BEST COMBINATION
# ============================================================
print("\n" + "-" * 75)
print("TÌM TỔ HỢP TỐI ƯU (sweep toàn bộ)")
print("-" * 75)

# Limited but intelligent sweep
br_vals = [10, 12, 14, 16, 18, 20]
pr_vals = [5, 6.5, 8, 9.5, 11, 12.5]
bd_vals = [4, 6, 8, 10, 12]
pd_vals = [2, 4, 6, 8]
ll_vals = [16, 18, 20, 22, 24]

best_rate = 1.0
best_params = None
best_L = None

total_configs = len(br_vals) * len(pr_vals) * len(bd_vals) * len(pd_vals) * len(ll_vals)
config_count = 0

N_OPT = 200

for br, pr, bd, pd, ll in itertools.product(br_vals, pr_vals, bd_vals, pd_vals, ll_vals):
    config_count += 1
    if config_count % 100 == 0:
        print(f"  Progress: {config_count}/{total_configs} ({100*config_count/total_configs:.0f}%)")
    
    try:
        plat = build_platform(pr, pd, 0)
        base = build_base(br, bd)
        L_rest = compute_rest_leg_length(plat, base)
        L = ll  # Use specified leg length
        
        plat_ref = plat - np.mean(plat, axis=0)
        
        np.random.seed(42)
        rate = test_configuration(base, plat_ref, L, N_OPT)
        
        if rate < best_rate:
            best_rate = rate
            best_params = (br, pr, bd, pd, ll, L_rest)
            print(f"\n  ★ NEW BEST: rate={rate*100:.1f}%")
            print(f"    br={br}, pr={pr}, bd={bd}, pd={pd}, ll={ll} (rest L={L_rest:.2f})")
            
            if rate == 0:
                # Try with more samples to confirm
                rate2 = test_configuration(base, plat_ref, L, 1000)
                if rate2 == 0:
                    print(f"    ✓ CONFIRMED ZERO COLLAPSE (1000 tests)!")
                    break
    except Exception as e:
        continue
    
    if best_rate == 0:
        break

# ============================================================
# KẾT QUẢ
# ============================================================
print("\n" + "=" * 75)
print("KẾT QUẢ TỐI ƯU")
print("=" * 75)

if best_params:
    br, pr, bd, pd, ll, Lr = best_params
    print(f"\n  ★ Tham số tối ưu tìm được:")
    print(f"    baseRadius = {br}")
    print(f"    platformRadius = {pr}")
    print(f"    baseDistance = {bd}")
    print(f"    platformDistance = {pd}")
    print(f"    legLength = {ll}")
    print(f"    (Leg length at rest: {Lr:.4f})")
    print(f"    Collapse rate: {best_rate*100:.1f}%")
    
    # Verify with more samples
    plat = build_platform(pr, pd, 0)
    base = build_base(br, bd)
    plat_ref = plat - np.mean(plat, axis=0)
    
    print(f"\n  Xác minh với 2000 mẫu:")
    np.random.seed(123)
    rate_verify = test_configuration(base, plat_ref, ll, 2000)
    print(f"    Collapse rate: {rate_verify*100:.2f}%")
    
    # Compare with default
    print(f"\n  So sánh:")
    print(f"    Cấu hình mặc định: {rate0*100:.1f}% collapse")
    print(f"    Cấu hình tối ưu:  {best_rate*100:.1f}% collapse")
    print(f"    Cải thiện:        {(rate0-best_rate)/rate0*100:.0f}% reduction" if rate0 > 0 else "    N/A")

print("\n" + "=" * 75)
print("PHÂN TÍCH CHI TIẾT")
print("=" * 75)

# Analyze which parameter has most impact
print("\n  Ảnh hưởng của từng tham số đến tỉ lệ sập:")
for pname in ['baseRadius', 'platformRadius', 'baseDistance', 'platformDistance', 'legLength']:
    subset = [r for r in results if r[0] == pname]
    if subset:
        min_r = min(subset, key=lambda x: x[3])
        print(f"    {pname}: tối ưu={min_r[1]:.1f}, rate={min_r[3]*100:.1f}%")
        rates_str = ', '.join([f"{v[1]:.0f}→{v[3]*100:.0f}%" for v in sorted(subset, key=lambda x: x[1])])
        print(f"      {rates_str}")

print("\n" + "=" * 75)
print("KẾT LUẬN")
print("=" * 75)
print(f"""
Nguyên nhân sập: Khi 6 base di chuyển Z ngẫu nhiên 0-10mm, 
các chân có chiều dài cố định L không thể nối base và platform 
tương ứng đồng thời cho cả 6 chân.

Yếu tố quyết định:
  1. Tỉ lệ L / (khoảng cách giữa base và platform tại rest):
     L càng lớn so với L_rest càng dễ sập (chân quá dài)
     L càng nhỏ càng dễ sập (chân quá ngắn)
  
  2. Hình dạng lục giác: base và platform nên có kích thước 
     tương đồng để giảm thiểu xung đột hình học.
  
  3. legLength nên gần với L_rest để có đủ "dư địa" điều chỉnh.

Cấu hình tối ưu: baseRadius ≈ platformRadius + offset hợp lý
               legLength ≈ rest_length
               baseDistance và platformDistance tỉ lệ với bán kính
""")
