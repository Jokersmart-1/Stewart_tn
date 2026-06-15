"""Minimal test of the Stewart platform solver"""
import numpy as np
import math
import sys

print("Starting test...")
sys.stdout.flush()

PLAT = np.array([
    [8.526279, -7.232051, 38.951552],
    [10.526279, -3.767949, 38.951552],
    [2.000000, 11.000000, 38.951552],
    [-2.000000, 11.000000, 38.951552],
    [-10.526279, -3.767949, 38.951552],
    [-8.526279, -7.232051, 38.951552],
])

BASE0 = np.array([
    [3.100000, -12.600000, 0.000000],
    [12.461920, 3.615321, 0.000000],
    [9.361920, 8.984679, 0.000000],
    [-9.361920, 8.984679, 0.000000],
    [-12.461920, 3.615321, 0.000000],
    [-3.100000, -12.600000, 0.000000],
])

LEG_LEN = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])
print(f"Leg lengths: {LEG_LEN}")
sys.stdout.flush()

PLAT_CENTER = np.mean(PLAT, axis=0)
REF_P = PLAT - PLAT_CENTER
print(f"Platform center: {PLAT_CENTER}")
sys.stdout.flush()

def rot_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def plat_points(c, r, p, y):
    R = rot_mat(r, p, y)
    return c + (R @ REF_P.T).T

# Test initial config
c = np.array([0.0, 0.0, 35.0])
rpy = np.array([0.0, 0.0, 0.0])
pts = plat_points(c, rpy[0], rpy[1], rpy[2])
print(f"Test points:\n{pts}")
sys.stdout.flush()

L = np.array([np.linalg.norm(pts[i] - BASE0[i]) for i in range(6)])
print(f"Test lengths: {L}")
print(f"Target lengths: {LEG_LEN}")
print(f"Error: {np.linalg.norm(L - LEG_LEN)}")
sys.stdout.flush()

# Newton iteration  
print("\nRunning Newton-Raphson solver...")
sys.stdout.flush()

for it in range(50):
    pts = plat_points(c, rpy[0], rpy[1], rpy[2])
    L = np.array([np.linalg.norm(pts[i] - BASE0[i]) for i in range(6)])
    e = L - LEG_LEN
    en = np.linalg.norm(e)
    
    if en < 1e-5:
        print(f"  Converged at iteration {it}, error={en}")
        sys.stdout.flush()
        break
    
    J = np.zeros((6,6))
    for i in range(6):
        d = pts[i] - BASE0[i]
        ln = np.linalg.norm(d)
        if ln < 1e-10:
            print(f"  Leg too short at iteration {it}")
            sys.stdout.flush()
            break
        u = d / ln
        J[i,:3] = u
        ri = pts[i] - c
        J[i,3:] = np.cross(ri, u)
    
    try:
        delta = np.linalg.solve(J, -e)
    except np.linalg.LinAlgError:
        print(f"  Singular J at iteration {it}")
        sys.stdout.flush()
        delta = np.linalg.lstsq(J, -e, rcond=None)[0]
    
    c += 0.3 * delta[:3]
    rpy += 0.3 * delta[3:]
    
    if it % 10 == 0:
        print(f"  Iter {it}: error={en}, center={c}, rpy={rpy}")
        sys.stdout.flush()
    
    if np.any(np.isnan(c)) or np.any(np.isnan(rpy)):
        print(f"  NaN detected!")
        sys.stdout.flush()
        break

print(f"\nFinal center: {c}")
print(f"Final rpy: {rpy}")
sys.stdout.flush()

# Now test with a random Z configuration
print("\n--- Testing with random Z ---")
sys.stdout.flush()
np.random.seed(42)
z = np.random.uniform(0, 10, 6)
bases = BASE0.copy()
bases[:,2] = z
print(f"Z displacements: {z}")
sys.stdout.flush()

c = np.array([0.0, 0.0, 35.0])
rpy = np.array([0.0, 0.0, 0.0])

for it in range(50):
    pts = plat_points(c, rpy[0], rpy[1], rpy[2])
    L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
    e = L - LEG_LEN
    en = np.linalg.norm(e)
    
    if en < 1e-5:
        print(f"  Converged at iteration {it}, error={en}")
        sys.stdout.flush()
        break
    
    J = np.zeros((6,6))
    for i in range(6):
        d = pts[i] - bases[i]
        ln = np.linalg.norm(d)
        if ln < 1e-10:
            break
        u = d / ln
        J[i,:3] = u
        ri = pts[i] - c
        J[i,3:] = np.cross(ri, u)
    
    try:
        delta = np.linalg.solve(J, -e)
    except:
        delta = np.linalg.lstsq(J, -e, rcond=None)[0]
    
    c += 0.3 * delta[:3]
    rpy += 0.3 * delta[3:]
    
    if it == 0 or it == 10 or it == 20 or it == 40:
        print(f"  Iter {it}: error={en:.6f}")
        sys.stdout.flush()

print(f"\nFinal: error={np.linalg.norm(L - LEG_LEN):.6f}")
print(f"Center: {c}")
print(f"Tilt: {math.degrees(math.acos(np.clip((rot_mat(rpy[0],rpy[1],rpy[2])@np.array([0,0,1]))[2], -1, 1))):.2f}deg")
sys.stdout.flush()

print("\nDONE!")