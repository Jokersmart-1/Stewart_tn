"""
Create animation video of Stewart Platform collapse
====================================================
Shows a smooth transition from stable → collapsed configuration
"""

import numpy as np
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import imageio

# ============================================================
# GEOMETRY (same as simulation)
# ============================================================
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

LEG_LENS = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])
C0 = np.mean(PLAT, axis=0)
REF = PLAT - C0
CONN = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)]

def R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,   cp*sr,           cp*cr]
    ])

def P(c, r, p, y):
    return c + (R(r, p, y) @ REF.T).T

def fk(bases, n=50, tol=1e-4):
    """Forward kinematics"""
    c = np.array([0., 0., 37.])
    a = np.array([0., 0., 0.])
    
    for _ in range(n):
        pts = P(c, a[0], a[1], a[2])
        L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = L - LEG_LENS
        en = np.linalg.norm(e)
        if en < tol:
            return True, c, a
        
        J = np.zeros((6,6))
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
    
    return True, c, a

# ============================================================
# FIND COLLAPSE CONFIGURATION
# ============================================================
np.random.seed(42)

# Find one good collapse example
collapse_target = None
stable_config = None

for i in range(100):
    z = np.random.uniform(0, 10, 6)
    bases = BASE0.copy()
    bases[:,2] = z
    
    ok, c, a = fk(bases)
    if ok:
        if stable_config is None:
            stable_config = (z.copy(), bases.copy(), c.copy(), a.copy())
    
    # Check if it would collapse
    if ok:
        pts = P(c, a[0], a[1], a[2])
        n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
        minz = np.min(pts[:,2])
    else:
        tilt = 0
        minz = 999
    
    # Check reasonable values (avoid infinity)
    if not ok or (ok and (tilt > 60 or minz < -2 or abs(c[0]) > 50 or abs(c[1]) > 50)):
        collapse_target = (z.copy(), bases.copy(), (0,0,0))
        break
    if stable_config is None:
        stable_config = (z.copy(), bases.copy(), c.copy(), a.copy())

print(f"Found collapse config: Z={np.round(collapse_target[0], 2)}")
    # Check if it would collapse
    if ok:
        pts = P(c, a[0], a[1], a[2])
        # Check for reasonable values (solver diverged)
        if np.any(np.abs(pts) > 1e4):
            ok = False
        n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
        minz = np.min(pts[:,2])
    else:
        tilt = 0
        minz = 999
    
    if not ok:
        # This is a collapse case (solver failed)
        collapse_target = (z.copy(), bases.copy(), False)
        break
    if stable_config is None:
        stable_config = (z.copy(), bases.copy(), c.copy(), a.copy())

print(f"Found collapse config: Z={np.round(collapse_target[0], 2)}")

print(f"Found collapse config: Z={np.round(collapse_target[0], 2)}")
print(f"  {collapse_target[2]}")

# ============================================================
# CREATE VIDEO - Smooth transition from stable to collapsed
# ============================================================
print("Creating video...")

if stable_config is not None:
    start_z = stable_config[0]
else:
    start_z = np.zeros(6)

end_z = collapse_target[0]
N_FRAMES = 150
FPS = 15

# Use imageio to create GIF
frames = []
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

for frame in range(N_FRAMES):
    ax.clear()
    
    # Interpolate Z smoothly
    # Use easing function for smooth transition
    t = frame / N_FRAMES
    # Ease-in-out cubic
    if t < 0.5:
        t_eased = 2 * t * t
    else:
        t_eased = 1 - (-2*t + 2)**2 / 2
    
    z = start_z + t_eased * (end_z - start_z)
    bases = BASE0.copy()
    bases[:,2] = z
    
    # Try to solve FK
    ok, c, a = fk(bases)
    
    if ok:
        plat = P(c, a[0], a[1], a[2])
        n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
        status = "STABLE"
    else:
        # If FK fails, show best guess
        c_fail = np.array([0.,0.,35.])
        a_fail = np.array([0.,0.,0.])
        plat = P(c_fail, a_fail[0], a_fail[1], a_fail[2])
        tilt = 0
        status = "COLLAPSED!"
    
    is_collapsed = not ok or tilt > 60 or np.min(plat[:,2]) < -2
    
    # Draw
    # Base
    ax.scatter(bases[:,0], bases[:,1], bases[:,2], c='blue', s=80, marker='o', label='Base joints')
    for i,j in CONN:
        ax.plot([bases[i,0], bases[j,0]], [bases[i,1], bases[j,1]], [bases[i,2], bases[j,2]], 
                'blue', linewidth=1, alpha=0.3)
    
    # Platform
    color = 'red' if is_collapsed else 'green'
    label = 'Platform (COLLAPSED)' if is_collapsed else 'Platform'
    ax.scatter(plat[:,0], plat[:,1], plat[:,2], c=color, s=100, marker='^', label=label)
    for i,j in CONN:
        ax.plot([plat[i,0], plat[j,0]], [plat[i,1], plat[j,1]], [plat[i,2], plat[j,2]], 
                color, linewidth=3)
    
    # Legs
    for i in range(6):
        ax.plot([bases[i,0], plat[i,0]], [bases[i,1], plat[i,1]], [bases[i,2], plat[i,2]], 
                'gray', linewidth=2, alpha=0.7)
    
    # Info text
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-5, 50)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Title
    status_text = f"Frame {frame+1}/{N_FRAMES} | {status}"
    if ok:
        status_text += f" | Tilt: {tilt:.1f}° | Center Z: {c[2]:.1f}"
    ax.set_title(status_text, fontsize=12)
    ax.legend(loc='upper left')
    
    # Add Z values as text
    z_text = f"Base Z: [{z[0]:.1f} {z[1]:.1f} {z[2]:.1f} {z[3]:.1f} {z[4]:.1f} {z[5]:.1f}]"
    ax.text2D(0.02, 0.02, z_text, transform=ax.transAxes, fontsize=9,
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Capture frame
    fig.canvas.draw()
    
    # Convert to RGB array
    buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    frames.append(buf)
    
    if frame % 20 == 0:
        print(f"  Frame {frame+1}/{N_FRAMES}")

plt.close(fig)

# Save as GIF
gif_path = 'd:/final/collapse_animation.gif'
print(f"Saving GIF to {gif_path}...")
imageio.mimsave(gif_path, frames, fps=FPS, loop=0)
print(f"GIF saved! {gif_path}")
print(f"  {len(frames)} frames, {FPS} FPS, duration: {len(frames)/FPS:.1f}s")

# Also save as MP4 if possible
try:
    mp4_path = 'd:/final/collapse_animation.mp4'
    print(f"Saving MP4 to {mp4_path}...")
    imageio.mimsave(mp4_path, frames, fps=FPS, codec='libx264')
    print(f"MP4 saved! {mp4_path}")
except Exception as e:
    print(f"Could not save MP4: {e}")
