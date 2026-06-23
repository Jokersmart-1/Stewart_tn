"""
Stewart Platform - Collapse Animation Video
Shows smooth transition from stable to collapsed configuration
"""
import numpy as np
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import imageio
import os

# Geometry
PLAT = np.array([
    [8.526279, -7.232051, 38.951552],
    [10.526279, -3.767949, 38.951552],
    [2.0, 11.0, 38.951552],
    [-2.0, 11.0, 38.951552],
    [-10.526279, -3.767949, 38.951552],
    [-8.526279, -7.232051, 38.951552],
])
BASE0 = np.array([
    [3.1, -12.6, 0],
    [12.46192, 3.615321, 0],
    [9.36192, 8.984679, 0],
    [-9.36192, 8.984679, 0],
    [-12.46192, 3.615321, 0],
    [-3.1, -12.6, 0],
])
LEG = np.array([np.linalg.norm(PLAT[i] - BASE0[i]) for i in range(6)])
C0 = np.mean(PLAT, axis=0)
REF = PLAT - C0
CONN = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)]

def R(r, p, y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])

def P(c, r, p, y):
    return c + (R(r, p, y) @ REF.T).T

def fk(bases, n=30, tol=1e-3):
    c = np.array([0., 0., 37.])
    a = np.array([0., 0., 0.])
    for _ in range(n):
        pts = P(c, a[0], a[1], a[2])
        L = np.array([np.linalg.norm(pts[i] - bases[i]) for i in range(6)])
        e = L - LEG; en = np.linalg.norm(e)
        if en < tol: return True, c, a
        J = np.zeros((6,6))
        for i in range(6):
            d = pts[i] - bases[i]; ln = np.linalg.norm(d)
            if ln < 1e-10: return False, c, a
            u = d/ln; J[i,:3] = u; J[i,3:] = np.cross(pts[i]-c, u)
        try:
            delta = np.linalg.solve(J, -e)
        except:
            delta = np.linalg.lstsq(J, -e, rcond=None)[0]
        c += delta[:3]; a += delta[3:]
        if np.any(np.isnan(c)) or np.any(np.isnan(a)):
            return False, c, a
    return True, c, a

# Find a stable config and a collapsing config
np.random.seed(42)
stable_z = None
collapse_z = None

for i in range(200):
    z = np.random.uniform(0, 10, 6)
    bases = BASE0.copy(); bases[:,2] = z
    ok, c, a = fk(bases)
    
    if ok:
        if stable_z is None:
            stable_z = z.copy()
        pts = P(c, a[0], a[1], a[2])
        n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
        minz = np.min(pts[:,2])
        div = np.any(np.abs(pts) > 1e4) or np.any(np.abs(c) > 500)
        if tilt > 60 or minz < -2 or div:
            if collapse_z is None:
                collapse_z = z.copy()
                break

if collapse_z is None:
    collapse_z = z.copy()

print(f"Stable Z:   {np.round(stable_z, 2)}")
print(f"Collapse Z: {np.round(collapse_z, 2)}")

# Generate video frames
N_FRAMES = 120
FPS = 12

frames = []
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

for frame in range(N_FRAMES):
    ax.clear()
    
    # Ease-in-out interpolation
    t = frame / N_FRAMES
    if t < 0.5:
        te = 2 * t * t
    else:
        te = 1 - (-2*t + 2)**2 / 2
    
    z = stable_z + te * (collapse_z - stable_z)
    bases = BASE0.copy(); bases[:,2] = z
    
    ok, c, a = fk(bases)
    
    if ok:
        plat = P(c, a[0], a[1], a[2])
        n = R(a[0], a[1], a[2]) @ np.array([0,0,1])
        tilt = math.degrees(math.acos(np.clip(n[2], -1, 1)))
        minz = np.min(plat[:,2])
        status = "STABLE"
    else:
        plat = P(np.array([0.,0.,35.]), 0, 0, 0)
        tilt = 0; minz = 0
        status = "COLLAPSED!"
    
    collapsed = not ok or tilt > 60 or minz < -2 or np.any(np.abs(c) > 500) if ok else True
    
    # Draw base
    ax.scatter(bases[:,0], bases[:,1], bases[:,2], c='blue', s=80, marker='o')
    for i,j in CONN:
        ax.plot([bases[i,0],bases[j,0]],[bases[i,1],bases[j,1]],[bases[i,2],bases[j,2]],'b-',lw=1,alpha=0.3)
    
    # Draw platform
    color = 'red' if collapsed else 'green'
    lbl = 'Platform (COLLAPSED)' if collapsed else 'Platform'
    ax.scatter(plat[:,0], plat[:,1], plat[:,2], c=color, s=100, marker='^')
    for i,j in CONN:
        ax.plot([plat[i,0],plat[j,0]],[plat[i,1],plat[j,1]],[plat[i,2],plat[j,2]],color,lw=3)
    
    # Draw legs
    for i in range(6):
        ax.plot([bases[i,0],plat[i,0]],[bases[i,1],plat[i,1]],[bases[i,2],plat[i,2]],'gray',lw=2,alpha=0.7)
    
    ax.set_xlim(-20,20); ax.set_ylim(-20,20); ax.set_zlim(-5,50)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    
    title = f"Frame {frame+1}/{N_FRAMES} | {status}"
    if ok and status == "STABLE":
        title += f" | Tilt: {tilt:.1f}° | Center Z: {c[2]:.1f}"
    ax.set_title(title, fontsize=12)
    
    zt = f"Base Z: [{z[0]:.1f} {z[1]:.1f} {z[2]:.1f} {z[3]:.1f} {z[4]:.1f} {z[5]:.1f}]"
    ax.text2D(0.02, 0.02, zt, transform=ax.transAxes, fontsize=9,
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    # Convert ARGB to RGB
    w, h = fig.canvas.get_width_height()
    buf = buf.reshape((h, w, 4))
    buf_rgb = np.zeros((h, w, 3), dtype=np.uint8)
    buf_rgb[:,:,0] = buf[:,:,1]  # R
    buf_rgb[:,:,1] = buf[:,:,2]  # G
    buf_rgb[:,:,2] = buf[:,:,3]  # B
    frames.append(buf_rgb)
    
    if frame % 30 == 0:
        print(f"  Frame {frame+1}/{N_FRAMES}")

plt.close(fig)

# Save
gif_path = 'd:/final/collapse_animation.gif'
print(f"Saving GIF: {gif_path}")
imageio.mimsave(gif_path, frames, fps=FPS, loop=0)
print(f"Done! {len(frames)} frames @ {FPS}fps = {len(frames)/FPS:.1f}s")

# Try MP4
try:
    mp4_path = 'd:/final/collapse_animation.mp4'
    imageio.mimsave(mp4_path, frames, fps=FPS, codec='libx264')
    print(f"MP4 saved: {mp4_path}")
except Exception as e:
    print(f"MP4 save failed: {e}")
