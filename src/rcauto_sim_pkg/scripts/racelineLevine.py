import os
import yaml
import numpy as np
import imageio.v2 as imageio
import matplotlib.pyplot as plt
from skimage import measure, morphology
from shapely.geometry import LineString
from scipy.interpolate import splprep, splev

# -------------------------
# Parámetros de usuario
# -------------------------
# Ajusta este valor al ancho total de tu pista (en metros)
track_width = 2.5

# ---------------------------------------------------------------------
# 1) Cargar y binarizar el mapa
# ---------------------------------------------------------------------
pkg_path  = os.path.expanduser('~/tfm_ws/src/rcauto_sim_pkg')
map_yaml  = os.path.join(pkg_path, 'mapas', 'levine', 'levine1.yaml')
with open(map_yaml, 'r') as f:
    m = yaml.safe_load(f)

resolution = m['resolution']
origin     = m['origin'][:2]
pgm_path   = os.path.join(os.path.dirname(map_yaml), m['image'])
free_th    = m.get('free_thresh', 0.25)
px_th      = 225

img      = imageio.imread(pgm_path)
bin_map  = (img >= px_th).astype(np.uint8)

#----------------------------------------------------------------------
# 2) Extraer el contorno exterior de la pista
#----------------------------------------------------------------------
# Limpiar y seleccionar la región más grande
mask = morphology.remove_small_holes(bin_map.astype(bool), area_threshold=1e4)
mask = morphology.remove_small_objects(mask, min_size=1e4)
lbl  = measure.label(mask)
props = measure.regionprops(lbl)
track = max(props, key=lambda p: p.area)
mask  = (lbl == track.label).astype(np.uint8)

contours = measure.find_contours(mask, level=0.5)
contours = sorted(contours, key=lambda c: len(c), reverse=True)
ext_cnt, int_cnt = contours[0], contours[1]

H, W     = mask.shape
# Convertir a coordenadas del mundo
def to_world(cnt):
    return [(
        c * resolution + origin[0],
        (H - r) * resolution + origin[1]
    ) for r, c in cnt]

ext_xy = to_world(ext_cnt)
int_xy = to_world(int_cnt)

ext_line = LineString(ext_xy)
int_line = LineString(int_xy)

N = 1000  # número de puntos deseados
us = np.linspace(0.0, 1.0, N)

# Listas de shapely Point
int_pts = [int_line.interpolate(u * int_line.length) for u in us]
ext_pts = [ext_line.interpolate(u * ext_line.length) for u in us]

int_xy_pts = [(p.x, p.y) for p in int_pts]
ext_xy_pts = [(p.x, p.y) for p in ext_pts]

# 2) Encuentra el desplazamiento (shift) que minimiza la distancia
#    entre int_xy_pts[0] y cada ext_xy_pts[k].
d0 = int_xy_pts[0]
dists = [ (abs(ex - d0[0]) + abs(ey - d0[1]), k)
          for k, (ex,ey) in enumerate(ext_xy_pts) ]
_, best_k = min(dists, key=lambda t: t[0])

# 3) Rota ext_pts y ext_xy_pts para alinearlos
ext_pts     = ext_pts[best_k:] + ext_pts[:best_k]
ext_xy_pts  = ext_xy_pts[best_k:] + ext_xy_pts[:best_k]

# 4) Asegúrate de que ambos recorridos van en el mismo sentido
#    (por si uno está dado en sentido antihorario y el otro horario)
#    Una forma rápida: compara distancias primera mitad-final.
if np.hypot(*(np.array(ext_xy_pts[1]) - np.array(ext_xy_pts[0])))\
   < np.hypot(*(np.array(ext_xy_pts[-1]) - np.array(ext_xy_pts[0]))):
    ext_pts    .reverse()
    ext_xy_pts .reverse()

# después de int_pts y ext_pts
mid_x = [(pi.x + pe.x)/2.0 for pi, pe in zip(int_pts, ext_pts)]
mid_y = [(pi.y + pe.y)/2.0 for pi, pe in zip(int_pts, ext_pts)]

xs_int = [p.x for p in int_pts]; ys_int = [p.y for p in int_pts]
xs_ext = [p.x for p in ext_pts]; ys_ext = [p.y for p in ext_pts]

# Preparo los datos para el spline
xs_mid = np.array(mid_x)
ys_mid = np.array(mid_y)

# Parámetros del spline: s controla el nivel de suavizado
tck, u = splprep([xs_mid, ys_mid], s=track_width**2, per= True)

# Muestreo uniforme
Npts = 10000
u_fine = np.linspace(0, 1, Npts)
x_smooth, y_smooth = splev(u_fine, tck)

# --- 1) calcula derivadas y curvatura ---
dx = np.gradient(x_smooth)
dy = np.gradient(y_smooth)
ddx = np.gradient(dx)
ddy = np.gradient(dy)
# curvatura κ = |x' y'' - y' x''| / (x'² + y'²)^(3/2)
kappa = np.abs(dx*ddy - dy*ddx) / ( (dx**2 + dy**2)**1.5 + 1e-8 )

# --- 2) define densidad rho(u) ---
rho0  = 1.0           # densidad mínima (puedes escalar en m⁻¹)
alpha = 10.0          # cuánto aumentas en curvas
rho = rho0 + alpha * (kappa / kappa.max())

# --- 3) integra rho · ds para obtener pesos acumulados w ---
# calcula ds[i] = distancia entre puntos consecutivos
ds = np.hypot(np.diff(x_smooth), np.diff(y_smooth))
# usamos rho[:-1] para que rho[i] multiplique ds[i]
w = rho[:-1] * ds
cumw = np.concatenate(([0.0], np.cumsum(w)))
total_w = cumw[-1]

# --- 4) muestreo uniforme en [0, total_w] ---
N_adapt = 500   # número final de puntos en la racing line
usamp = np.linspace(0, total_w, N_adapt)

# invierte integración para encontrar índice + fracción
x_adapt = []
y_adapt = []
for target in usamp:
    idx = np.searchsorted(cumw, target) - 1
    idx = np.clip(idx, 0, len(ds)-1)
    # fracción entre cumw[idx] y cumw[idx+1]
    t = (target - cumw[idx]) / (cumw[idx+1] - cumw[idx] + 1e-8)
    # interpola en el spline original
    x_val = x_smooth[idx] + t*(x_smooth[idx+1] - x_smooth[idx])
    y_val = y_smooth[idx] + t*(y_smooth[idx+1] - y_smooth[idx])
    x_adapt.append(x_val)
    y_adapt.append(y_val)

# ahora x_adapt, y_adapt tienen más puntos donde kappa es grande
# puedes reemplazar x_smooth,y_smooth por estos para calcular thetas y guardar
x_smooth = np.array(x_adapt)
y_smooth = np.array(y_adapt)

# ---------------------------------------------------------------------
# 6) Calcular las orientaciones (yaw) de la trayectoria
# ---------------------------------------------------------------------
thetas = np.arctan2(np.gradient(y_smooth), np.gradient(x_smooth))

# ---------------------------------------------------------------------
# 7) Guardar en YAML
# ---------------------------------------------------------------------
import yaml
out = {'racing_line': [
    {'x': float(x), 'y': float(y), 'theta': float(th)}
    for x, y, th in zip(x_smooth, y_smooth, thetas)
]}

out_path = os.path.join(pkg_path, 'config', 'line_levineF1.yaml')
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, 'w') as f:
    yaml.dump(out, f)

plt.plot(xs_int, ys_int, 'r.', label='interior resampleado')
plt.plot(xs_ext, ys_ext, 'g.', label='exterior resampleado')
plt.plot(mid_x, mid_y, 'b-', label='racing line (media)')
plt.plot(x_smooth, y_smooth, 'y-', lw=2, label='raceline suavizado')
plt.legend(); plt.show()

# Convertir a píxeles
px_s = (x_smooth - origin[0]) / resolution
py_s = H - (y_smooth - origin[1]) / resolution

plt.figure(figsize=(6,6))
plt.imshow(mask, cmap='gray', origin='lower')
plt.plot(px_s, py_s, 'y-', lw=2, label='raceline suavizado')
plt.axis('off')
plt.legend()
plt.show()