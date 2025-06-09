from sklearn.metrics import mean_squared_error
import numpy as np
import yaml
import matplotlib.pyplot as plt

# Cargar los datos reales de ambos simuladores
real_custom = pd.read_csv("/home/spech/tfm_ws/src/rcauto_sim_pkg/output/real_path.csv")
real_f1 = pd.read_csv("/home/spech/sim_ws/src/f1tenth_gym_ros/output/real_pathF1.csv")

# Cargar la trayectoria planificada desde el YAML
with open("/home/spech/tfm_ws/src/rcauto_sim_pkg/config/pruebaPath.yaml", 'r') as f:
    yaml_data = yaml.safe_load(f)
planned_path = pd.DataFrame(yaml_data['racing_line'])

# Extraer la trayectoria planificada (solo x, y)
planned_xy = planned_path[['x', 'y']].to_numpy()

# Función para calcular RMSE entre trayectorias reales y planificada (interpolada)
def compute_rmse(real_df, planned_xy):
    # Interpolar la trayectoria planificada al mismo número de puntos que la real
    real_len = len(real_df)
    interp_indices = np.linspace(0, len(planned_xy) - 1, real_len).astype(int)
    interp_planned = planned_xy[interp_indices]

    # Extraer posiciones reales
    real_xy = real_df[['x', 'y']].to_numpy()

    # Calcular RMSE
    rmse = np.sqrt(mean_squared_error(real_xy, interp_planned))
    return rmse, interp_planned, real_xy

rmse_custom, interp_planned_custom, real_xy_custom = compute_rmse(real_custom, planned_xy)
rmse_f1, interp_planned_f1, real_xy_f1 = compute_rmse(real_f1, planned_xy)

# Crear la gráfica
plt.figure(figsize=(10, 6))
plt.plot(interp_planned_custom[:, 0], interp_planned_custom[:, 1], 'k--', label='Planificada')
plt.plot(real_xy_custom[:, 0], real_xy_custom[:, 1], 'b-', label=f'Simulador Propio (RMSE={rmse_custom:.2f} m)')
plt.plot(real_xy_f1[:, 0], real_xy_f1[:, 1], 'g-', label=f'Simulador F1TENTH (RMSE={rmse_f1:.2f} m)')
plt.title("Comparación de Trayectorias: Real vs. Planificada")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()