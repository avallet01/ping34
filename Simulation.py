import serial
import numpy as np
import imufusion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuration du port série
ser = serial.Serial('COM3', 115200, timeout=1)

# Configuration du graphique
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])
ax.set_title("Position Relative en Temps Réel (3D)")
ax.set_xlabel("Position X (m)")
ax.set_ylabel("Position Y (m)")
ax.set_zlabel("Position Z (m)")

point, = ax.plot([], [], [], 'ro', label="Capteur mobile")
path, = ax.plot([], [], [], 'r-', alpha=0.5, label="Trajectoire")
ax.legend()

trajectory_x, trajectory_y, trajectory_z = [0], [0], [0]

# Classe pour le Filtre de Kalman Étendu
class EKF:
    def __init__(self, dt):
        self.dt = dt
        self.state = np.zeros(9)  # [x, y, z, vx, vy, vz, bx, by, bz]
        self.P = np.eye(9) * 0.1
        self.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1])  # Ajusté
        self.R = np.diag([0.2, 0.2, 0.2])  # Ajusté
        self.H = np.zeros((3, 9))
        self.H[:, :3] = np.eye(3)

    def predict(self, acceleration_corrected):
        F = np.eye(9)
        F[:3, 3:6] = np.eye(3) * self.dt

        self.state[:3] += self.state[3:6] * self.dt
        self.state[3:6] += acceleration_corrected * self.dt
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        y = z - self.state[:3]
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(9) - K @ self.H) @ self.P

ekf = EKF(dt=0.05)
fusion_filter = imufusion.Ahrs()
stabilization_threshold = 0.005

# Calibration des capteurs
accel_bias, gyro_bias = np.zeros(3), np.zeros(3)

def init():
    point.set_data([], [])
    point.set_3d_properties([])
    path.set_data([], [])
    path.set_3d_properties([])
    return point, path

def update(frame):
    global trajectory_x, trajectory_y, trajectory_z

    line = ser.readline().decode('utf-8').strip()
    if line:
        try:
            parts = line.split(",")
            if len(parts) == 11:
                _, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, _, _, _ = map(float, parts[1:])
                accelerometer = np.array([accel_x, accel_y, accel_z]) - accel_bias
                gyroscope = np.array([gyro_x, gyro_y, gyro_z]) - gyro_bias

                # Mise à jour du filtre de fusion
                fusion_filter.update_no_magnetometer(gyroscope, accelerometer, ekf.dt)
                gravity = fusion_filter.gravity
                acceleration_corrected = accelerometer - gravity

                # Stabilisation si immobile
                if np.linalg.norm(acceleration_corrected) < stabilization_threshold and np.linalg.norm(gyroscope) < stabilization_threshold:
                    ekf.state[3:6] *= 0.9  # Réduction de la vitesse
                    ekf.state[6:9] *= 0.99  # Réduction des biais

                # Prédiction et mise à jour EKF
                ekf.predict(acceleration_corrected)

                # Mise à jour des trajectoires
                simulated_position = ekf.state[:3]
                trajectory_x.append(simulated_position[0])
                trajectory_y.append(simulated_position[1])
                trajectory_z.append(simulated_position[2])

                trajectory_x = trajectory_x[-100:]
                trajectory_y = trajectory_y[-100:]
                trajectory_z = trajectory_z[-100:]

        except ValueError:
            print("Erreur : Données mal formatées")

    point.set_data(trajectory_x[-1:], trajectory_y[-1:])
    point.set_3d_properties(trajectory_z[-1:])
    path.set_data(trajectory_x, trajectory_y)
    path.set_3d_properties(trajectory_z)

    return point, path

ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=False)
plt.show()
