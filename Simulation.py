import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuration du port série (remplacez 'COM3' par votre port)
ser = serial.Serial('COM3', 115200, timeout=1)

# Initialisation des données
time_window = 100  # Nombre de points à afficher en temps réel
time_step = 0.05  # Intervalle de temps entre deux mises à jour en secondes (correspondant à delay dans Arduino)
data_buffer = {
    'sensor1_y': [], 'sensor1_z': [],
    'sensor2_y': [], 'sensor2_z': [],
    'sensor1_vy': [0], 'sensor1_vz': [0],  # Vitesses initiales (Y, Z) pour capteur 1
    'sensor2_vy': [0], 'sensor2_vz': [0],  # Vitesses initiales (Y, Z) pour capteur 2
    'sensor1_py': [0], 'sensor1_pz': [0],  # Positions initiales (Y, Z) pour capteur 1
    'sensor2_py': [0], 'sensor2_pz': [0]   # Positions initiales (Y, Z) pour capteur 2
}

# Configuration du graphique
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-10, 10)  # Ajustez les limites selon vos données
ax.set_ylim(-10, 10)  # Ajustez les limites selon vos données
ax.set_title("Simulation en temps réel des mouvements des capteurs")
ax.set_xlabel("Position Y (m)")
ax.set_ylabel("Position Z (m)")

# Objets pour l'animation
sensor1_point, = ax.plot([], [], 'ro', label="Capteur 1")
sensor2_point, = ax.plot([], [], 'bo', label="Capteur 2")
sensor1_path, = ax.plot([], [], 'r-', alpha=0.5, label="Trajectoire Capteur 1")
sensor2_path, = ax.plot([], [], 'b-', alpha=0.5, label="Trajectoire Capteur 2")
ax.legend()

# Fonction d'initialisation
def init():
    sensor1_point.set_data([], [])
    sensor2_point.set_data([], [])
    sensor1_path.set_data([], [])
    sensor2_path.set_data([], [])
    return sensor1_point, sensor2_point, sensor1_path, sensor2_path

# Fonction de mise à jour
def update(frame):
    global data_buffer

    # Lire une ligne du port série
    line = ser.readline().decode('utf-8').strip()
    if line:
        print(f"Reçu : {line}")  # Débogage : afficher les données brutes
        parts = line.split(",")
        if len(parts) == 11:
            sensor = parts[0]
            try:
                temp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = map(float, parts[1:])

                # Mise à jour des vitesses et positions par intégration
                if sensor == "C1":
                    # Calcul des vitesses
                    vy = data_buffer['sensor1_vy'][-1] + accel_y * time_step
                    vz = data_buffer['sensor1_vz'][-1] + accel_z * time_step
                    data_buffer['sensor1_vy'].append(vy)
                    data_buffer['sensor1_vz'].append(vz)

                    # Calcul des positions
                    py = data_buffer['sensor1_py'][-1] + vy * time_step
                    pz = data_buffer['sensor1_pz'][-1] + vz * time_step
                    data_buffer['sensor1_py'].append(py)
                    data_buffer['sensor1_pz'].append(pz)

                elif sensor == "C2":
                    # Calcul des vitesses
                    vy = data_buffer['sensor2_vy'][-1] + accel_y * time_step
                    vz = data_buffer['sensor2_vz'][-1] + accel_z * time_step
                    data_buffer['sensor2_vy'].append(vy)
                    data_buffer['sensor2_vz'].append(vz)

                    # Calcul des positions
                    py = data_buffer['sensor2_py'][-1] + vy * time_step
                    pz = data_buffer['sensor2_pz'][-1] + vz * time_step
                    data_buffer['sensor2_py'].append(py)
                    data_buffer['sensor2_pz'].append(pz)

                # Conserver uniquement les derniers points
                for key in data_buffer:
                    if len(data_buffer[key]) > time_window:
                        data_buffer[key] = data_buffer[key][-time_window:]

            except ValueError:
                print("Erreur : Données mal formatées, ignorées.")

    # Mettre à jour les graphiques
    if data_buffer['sensor1_py'] and data_buffer['sensor1_pz']:
        sensor1_point.set_data(data_buffer['sensor1_py'][-1:], data_buffer['sensor1_pz'][-1:])
        sensor1_path.set_data(data_buffer['sensor1_py'], data_buffer['sensor1_pz'])

    if data_buffer['sensor2_py'] and data_buffer['sensor2_pz']:
        sensor2_point.set_data(data_buffer['sensor2_py'][-1:], data_buffer['sensor2_pz'][-1:])
        sensor2_path.set_data(data_buffer['sensor2_py'], data_buffer['sensor2_pz'])

    # Calcul et affichage de la position relative
    if data_buffer['sensor1_py'] and data_buffer['sensor2_py']:
        relative_py = data_buffer['sensor1_py'][-1] - data_buffer['sensor2_py'][-1]
        relative_pz = data_buffer['sensor1_pz'][-1] - data_buffer['sensor2_pz'][-1]
        print(f"Position relative (Y, Z) : ({relative_py:.2f}, {relative_pz:.2f})")

    return sensor1_point, sensor2_point, sensor1_path, sensor2_path

# Animation
ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=False)

plt.show()
