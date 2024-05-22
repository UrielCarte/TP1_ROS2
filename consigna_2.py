import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo de registro
df = pd.read_csv('log.txt', sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

# Filtrar las filas donde todos los datos son cero
df = df.loc[~(df == 0).all(axis=1)]

# Ajustar los timestamps para que comiencen en cero
df['timestamp'] -= df['timestamp'].min()

# Convertir columnas a numpy arrays para evitar el problema de indexación multidimensional
timestamps = df['timestamp'].to_numpy()
x = df['x'].to_numpy()
y = df['y'].to_numpy()
linear_velocity = df['linear_velocity'].to_numpy()
angular_velocity = df['angular_velocity'].to_numpy()

# Configuración de la figura
plt.figure(figsize=(18, 6))

# i) Gráfico del camino seguido por el robot
plt.subplot(1, 3, 1)
plt.plot(x, y, marker='o')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Camino seguido por el robot')
plt.axis('equal')  # Relación de aspecto 1:1

# ii) Gráfico de la trayectoria (pose respecto al tiempo)
plt.subplot(1, 3, 2)
plt.plot(timestamps, x, label='X')
plt.plot(timestamps, y, label='Y')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición')
plt.title('Trayectoria (pose respecto al tiempo)')
plt.legend()

# iii) Gráfico de la velocidad del robot respecto al tiempo
plt.subplot(1, 3, 3)
plt.plot(timestamps, linear_velocity, label='Velocidad lineal')
plt.plot(timestamps, angular_velocity, label='Velocidad angular')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad')
plt.title('Velocidad del robot respecto al tiempo')
plt.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()
