import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo de registro
df = pd.read_csv('log_circular.txt', sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

# Filtrar las filas donde todos los datos son cero
df = df.loc[~(df == 0).all(axis=1)]

# Ajustar los timestamps para que comiencen en cero
df['timestamp'] -= df['timestamp'].min()

# Convertir columnas a numpy arrays para evitar problemas de indexación multidimensional
timestamps = df['timestamp'].to_numpy()
x = df['x'].to_numpy()
y = df['y'].to_numpy()

# Configuración de la figura
plt.figure(figsize=(12, 6))

# Gráfico del camino seguido por el robot
plt.subplot(1, 2, 1)
plt.plot(x, y, marker='o')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Camino seguido por el robot')
plt.axis('equal')  # Relación de aspecto 1:1

# Gráfico de la trayectoria (pose respecto al tiempo)
plt.subplot(1, 2, 2)
plt.plot(timestamps, x, label='X')
plt.plot(timestamps, y, label='Y')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición')
plt.title('Trayectoria (pose respecto al tiempo)')
plt.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()
