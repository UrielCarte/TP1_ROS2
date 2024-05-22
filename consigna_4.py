import pandas as pd
import matplotlib.pyplot as plt

def plot_circular_path(file_path, label, ax):
    # Cargar los datos del archivo de registro
    df = pd.read_csv(file_path, sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

    # Filtrar las filas donde todos los datos son cero
    df = df.loc[~(df == 0).all(axis=1)]

    # Ajustar los timestamps para que comiencen en cero
    df['timestamp'] -= df['timestamp'].min()

    # Convertir columnas a numpy arrays para evitar problemas de indexación multidimensional
    x = df['x'].to_numpy()
    y = df['y'].to_numpy()

    # Graficar el camino seguido por el robot
    ax.plot(x, y, marker='o', label=label)
    # Agregar una flecha indicando el sentido de avance
    ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color='red')

# Configuración de la figura
fig, axs = plt.subplots(2, 2, figsize=(12, 12))

# Graficar cada combinación
plot_circular_path('log_circular_1.txt', 'Linear +, Angular +', axs[0, 0])
axs[0, 0].set_title('Linear +, Angular +')

plot_circular_path('log_circular_2.txt', 'Linear +, Angular -', axs[0, 1])
axs[0, 1].set_title('Linear +, Angular -')

plot_circular_path('log_circular_3.txt', 'Linear -, Angular +', axs[1, 0])
axs[1, 0].set_title('Linear -, Angular +')

plot_circular_path('log_circular_4.txt', 'Linear -, Angular -', axs[1, 1])
axs[1, 1].set_title('Linear -, Angular -')

# Ajustar la relación de aspecto y etiquetas
for ax in axs.flat:
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.axis('equal')
    ax.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()
