import matplotlib.pyplot as plt
import numpy as np
rewards = [
    -53.10, 200.00, 55.10, -46.48, 192.36, -119.36, -33.45, -136.85, 108.89, -43.79,
    91.85, -6.76, 128.04, -156.68, 97.65, 210.46, 66.30, -17.51, 167.90, -29.39,
    -94.62, 440.66, 136.62, 114.54, 73.63, 71.01, -61.28, 336.52, -143.99, -157.42,
    194.35, 25.09, -173.00, 6.87, 90.13, 95.98, 185.25, 65.12, 328.17, 461.44,
    207.72, -3.53, 54.99, 330.20, 91.57, 28.14, -37.40, 22.12, 443.48, 83.85,
    -128.26, 54.26, -160.52, 170.95, 11.65, 130.45, 570.77, 17.30, 143.89, 85.45,
    55.29, 304.58, 10.03, -42.32, 304.91, 287.05, 430.71, -66.05, 59.16, -73.51,
    -3.89, -125.05, 200.00, 396.90, 530.46, 77.00, -15.59, 236.34, 92.41, 387.18,
    618.36, 31.86, 176.04, -143.04, 7.02, 64.21, 325.92, -78.86, -72.55, -109.71,
    -146.46, -29.44, 5.15, 14.30, 61.87, -45.17, 234.84, -41.70, -39.08, 534.58,
    -19.05, 179.94, 535.86, 534.21, 290.37, -209.10, -139.05, 73.41, -54.60, -13.09,
    228.59, 109.02, 70.30, 502.33, 497.52, 121.04, 95.65, -214.73, -73.78, 594.25,
    129.96, 132.99, 138.55, 78.25, 320.47, 568.55, 201.25, 19.18, 129.88, 150.45,
    236.82, 539.59, 416.84, 24.40, 29.21, 152.04, 336.58, 468.48, 470.94, 45.40,
    -43.24, 349.12, 415.80, 53.33, 70.55, 116.29, 1.26, 496.88, 101.84, 163.76,
    250.81, -57.26, 305.74, 205.07, 312.38, 40.70, 38.94, -72.69, 278.03, 84.80,
    60.58, 38.15, -72.97, 29.65, 199.50, 8.33, -2.57, 237.21, 24.99, 50.97,
    -95.96, -86.84, 517.68, 368.32, 97.53, 255.23, 318.14, 68.87, 384.57, 425.44,
    522.12, 387.81, 75.20, 299.53, -198.49, 422.83, 363.43, 79.63, -131.31, 313.05,
    33.26, 187.69, 488.94, -22.48, -22.17, -41.57, 200.00, 516.53, 366.18, 191.31
]

episodes = np.arange(len(rewards))
window = 10
moving_avg = np.convolve(rewards, np.ones(window)/window, mode='valid')

plt.figure(figsize=(12, 6))
plt.plot(episodes, rewards, label='Recompensa por Episodio', color='lightblue', alpha=0.5)
plt.plot(episodes[window-1:], moving_avg, label=f'Tendencia (Media móvil {window} ep)', color='red', linewidth=2)
plt.title('Curva de Aprendizaje: Navegación Autónoma TurtleBot3', fontsize=16)
plt.xlabel('Episodios de Entrenamiento', fontsize=12)
plt.ylabel('Puntos Totales (Recompensa)', fontsize=12)
plt.axhline(200, color='green', linestyle='--', alpha=0.5, label='Meta Alcanzada (ref)')
plt.axhline(0, color='black', linestyle='-', linewidth=0.5)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
nombre_archivo = 'curva_aprendizaje.png'
plt.savefig(nombre_archivo, dpi=300)
print(f"✅ ¡Gráfica generada con éxito! Guardada como: {nombre_archivo}")
plt.show()