import numpy as np

class StateProcessor:
    def __init__(self, num_sectors=12, lidar_max_range=3.5):
        # Configuración inicial
        self.num_sectors = num_sectors          # Dividimos la visión en 12 rebanadas de pastel
        self.lidar_max_range = lidar_max_range  # El láser ve hasta 3.5 metros máximo

    def process_scan(self, scan_msg, distance_to_goal, angle_to_goal):
        """
        Transforma los datos crudos del láser en algo que la IA pueda entender.
        Entrada: Datos del láser (360 puntos), distancia al objetivo, ángulo al objetivo.
        Salida: Una lista de 14 números (Estado).
        """
        
        # 1. LIMPIEZA DE DATOS
        # Convertimos la lista de distancias a un arreglo numérico rápido (numpy)
        ranges = np.array(scan_msg.ranges)
        
        # A veces el láser devuelve "infinito" (inf) si no ve nada.
        # Lo cambiamos por la distancia máxima (3.5m)
        ranges = np.nan_to_num(ranges, posinf=self.lidar_max_range)
        
        # Nos aseguramos que no haya números negativos ni mayores al máximo
        ranges = np.clip(ranges, 0, self.lidar_max_range)

        # 2. SIMPLIFICACIÓN (Downsampling)
        # Convertimos 360 puntos en 12 sectores.
        # 360 / 12 = 30 grados por sector.
        state = []
        chunk_size = int(len(ranges) / self.num_sectors)
        
        for i in range(self.num_sectors):
            # Tomamos un trozo de 30 grados
            sector_chunk = ranges[i*chunk_size : (i+1)*chunk_size]
            
            # Nos quedamos con la distancia MÍNIMA de ese sector.
            # ¿Por qué la mínima? Porque si hay un obstáculo cerca en ese sector,
            # queremos saberlo para no chocar (somos conservadores).
            min_dist = np.min(sector_chunk)
            
            # Normalizamos entre 0 y 1 (Ayuda a la red neuronal a aprender más rápido)
            state.append(min_dist / self.lidar_max_range)

        # 3. AGREGAR EL OBJETIVO (Goal)
        # La IA necesita saber dónde está la meta, no solo dónde están las paredes.
        
        # Normalizamos la distancia (asumimos que 5 metros es "muy lejos")
        norm_dist = min(distance_to_goal, 5.0) / 5.0
        
        # Normalizamos el ángulo (entre -1 y 1)
        norm_angle = angle_to_goal / np.pi
        
        state.append(norm_dist)
        state.append(norm_angle)

        # Devolvemos el estado final: 12 datos de láser + 1 distancia + 1 ángulo = 14 datos
        return np.array(state)