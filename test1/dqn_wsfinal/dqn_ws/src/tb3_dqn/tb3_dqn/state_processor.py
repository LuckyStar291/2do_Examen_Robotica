import numpy as np

class StateProcessor:
    def __init__(self, num_sectors=12, lidar_max_range=3.5):
        self.num_sectors = num_sectors         
        self.lidar_max_range = lidar_max_range  

    def process_scan(self, scan_msg, distance_to_goal, angle_to_goal):
        """
        Transforma los datos crudos del láser en algo que la IA pueda entender.
        Entrada: Datos del láser (360 puntos), distancia al objetivo, ángulo al objetivo.
        Salida: Una lista de 14 números (Estado).
        """
        
        ranges = np.array(scan_msg.ranges)
        
        ranges = np.nan_to_num(ranges, posinf=self.lidar_max_range)
        

        ranges = np.clip(ranges, 0, self.lidar_max_range)

        state = []
        chunk_size = int(len(ranges) / self.num_sectors)
        
        for i in range(self.num_sectors):
            
            sector_chunk = ranges[i*chunk_size : (i+1)*chunk_size]
            
            
            min_dist = np.min(sector_chunk)
            
            state.append(min_dist / self.lidar_max_range)

        norm_dist = min(distance_to_goal, 5.0) / 5.0
        
        norm_angle = angle_to_goal / np.pi
        
        state.append(norm_dist)
        state.append(norm_angle)

        return np.array(state)