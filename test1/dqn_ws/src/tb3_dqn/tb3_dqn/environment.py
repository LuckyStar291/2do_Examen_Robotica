import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
import math
import time

class TurtleBot3Env(Node):
    def __init__(self):
        super().__init__('turtlebot3_env')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.client_reset = self.create_client(Empty, '/reset_simulation')

        self.scan_data = None
        self.position = Point()
        self.goal_x = 1.0
        self.goal_y = 0.0
        self.last_goal_dist = 0.0 # Variable clave para medir el progreso

        # --- CAMBIO IMPORTANTE: SOLO 3 ACCIONES ---
        # Quitamos los giros fuertes. Ahora solo puede ir adelante o girar suavemente MIENTRAS avanza.
        # Esto hace imposible que se quede girando en el sitio.
        self.actions = [
            (0.20, 0.0),   # 0: Adelante Rápido
            (0.15, 0.6),   # 1: Izquierda avanzando
            (0.15, -0.6)   # 2: Derecha avanzando
        ]
        self.action_size = len(self.actions)

    def scan_callback(self, msg):
        self.scan_data = msg

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position

    def get_goal_info(self):
        goal_dist = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2)
        inc_y = self.goal_y - self.position.y
        inc_x = self.goal_x - self.position.x
        angle_to_goal = math.atan2(inc_y, inc_x)
        # Simplificamos el retorno del ángulo (asumimos que el estado lo maneja)
        return goal_dist, 0 

    def step(self, action_idx):
        linear, angular = self.actions[action_idx]
        
        # 1. Mover
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_cmd_vel.publish(cmd)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # 2. Calcular nueva distancia
        curr_dist = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2)
        done = False
        
        # --- RECOMPENSA BASADA EN EL HAMBRE DE MOVIMIENTO ---
        # Calculamos la diferencia: ¿Estoy más cerca que hace 0.1 segundos?
        progress = self.last_goal_dist - curr_dist
        
        if progress > 0:
            # SI AVANZÓ: Premio muy grande (multiplicado por 100)
            reward = progress * 100.0 
        else:
            # SI SE QUEDÓ QUIETO O RETROCEDIÓ: Castigo
            reward = -5.0
            
        # Actualizamos la memoria para el siguiente paso
        self.last_goal_dist = curr_dist

        # Choque
        if self.scan_data and np.min(self.scan_data.ranges) < 0.22:
            reward = -100
            done = True
            
        # Meta
        if curr_dist < 0.50:
            reward = 200
            done = True
            print("🏁 ¡DIANA! LLEGÓ MOVIÉNDOSE")
            
        return self.scan_data, curr_dist, 0, reward, done

    def reset_env(self):
        # Frenar y reiniciar
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)
        
        req = Empty.Request()
        while not self.client_reset.wait_for_service(timeout_sec=1.0):
            pass
        self.client_reset.call_async(req)
        
        time.sleep(0.5)
        
        # Generar objetivo fácil para empezar (al frente o lado cercano)
        quadrant = np.random.choice([0, 1])
        if quadrant == 0: 
            self.goal_x, self.goal_y = 1.5, 0.0
        else: 
            self.goal_x, self.goal_y = 1.5, -1.0
            
        self.scan_data = None 
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan_data is not None:
                if np.min(self.scan_data.ranges) > 0.22:
                    break
        
        # ¡IMPORTANTE! Inicializar la distancia base
        self.last_goal_dist = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2)
        
        return self.scan_data