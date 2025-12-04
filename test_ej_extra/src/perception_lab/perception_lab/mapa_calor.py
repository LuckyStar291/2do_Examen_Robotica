import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge


class DepthHeatmapNode(Node):
    def __init__(self):
        super().__init__("depth_heatmap_node")

        self.bridge = CvBridge()

        # Suscripción a la imagen de profundidad
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.depth_callback,
            10
        )

        # Publicador a /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Nodo de mapa de calor iniciado")

    def depth_callback(self, msg):
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        h, w = depth_img.shape

        # Dividir en 3 zonas verticales
        left = depth_img[:, : w//3]
        center = depth_img[:, w//3 : 2*w//3]
        right = depth_img[:, 2*w//3 :]

        # Reemplazar NaN e infinitos con valores grandes
        left = np.nan_to_num(left, nan=5.0, posinf=5.0)
        center = np.nan_to_num(center, nan=5.0, posinf=5.0)
        right = np.nan_to_num(right, nan=5.0, posinf=5.0)

        # Calcular promedios
        d_left = np.mean(left)
        d_center = np.mean(center)
        d_right = np.mean(right)

        self.get_logger().info(
            f"IZQ: {d_left:.2f} m | CENTRO: {d_center:.2f} m | DER: {d_right:.2f} m"
        )

        twist = Twist()

        # Reglas del examen:
        # Zona frontal < 1m → reducir velocidad
        # Zona frontal < 0.5m → detener
        # Laterales < 0.7m → corregir giro

        # --- AVANCE ---
        if d_center < 0.5:
            twist.linear.x = 0.0
        elif d_center < 1.0:
            twist.linear.x = 0.1  # 50% velocidad
        else:
            twist.linear.x = 0.2  # velocidad normal

        # --- GIRO AUTOMÁTICO según obstáculos laterales ---
        if d_left < 0.7:
            twist.angular.z = -0.6  # girar derecha
        elif d_right < 0.7:
            twist.angular.z = 0.6  # girar izquierda
        else:
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DepthHeatmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
