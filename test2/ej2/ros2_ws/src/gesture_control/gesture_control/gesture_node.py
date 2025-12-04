#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
from typing import Optional

import mediapipe as mp

GESTURE_TOPIC = "/gesture_command"
IMAGE_TOPIC = "/kinect/image_raw"


class GestureControlNode(Node):
    def __init__(self):
        super().__init__("gesture_control_node")

        self.bridge = CvBridge()

        # MediaPipe (rosbag)
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose_detector = self.mp_pose.Pose(
            static_image_mode=True,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.4,
            min_tracking_confidence=0.4,
        )

        # Suscriptor a rosbag
        self.image_sub = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10,
        )

        # Publicador de /gesture_command 
        self.gesture_pub = self.create_publisher(
            String,
            GESTURE_TOPIC,
            10,
        )

        # Filtro 
        self.last_raw_gesture: Optional[str] = None
        self.raw_count: int = 0
        self.stable_gesture: Optional[str] = None  # ojitoooo

        self.get_logger().info(
            f"GestureControlNode iniciado. Subscribiéndose a {IMAGE_TOPIC} y publicando en {GESTURE_TOPIC}"
        )

    # ==========================
    #   CALLBACK PRINCIPAL
    # ==========================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        # Preprocesar imagen
        frame_small = cv2.resize(frame, (640, 360))
        frame_small = cv2.convertScaleAbs(frame_small, alpha=1.5, beta=20)

        h, w, _ = frame_small.shape

        rgb = cv2.cvtColor(frame_small, cv2.COLOR_BGR2RGB)
        results = self.pose_detector.process(rgb)

        raw_gesture: Optional[str] = None

        if results.pose_landmarks:
            # Dibujar el esqueleto
            self.mp_drawing.draw_landmarks(
                frame_small,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
            )

            # Clasificar el gesto según las posiciones
            raw_gesture = self.classify_gesture(results, h, w)

        gesture = self.update_gesture_filter(raw_gesture)


        if gesture is not None:
            # Publicar /gesture_command
            cmd_msg = String()
            cmd_msg.data = gesture
            self.gesture_pub.publish(cmd_msg)


            texto = f"Gesto estable: {gesture}"
            color = (0, 255, 0)
        else:
            texto = "Sin gesto detectado"
            color = (0, 0, 255)

        cv2.putText(
            frame_small,
            texto,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
        )

        cv2.imshow("Gesture Control", frame_small)
        cv2.waitKey(1)


    def update_gesture_filter(self, new_gesture: Optional[str]) -> Optional[str]:
        """
        Solo acepta un gesto si aparece igual varios frames seguidos.
        Esto elimina el "temblor" en /gesture_command.
        """
        if new_gesture == self.last_raw_gesture:
            self.raw_count += 1
        else:
            self.last_raw_gesture = new_gesture
            self.raw_count = 1

        # N frames para estabilizar
        N = 2

        if new_gesture is not None and self.raw_count >= N:
            self.stable_gesture = new_gesture
        elif new_gesture is None and self.raw_count >= N:
            # Si llevamos varios frames sin gesto, limpiamos
            self.stable_gesture = None

        return self.stable_gesture

    # ==========================
    #   CLASIFICAR GESTO
    # ==========================
    def classify_gesture(self, results, h: int, w: int) -> Optional[str]:

        lm = results.pose_landmarks.landmark

        try:
            lw = lm[self.mp_pose.PoseLandmark.LEFT_WRIST.value]
            rw = lm[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
            ls = lm[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
            rs = lm[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        except IndexError:
            return None

        # Pasar a píxeles
        lwx, lwy = int(lw.x * w), int(lw.y * h)
        rwx, rwy = int(rw.x * w), int(rw.y * h)
        lsx, lsy = int(ls.x * w), int(ls.y * h)
        rsx, rsy = int(rs.x * w), int(rs.y * h)

        # Centro del torso (promedio de hombros)
        cx_sh = (lsx + rsx) // 2
        cy_sh = (lsy + rsy) // 2

        # Centro de las manos
        mean_wx = (lwx + rwx) // 2

        # Distancia horizontal entre manos
        sep_wrists = abs(lwx - rwx)

        # ----- Umbrales (ajustables) -----
        up_thresh_strong = int(0.12 * h)
        up_thresh_soft   = int(0.06 * h)     
        down_thresh      = int(0.12 * h)
        level_thresh     = int(0.18 * h)
        sep_stop_thresh  = int(0.35 * w)
        center_side_thresh = int(0.06 * w)

        # Diferencias verticales
        diff_l_up = lsy - lwy  
        diff_r_up = rsy - rwy
        diff_l_down = lwy - lsy
        diff_r_down = rwy - rsy

        # ¿Están arriba?
        wrists_above_strong = diff_l_up > up_thresh_strong and diff_r_up > up_thresh_strong
        wrists_above_soft   = diff_l_up > up_thresh_soft   and diff_r_up > up_thresh_soft

        # ¿Están claramente abajo?
        wrists_below = diff_l_down > down_thresh and diff_r_down > down_thresh

        # ¿Brazos a la altura del torso?
        left_level  = abs(lwy - cy_sh) < level_thresh
        right_level = abs(rwy - cy_sh) < level_thresh
        arms_level  = left_level and right_level

        # 1) FORWARD: brazos arriba
        if wrists_above_strong:
            return "forward"

        if wrists_above_soft:
            if abs(mean_wx - cx_sh) < center_side_thresh * 2 and sep_wrists < int(0.4 * w):
                return "forward"

        # 2) BACKWARD: brazos abajo
        if wrists_below:
            return "backward"

        # gestos laterales / stop 
        if arms_level:
            # STOP
            if sep_wrists > sep_stop_thresh and abs(mean_wx - cx_sh) < center_side_thresh:
                return "stop"

            # LEFT
            if mean_wx < cx_sh - center_side_thresh:
                return "left"

            # RIGHT
            if mean_wx > cx_sh + center_side_thresh:
                return "right"

        return None



    # ==========================
    #   MAPEO GESTO → TWIST
    # ==========================
    def gesture_to_twist(self, gesture: str) -> Twist:
        """
        Convierte un gesto textual a un comando Twist.
        Esto mismo harás en el ESP32 (micro-ROS), pero aquí sirve para probar.
        """
        twist = Twist()

        linear_speed = 0.15
        angular_speed = 0.8

        if gesture == "forward":
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
        elif gesture == "backward":
            twist.linear.x = -linear_speed
            twist.angular.z = 0.0
        elif gesture == "left":
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
        elif gesture == "right":
            twist.linear.x = 0.0
            twist.angular.z = -angular_speed
        elif gesture == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        return twist


def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
