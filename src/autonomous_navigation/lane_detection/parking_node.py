import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adaptez selon votre configuration Gazebo
            self.image_callback,
            10)
        self.blue_lines_publisher = self.create_publisher(
            Float32MultiArray, 
            '/detected_blue_lines', 
            10)
        self.white_lines_publisher = self.create_publisher(
            Float32MultiArray, 
            '/detected_white_lines', 
            10)
        self.debug_image_publisher = self.create_publisher(
            Image, 
            '/debug_image', 
            10)
        self.bridge = CvBridge()
        
    def detect_lines_by_color(self, image, lower_bound, upper_bound):
        # Conversion en HSV pour une meilleure segmentation par couleur
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Créer un masque pour la couleur spécifiée
        color_mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Appliquer des opérations morphologiques pour réduire le bruit
        kernel = np.ones((5, 5), np.uint8)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
        
        # Détecter les contours
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrer les contours par taille
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Ajustez selon vos besoins
                valid_contours.append(contour)
        
        # Extraire les lignes à partir des contours valides
        lines = []
        for contour in valid_contours:
            [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            # Stocker les paramètres de la ligne (point et vecteur directeur)
            lines.append((float(x), float(y), float(vx), float(vy)))
        
        return lines, color_mask
        
    def image_callback(self, msg):
        # Convertir l'image ROS en format OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        debug_image = cv_image.copy()
        
        # Définir les plages de couleurs (à ajuster selon les conditions)
        lower_blue = np.array([100, 50, 50])  # HSV pour bleu
        upper_blue = np.array([130, 255, 255])
        
        lower_white = np.array([0, 0, 200])  # HSV pour blanc
        upper_white = np.array([180, 30, 255])
        
        # Détecter les lignes bleues et blanches
        blue_lines, blue_mask = self.detect_lines_by_color(cv_image, lower_blue, upper_blue)
        white_lines, white_mask = self.detect_lines_by_color(cv_image, lower_white, upper_white)
        
        # Visualiser les résultats sur l'image de débogage
        cv2.drawContours(debug_image, cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, 
                        cv2.CHAIN_APPROX_SIMPLE)[0], -1, (255, 0, 0), 2)
        cv2.drawContours(debug_image, cv2.findContours(white_mask, cv2.RETR_EXTERNAL, 
                        cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 0), 2)
        
        # Dessiner les lignes détectées
        for x, y, vx, vy in blue_lines:
            cv2.line(debug_image, 
                    (int(x - vx * 100), int(y - vy * 100)), 
                    (int(x + vx * 100), int(y + vy * 100)), 
                    (255, 0, 0), 2)
        
        for x, y, vx, vy in white_lines:
            cv2.line(debug_image, 
                    (int(x - vx * 100), int(y - vy * 100)), 
                    (int(x + vx * 100), int(y + vy * 100)), 
                    (255, 255, 255), 2)
        
        # Publier les lignes détectées
        blue_msg = Float32MultiArray()
        blue_msg.data = [val for line in blue_lines for val in line]
        self.blue_lines_publisher.publish(blue_msg)
        
        white_msg = Float32MultiArray()
        white_msg.data = [val for line in white_lines for val in line]
        self.white_lines_publisher.publish(white_msg)
        
        # Publier l'image de débogage
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        self.debug_image_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetectorNode()
    rclpy.spin(line_detector)
    line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()