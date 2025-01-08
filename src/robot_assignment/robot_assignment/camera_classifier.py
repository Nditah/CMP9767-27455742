import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sqlite3
import os
from datetime import datetime

DB_PATH = os.path.join(os.path.expanduser('~'), 'toy_detections.db')

class CameraClassifierNode(Node):
    def __init__(self):
        super().__init__('camera_classifier_node')

        # Create/ connect to SQLite database
        self.connection = sqlite3.connect(DB_PATH, check_same_thread=False)
        self.cursor = self.connection.cursor()
        self._create_table_if_not_exists()

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscription (replace '/image_raw' with your camera topic)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("CameraClassifierNode is up and running.")

    def _create_table_if_not_exists(self):
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS objects (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                color TEXT,
                shape TEXT,
                pos_x REAL,
                pos_y REAL,
                pos_z REAL,
                count INTEGER DEFAULT 1,
                last_seen TIMESTAMP
            )
            """
        )
        self.connection.commit()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # >>> Example: detect color, shape
        color_label = self.detect_color(cv_image)
        shape_label = self.detect_shape(cv_image)

        # >>> Example: get approximate position in 3D
        pos_x, pos_y, pos_z = self.estimate_position(cv_image)

        # Check if there's already an object of this color+shape near this position
        existing_id = self.find_existing_object(color_label, shape_label, pos_x, pos_y, pos_z)
        now_str = datetime.now().isoformat()

        if existing_id:
            # Update that row
            self.cursor.execute("""
                UPDATE objects
                SET count = count + 1,
                    last_seen = ?
                WHERE id = ?
            """, (now_str, existing_id))
            self.connection.commit()
            self.get_logger().info(f"Updated existing object (ID {existing_id}) - color={color_label}, shape={shape_label}, pos=({pos_x:.2f},{pos_y:.2f},{pos_z:.2f})")
        else:
            # Insert new row
            self.cursor.execute("""
                INSERT INTO objects (color, shape, pos_x, pos_y, pos_z, last_seen)
                VALUES (?, ?, ?, ?, ?, ?)
            """, (color_label, shape_label, pos_x, pos_y, pos_z, now_str))
            self.connection.commit()
            new_id = self.cursor.lastrowid
            self.get_logger().info(f"Inserted new object (ID {new_id}) - color={color_label}, shape={shape_label}, pos=({pos_x:.2f},{pos_y:.2f},{pos_z:.2f})")

    def detect_color(self, image):
        """ 
        Simple color detection by average BGR. 
        You may need HSV-based approach for better accuracy. 
        """
        mean_bgr = np.array(cv2.mean(image)[:3], dtype=np.float32)
        color_map = {
            'red':   np.array([0,   0, 255], dtype=np.float32),
            'green': np.array([0, 255,   0], dtype=np.float32),
            'blue':  np.array([255,  0,   0], dtype=np.float32),
        }
        distances = {c: np.linalg.norm(mean_bgr - ref) for c, ref in color_map.items()}
        return min(distances, key=distances.get)

    def detect_shape(self, image):
        """
        Rudimentary shape detection by contour approximation.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            epsilon = 0.02 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)
            sides = len(approx)
            if sides == 3:
                return "triangle"
            elif sides == 4:
                return "box"
            else:
                return "cylinder"
        return "unknown"

    def estimate_position(self, image):
        """
        Demo: Return fake x,y,z just for illustration.
        In real usage, you'd get the depth or transform 
        from the camera coordinate frame to your map/world frame.
        """
        return (1.0, 2.0, 0.5)  # Hard-coded example

    def find_existing_object(self, color, shape, x, y, z, threshold=0.5):
        """
        Find an existing object in the DB that matches 
        (color, shape) and is within 'threshold' distance of (x, y, z).
        Return its ID or None.
        """
        # Retrieve all known objects of the same color & shape
        self.cursor.execute("""
            SELECT id, pos_x, pos_y, pos_z 
            FROM objects
            WHERE color = ? AND shape = ?
        """, (color, shape))
        candidates = self.cursor.fetchall()

        for row in candidates:
            obj_id, ox, oy, oz = row
            dist = np.linalg.norm([x - ox, y - oy, z - oz])
            if dist < threshold:
                return obj_id

        return None

    def destroy_node(self):
        self.connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
