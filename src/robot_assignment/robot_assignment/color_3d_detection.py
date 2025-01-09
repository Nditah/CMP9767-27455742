import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import cv2
import image_geometry
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

class Color3DDetection(Node):
    def __init__(self):
        super().__init__('Color3DDetection')
        self.bridge = CvBridge()
        self.real_robot = self.declare_parameter('real_robot', False).value
        self.min_area_size = self.declare_parameter('min_area_size', 100).value
        self.global_frame = self.declare_parameter('global_frame', 'odom').value
        self.visualisation = self.declare_parameter('visualisation', True).value

        self.ccamera_model = None
        self.dcamera_model = None
        self.image_depth_ros = None
        self.color2depth_aspect = None

        ccamera_info_topic = '/limo/depth_camera_link/camera_info'
        dcamera_info_topic = '/limo/depth_camera_link/depth/camera_info'
        cimage_topic = '/limo/depth_camera_link/image_raw'
        dimage_topic = '/limo/depth_camera_link/depth/image_raw'
        self.camera_frame = 'depth_link'

        if self.real_robot:
            ccamera_info_topic = '/camera/color/camera_info'
            dcamera_info_topic = '/camera/depth/camera_info'
            cimage_topic = '/camera/color/image_raw'
            dimage_topic = '/camera/depth/image_raw'
            self.camera_frame = 'camera_color_optical_frame'

        self.ccamera_info_sub = self.create_subscription(CameraInfo, ccamera_info_topic,
                                                         self.ccamera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.dcamera_info_sub = self.create_subscription(CameraInfo, dcamera_info_topic,
                                                         self.dcamera_info_callback, qos_profile=qos.qos_profile_sensor_data)

        self.cimage_sub = self.create_subscription(Image, cimage_topic,
                                                   self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.dimage_sub = self.create_subscription(Image, dimage_topic,
                                                   self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.object_location_pub = self.create_publisher(PoseStamped, '/object_location', qos.qos_profile_parameters)

        # tf functionality
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color2depth_calc(self):
        if self.color2depth_aspect is None and self.ccamera_model and self.dcamera_model:
            self.color2depth_aspect = (math.atan2(self.ccamera_model.width, 2 * self.ccamera_model.fx()) / self.ccamera_model.width) \
                / (math.atan2(self.dcamera_model.width, 2 * self.dcamera_model.fx()) / self.dcamera_model.width)

    def image2camera_tf(self, image_coords, image_color, image_depth):
        """
        Transforms image coordinates to camera coordinates.
        
        Args:
            image_coords (tuple): Coordinates in the image.
            image_color (numpy.ndarray): Color image.
            image_depth (numpy.ndarray): Depth image.
        
        Returns:
            Pose: 3D pose of the object in camera coordinates.
        """
        depth_coords = np.array(image_depth.shape[:2])/2 + (np.array(image_coords) - np.array(image_color.shape[:2])/2)*self.color2depth_aspect
        
        if 0 <= int(depth_coords[0]) < image_depth.shape[0] and 0 <= int(depth_coords[1]) < image_depth.shape[1]:
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]
        else:
            self.get_logger().warn('Depth coordinates out of bounds')
            return None
        
        camera_coords = np.array(self.ccamera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])))
        camera_coords /= camera_coords[2]
        camera_coords *= depth_value
        
        pose = Pose(position=Point(x=camera_coords[0], y=camera_coords[1], z=camera_coords[2]), 
                    orientation=Quaternion(w=1.0))
        
        return pose
    
    def ccamera_info_callback(self, data):
        if not self.ccamera_model:
            try:
                self.ccamera_model = image_geometry.PinholeCameraModel()
                self.ccamera_model.fromCameraInfo(data)
                self.color2depth_calc()
            except Exception as e:
                self.get_logger().error(f"Failed to process color camera info: {e}")

    def dcamera_info_callback(self, data):
        if not self.dcamera_model:
            try:
                self.dcamera_model = image_geometry.PinholeCameraModel()
                self.dcamera_model.fromCameraInfo(data)
                self.color2depth_calc()
            except Exception as e:
                self.get_logger().error(f"Failed to process depth camera info: {e}")

    def image_depth_callback(self, data):
        try:
            self.image_depth_ros = data
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def detect_color_blobs(self, color_image):
        """
        Detects color blobs in the given color image.
        
        Args:
            color_image (numpy.ndarray): Color image.
        
        Returns:
            dict: Dictionary containing contours for each color.
        """
        color_ranges = {
            'red': ((0, 0, 80), (50, 50, 255)),
            'green': ((0, 80, 0), (50, 255, 50)),
            'yellow': ((0, 80, 80), (50, 255, 255)),
            'blue': ((80, 0, 0), (255, 50, 50))
        }
        
        contours_dict = {}
        
        for color_name, (lower_bound, upper_bound) in color_ranges.items():
            mask = cv2.inRange(color_image, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_dict[color_name] = contours
        
        return contours_dict

    def process_contours(self, contours_dict):
        """
        Processes the detected contours and publishes their locations.
        
        Args:
            contours_dict (dict): Dictionary containing contours for each color.
        
        Returns:
            None
        """
        
        for color_name, contours in contours_dict.items():
            for num, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                
                if area > self.min_area_size:
                    cmoms = cv2.moments(cnt)
                    image_coords = (cmoms["m01"] / cmoms["m00"], cmoms["m10"] / cmoms["m00"])
                    camera_pose = self.image2camera_tf(image_coords, self.image_color, self.image_depth)
                    
                    if camera_pose is None:
                        continue
                    
                    try:
                        global_pose = do_transform_pose(camera_pose, 
                                                        self.tf_buffer.lookup_transform(self.global_frame, self.camera_frame, rclpy.time.Time()))
                        
                        self.object_location_pub.publish(PoseStamped(header=Header(frame_id=self.global_frame),
                                                                     pose=global_pose))        
                        
                        self.get_logger().info(f'---{color_name.upper()} Object id {num} ---')
                        self.get_logger().info(f'image coords: {image_coords}')
                        self.get_logger().info(f'camera coords: {camera_pose.position}')
                        self.get_logger().info(f'global coords: {global_pose.position}')
                        
                        if self.visualisation:
                            cv2.circle(self.image_color, (int(image_coords[1]), int(image_coords[0])), 5, 255, -1)
                    
                    except Exception as e:
                        self.get_logger().error(f"Failed to transform pose: {e}")

    def image_color_callback(self, data):
        if self.color2depth_aspect is None or self.image_depth_ros is None:
            return
        
        try:
            self.image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
            
            if self.real_robot:
                self.image_depth /= 1000.0
            
            contours_dict = self.detect_color_blobs(self.image_color)
            self.process_contours(contours_dict)
            
            if self.visualisation:
                self.image_depth *= 1.0/10.0
                self.image_color = cv2.resize(self.image_color, (0,0), fx=0.5, fy=0.5)
                self.image_depth = cv2.resize(self.image_depth, (0,0), fx=0.5, fy=0.5)
                cv2.imshow("image color", self.image_color)
                cv2.imshow("image depth", self.image_depth)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"Failed to process color image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_projection = Color3DDetection()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()