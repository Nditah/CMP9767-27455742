import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from robot_assignment.color_3d_detection import Color3DDetection

# Suppress the SelectableGroups warning
warnings.filterwarnings("ignore", category=DeprecationWarning, module="pkg_resources")

class TestColor3DDetection(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Color3DDetection()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    @patch('cv2.inRange')
    @patch('cv2.findContours')
    @patch('cv2.moments')
    @patch('cv2.circle')
    def test_image_color_callback(self, mock_circle, mock_moments, mock_findContours, mock_inRange):
        # Mock the methods and attributes used in the callback
        self.node.color2depth_aspect = 1.0
        self.node.image_depth_ros = MagicMock()
        self.node.bridge.imgmsg_to_cv2 = MagicMock(return_value=np.zeros((480, 640, 3), dtype=np.uint8))
        mock_inRange.return_value = np.zeros((480, 640), dtype=np.uint8)
        mock_findContours.return_value = ([np.array([[0, 0], [1, 1], [2, 2]])], None)
        mock_moments.return_value = {"m00": 1.0, "m10": 1.0, "m01": 1.0}

        # Create a mock Image message
        image_msg = Image()

        # Call the callback
        self.node.image_color_callback(image_msg)

        # Check if the methods were called
        self.assertTrue(mock_inRange.called)
        self.assertTrue(mock_findContours.called)
        self.assertTrue(mock_moments.called)
        self.assertTrue(mock_circle.called)

    # @patch('cv2.imshow')
    # @patch('cv2.waitKey')
    # def test_image_color_callback(self, mock_waitKey, mock_imshow):
    #     self.node.image_depth_ros = Image()
    #     self.node.image_depth_ros.data = np.ones((480, 640), dtype=np.float32).tobytes()
    #     data = Image()
    #     data.data = np.zeros((480, 640, 3), dtype=np.uint8).tobytes()
    #     self.node.image_color_callback(data)
    #     self.assertTrue(mock_imshow.called)
    #     self.assertTrue(mock_waitKey.called)


    def test_ccamera_info_callback(self):
        # Create a mock CameraInfo message
        camera_info_msg = CameraInfo()

        # Call the callback
        self.node.ccamera_info_callback(camera_info_msg)

        # Check if the camera model was set
        self.assertIsNotNone(self.node.ccamera_model)

    def test_dcamera_info_callback(self):
        # Create a mock CameraInfo message
        camera_info_msg = CameraInfo()

        # Call the callback
        self.node.dcamera_info_callback(camera_info_msg)

        # Check if the camera model was set
        self.assertIsNotNone(self.node.dcamera_model)

    def test_image_depth_callback(self):
        # Create a mock Image message
        image_msg = Image()

        # Call the callback
        self.node.image_depth_callback(image_msg)

        # Check if the depth image was set
        self.assertIsNotNone(self.node.image_depth_ros)
#
    def test_color2depth_calc(self):
        self.node.ccamera_model = MagicMock()
        self.node.dcamera_model = MagicMock()
        self.node.ccamera_model.width = 640
        self.node.ccamera_model.fx.return_value = 320
        self.node.dcamera_model.width = 640
        self.node.dcamera_model.fx.return_value = 320

        self.node.color2depth_calc()
        self.assertIsNotNone(self.node.color2depth_aspect)

    def test_image2camera_tf(self):
        self.node.ccamera_model = MagicMock()
        self.node.ccamera_model.projectPixelTo3dRay.return_value = np.array([1, 1, 1])

        image_coords = (320, 240)
        image_color = np.zeros((480, 640, 3), dtype=np.uint8)
        image_depth = np.ones((480, 640), dtype=np.float32)

        pose = self.node.image2camera_tf(image_coords, image_color, image_depth)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, 1.0)
        self.assertAlmostEqual(pose.position.z, 1.0)

    @patch('cv2.findContours')
    def test_detect_color_blobs(self, mock_findContours):
        mock_findContours.return_value = ([np.array([[0, 0], [1, 1], [2, 2]])], None)
        color_image = np.zeros((480, 640, 3), dtype=np.uint8)
        contours_dict = self.node.detect_color_blobs(color_image)
        self.assertIn('red', contours_dict)
        self.assertEqual(len(contours_dict['red']), 1)

    @patch('color_3d_detection.do_transform_pose')
    @patch('color_3d_detection.cv2.circle')
    def test_process_contours(self, mock_circle, mock_do_transform_pose):
        mock_do_transform_pose.return_value = PoseStamped()
        self.node.image_color = np.zeros((480, 640, 3), dtype=np.uint8)
        self.node.image_depth = np.ones((480, 640), dtype=np.float32)
        self.node.ccamera_model = MagicMock()
        self.node.ccamera_model.projectPixelTo3dRay.return_value = [1, 1, 1]

        contours_dict = {'red': [np.array([[0, 0], [1, 1], [2, 2]])]}
        self.node.process_contours(contours_dict)
        self.assertTrue(mock_circle.called)




if __name__ == '__main__':
    unittest.main()