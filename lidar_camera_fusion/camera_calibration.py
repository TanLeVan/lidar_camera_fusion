#!/usr/bin/env python3
import cv2 as cv
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import json

class CamCalibration(Node):
    def __init__(self):
        super().__init__("camera_calibration")
        self.subscriber = self.create_subscription(Image,"front_image", self.camera_calibration_callback, 10)
        self.bridge = CvBridge()
        self.num_of_image = 0
        #prepare object points like (0,0,0), (25,0,0), (0,25,0),... Length in (mm)
        self.objp = np.zeros((6*8,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)*25
        self.objpoints = []
        self.imgpoints = []

        self.cam_matrix_list = [] 
        self.output_file = "../config/camera_matrix.json"

    def camera_calibration_callback(self, image_msg):
        if self.num_of_image <= 20:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg)
                gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
                ret, corners = cv.findChessboardCorners(gray, (6,8), None)
                if ret == True:
                    self.objpoints.append(self.objp)
                    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1),  (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    self.imgpoints.append(corners2)
                    cv.drawChessboardCorners(cv_image, (6,8), corners2, ret)
                    cv.imshow('img', cv_image)
                    cv.waitKey(5)
                    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, (gray.shape[1], gray.shape[0]), None, None)
                    self.cam_matrix_list.append(mtx)
                    self.num_of_image+=1

            except Exception as e:
                self.get_logger().error(f"Error converting image: {str(e)}")
        elif self.num_of_image > 20:
            sum = 0
            for camera_matrix in self.cam_matrix_list:
                sum += camera_matrix
            camera_matrix = sum/len(self.cam_matrix_list)
            data = {"camera_matrix": camera_matrix.tolist()}
            with open(self.output_file,"w") as f:
                json.dump(data,f)
            print("Finish calibration")
            cam_calibration.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    cam_calibration = CamCalibration()
    rclpy.spin(cam_calibration)
    cv.destroyAllWindows()
    cam_calibration.destroy_node()
    rclpy.shutdown()

