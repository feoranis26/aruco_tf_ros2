import rclpy
from rclpy.node import Node
import math
import cv2
import numpy as np
from pyquaternion import Quaternion
from cv2 import aruco

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArUco_TF(Node):
    def __init__(self):
        super().__init__('aruco_tf')

        self.rate = self.declare_parameter("rate", 30).value
        self.camera_frame_id = self.declare_parameter("camera_frame_id", "camera").value
        self.tag_frame_id = self.declare_parameter("tag_frame_id", "tag").value
        
        self.capture = cv2.VideoCapture("http://172.18.64.1:8081/video.mjpg")

        if not self.capture.isOpened():
            raise Exception("Cannot open camera")

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        self.params = aruco.DetectorParameters()

        self.cam_matrix = np.loadtxt("/home/feoranis/camera_matrix.txt")
        self.cam_coeff = np.loadtxt("/home/feoranis/camera_coeff.txt")

        self.board = aruco.GridBoard(
                    size=(2, 2),
                    markerLength=50,
                    markerSeparation=5,
                    dictionary=self.dictionary)

        self.detector = aruco.ArucoDetector(dictionary=self.dictionary)

        self.broadcaster = TransformBroadcaster(self)

        self.create_timer(1 / self.rate, self.detect)

    def detect(self):
        ret, frame = self.capture.read()
    
        # if frame is read correctly ret is True
        if not ret:
            raise Exception("Can't receive frame (stream end?). Exiting ...")
            

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        (corners, ids, rejected) = self.detector.detectMarkers(gray)
        #(corners, ids, rejected, recoveredIds) = aruco.refineDetectedMarkers(gray, board, corners, ids, rejected, cam_matrix, cam_coeff)
        
        #out_frame = aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))

        if ids is not None and len(ids) >= 0:
            pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.cam_matrix, self.cam_coeff, None, None)
            rvecs, tvecs, points = aruco.estimatePoseSingleMarkers(corners, 50, self.cam_matrix, self.cam_coeff)

            if pose:
                #cv2.drawFrameAxes(out_frame, self.cam_matrix, self.cam_coeff, rvec, tvec, 105)
                print(tvec[0], tvec[1], tvec[2])

                matrix, _ = cv2.Rodrigues(rvec)
                #euler = rotationMatrixToEulerAngles(matrix)
                quat = Quaternion(matrix=matrix)
                rotated_vec = quat.rotate([0, -1, 0])
                #print(rotated_vec)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.camera_frame_id
                t.child_frame_id = self.tag_frame_id

                t.transform.translation.x = -tvec[0][0] / 1000.0
                t.transform.translation.y = -tvec[1][0] / 1000.0
                t.transform.translation.z = tvec[2][0] / 1000.0

                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                # Send the transformation
                self.broadcaster.sendTransform(t)

                #print("Pitch: {}, Yaw: {}, Roll: {}".format(math.degrees(euler[0]), math.degrees(euler[2]), math.degrees(euler[1])))

            
            #for i in range(len(tvecs)):
            #    cv2.drawFrameAxes(out_frame, self.cam_matrix, self.cam_coeff, rvecs[i], tvecs[i], 50)

        
        # Display the resulting frame
        #cv2.imshow('out', out_frame)


def main(args=None):
    rclpy.init(args=args)
    my_node = ArUco_TF()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()