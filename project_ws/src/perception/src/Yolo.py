#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        self.model = YOLO('/home/jeongwoo/carla_ws/src/perception/src/best.pt')  # YOLO 모델 경로
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/carla/hero/front/image', Image, self.image_callback)
        self.img = None

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.img is None or self.img.size == 0:
                rospy.logwarn("Empty image received")
                return
            
            # Run YOLO model on the image
            results = self.model(self.img, verbose=False, conf=0.55)
            result_img = results[0].plot()

            # Display the result
            cv2.imshow('YOLO Detection', result_img)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def run(self):
        # if hasattr(self.model, 'model'):
        #     print("모델이 성공적으로 로드되었습니다.")
        #     print(self.model.model)
        # else:
        #     print("모델 로드에 실패했습니다.")
        # 클래스 이름 출력
        if hasattr(self.model, 'names'):
            print("클래스 이름:", self.model.names)
        else:
            print("모델에서 클래스 이름을 찾을 수 없습니다.")

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('yolo_detector')
    detector = YOLODetector()
    rospy.loginfo('YOLO detector started')
    detector.run()
