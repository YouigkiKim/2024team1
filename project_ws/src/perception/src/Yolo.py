#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from custom_msgs.msg import yolo_state

class YOLODetector:
    def __init__(self):
        self.model = YOLO('/home/jeongwoo/carla_ws/src/perception/src/best3.pt')  # YOLO 모델 경로
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/carla/hero/front/image', Image, self.image_callback)
        self.yolo_pub = rospy.Publisher('/yolo_state', yolo_state, queue_size=10)  # /yolo 토픽 퍼블리셔
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

            # Initialize the custom message
            yolo_msg = yolo_state()
            yolo_msg.red = False
            yolo_msg.yellow = False
            yolo_msg.green = False
            yolo_msg.person = False
            yolo_msg.bicycle = False
            yolo_msg.control_loss = False
            yolo_msg.red_bbox_center_y = -1  # 초기화 (존재하지 않는 경우)

            # Process YOLO results
            for result in results[0].boxes:
                class_id = int(result.cls)
                class_name = self.model.names[class_id]
                
                if class_name == 'red':
                    yolo_msg.red = True
                    bbox = result.xyxy[0].numpy().astype(int)  # 첫 번째 객체의 좌표만 추출

                    # Check if bbox has the expected length of 4 (x1, y1, x2, y2)
                    if bbox.size == 4:
                        center_y = (bbox[1] + bbox[3]) // 2  # 바운딩 박스의 중앙 X 좌표 계산
                        yolo_msg.red_bbox_center_y = center_y
                    else:
                        rospy.logwarn(f"Unexpected bbox size: {len(bbox)}")
                    
                if class_name == 'yellow':
                    yolo_msg.yellow = True
                if class_name == 'green':
                    yolo_msg.green = True
                if class_name == 'person':
                    yolo_msg.person = True
                if class_name == 'bicycle':
                    yolo_msg.bicycle = True
                if class_name == 'control_loss':
                    yolo_msg.control_loss = True

            # Publish the message to the /yolo topic
            self.yolo_pub.publish(yolo_msg)

            # Display the result
            result_img = results[0].plot()
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
        # # 클래스 이름 출력
        # if hasattr(self.model, 'names'):
        #     print("클래스 이름:", self.model.names)
        # else:
        #     print("모델에서 클래스 이름을 찾을 수 없습니다.")

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('yolo_detector')
    detector = YOLODetector()
    rospy.loginfo('YOLO detector started')
    detector.run()
