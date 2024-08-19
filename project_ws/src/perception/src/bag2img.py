#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rosbag 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    bag = rosbag.Bag("/home/jeongwoo/carla_ws/src/bag/control_loss3.bag", "r")
    bridge = CvBridge()
    count = 390
    frame_interval = 1  # 원하는 프레임 간격 설정

    for topic, msg, t in bag.read_messages("/carla/hero/front/image"):
        count += 1
        if count % frame_interval == 0:  # 설정한 간격마다 이미지 저장
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join("/home/jeongwoo/carla_ws/src/img/learning", "control_loss%05i.png" % (count // frame_interval)), cv_img)
            print("Wrote image %i" % (count // frame_interval))

    bag.close()

if __name__ == '__main__':
    main()
