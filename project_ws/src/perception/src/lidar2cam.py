#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
import time
import rospkg
import os
import ros_numpy
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
import sensor_msgs.point_cloud2 as pc2
from ultralytics import YOLO
import torch
# torch.cuda.set_device(0)

parameters_cam = {
    "WIDTH": 1280, # image width
    "HEIGHT": 720, # image height
    "FOV": 100, # Field of view
    "X": 0.3, # meter
    "Y": 0,
    "Z": 1.60,
    "YAW": 0,
    "PITCH": 0.0,
    "ROLL": 0
}

parameters_lidar = {
    "X": 0.00, # meter
    "Y": 0,
    "Z": 2.00,
    "YAW": 0, # -7.5*math.pi/180.0. # radian
    "PITCH": 0.0,
    "ROLL": 0
}

def translationMtx(x, y, z):

    M = np.array([[1,        0,          0,          x],
                  [0,        1,          0,          y],
                  [0,        0,          1,          z],
                  [0,        0,          0,          1],
                  ])
    
    return M

def rotationMtx(yaw, pitch, roll):

    R_x = np.array([[1,      0,             0,               0],
                    [0,      math.cos(roll), -math.sin(roll) ,0],
                    [0,      math.sin(roll), math.cos(roll)  ,0],
                    [0,      0,             0,               1],
                    ])
    
    R_y = np.array([[math.cos(pitch),   0,  math.sin(pitch) , 0],
                    [0,                 1,  0               , 0],
                    [-math.sin(pitch), 0,  math.cos(pitch) , 0],
                    [0,                 0,  0               , 1],
                    ])
    
    R_z = np.array([[math.cos(yaw),     -math.sin(yaw) , 0 , 0],
                    [math.sin(yaw),      math.cos(yaw) , 0 , 0],
                    [0,                 0              , 1 , 0],
                    [0,                 0              , 0 , 1],
                    ])
    R = np.matmul(R_x, np.matmul(R_y, R_z))

    return R


def transformMTX_lidar2cam(params_lidar, params_cam):
    lidar_pos = [params_lidar.get(i) for i in (["X", "Y", "Z"])]
    cam_pos = [params_cam.get(i) for i in (["X", "Y", "Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(translationMtx(x_rel, y_rel, z_rel), rotationMtx(np.deg2rad(-90), 0., 0.))
    R_T = np.matmul(R_T, rotationMtx(0, 0., np.deg2rad(-90)))

    R_T = np.linalg.inv(R_T)

    return R_T


def project2img_mtx(params_cam):
    fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2

    R_f = np.array([[fc_x, 0, cx],
                    [0, fc_y, cy]])

    return R_f


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):
        self.model = YOLO('/home/jeongwoo/carla_ws/src/perception/src/best.pt')  # model 입력
        # self.model.to('cuda')
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)
        self.proj_mtx = project2img_mtx(params_cam)
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/voxel', PointCloud2, self.lidar_cb)
        self.cam_sub = rospy.Subscriber('/carla/hero/front/image', Image, self.img_cb)
        # self.image_pub = rospy.Publisher('/result_img', Image, queue_size=1)
        self.points = None
        self.img = None
        self.results = None
        self.angle = None
        # self.st = time.time()

    def lidar_cb(self, msg):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        #self.points = self.points[:, [1, 0, 2]]
        print(self.points)

    def img_cb(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.img is None or self.img.size == 0:
                rospy.logwarn("Empty image received")
                return
            # rospy.loginfo(f"Received image with dimensions: {self.img.shape}")
            # cv2.imshow("img", self.img)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

    def transform_lidar2cam(self, xyz_p):
       
        #print(xyz_p)
        # y 좌표에 - 부호를 붙입니다.
        xyz_p[:, 1] = -xyz_p[:, 1]
        xyz_p[:, 2] = -xyz_p[:, 2]
        d = np.array([np.sqrt(x**2 + y**2) for x, y in xyz_p[:, :2]])
        d = d[:, np.newaxis]
        xyz_c = np.matmul(np.concatenate([xyz_p, d], axis=1), self.RT.T)
        
        return xyz_c
    
    def project_pts2img(self, xyz_c, crop=True):
        xyz_c = xyz_c.T

        xc, yc, zc, d = xyz_c[0, :].reshape([1, -1]), xyz_c[1,:].reshape([1, -1]), xyz_c[2,:].reshape([1, -1]), xyz_c[3,:].reshape([1, -1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = np.vstack([xyi, d])
        #print(xyi)

        xyi = xyi[0:3, :].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        #print(np.shape(xyi))
        return xyi
    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:,0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:,1]<self.height), :]
        return xyi
     
    def draw_pts_img(self, img, xi, yi, di):
        point_np = img
        # 5m ~ 30m normalize
        # B, G, R
        for ctr in zip(xi, yi, di):
            ctp = (int(ctr[0]), int(ctr[1]))
            #print(ctr)
            dist = ctr[2]
            color = None
            if dist < 10:
                color = (0, 0, 255) # red
            elif dist < 20:
                color = (0, 255, 0) # green
            else:
                color = (255, 0, 0)

            point_np = cv2.circle(point_np, ctp, 2, color, -1)
        
        return point_np

    def detect_person(self, xyi, img):
        for r in self.results:
            size = r.boxes.xyxy.cpu().numpy()
            print("num cls :", len(size))
            if len(size) != 0:
                for i in range(len(size)):
                    x = size[i][0]
                    y = size[i][1]
                    x_e = size[i][2]
                    y_e = size[i][3]
                    
                    xy_p = xyi[(xyi[:,0] > x) & (xyi[:,0] < x_e) & (xyi[:,1] > y) & (xyi[:,1] < y_e)]
                    
                    if len(xy_p) > 0:  
                        left_most_point = xy_p[xy_p[:,0].argmin()]
                        right_most_point = xy_p[xy_p[:,0].argmax()]
                        
                        print(f"{i}th box's left-most point x,y: {left_most_point[:2]}, right-most point x,y: {right_most_point[:2]}")
                        
                        distance_x = right_most_point[0] - left_most_point[0]
                        print(f"{i}th box's horizontal distance: {distance_x}")
                    else:
                        print("No points found in the bounding box.")
                    
                    # 최소 거리 계산
                    dist = min(xy_p[:,2]) if len(xy_p) > 0 else 0
                    print("{0}th dist : {1}".format(i, dist))
                    
                    # YOLO bounding box에 거리 정보를 이미지에 추가  
                    cv2.putText(img, f"Dist: {dist:.2f}", (int(x), int(y_e) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
        return img
    
    



    def main(self):
        st = time.time()
        # 라이다 -> 카메라 
        xyz_c = self.transform_lidar2cam(self.points)
        xyi = self.project_pts2img(xyz_c)
        xyi = self.crop_pts(xyi)
        #print(np.shape(xyi[:,0]))

        # YOLO
        self.results = self.model(self.img,verbose=False, conf= 0.55)
        result_img = self.results[0].plot()
        # cv2.imshow('yolo', result_img)
        result_img = self.detect_person(xyi, result_img)
        img = self.draw_pts_img(result_img, xyi[:,0], xyi[:,1], xyi[:,2])
        # img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # self.image_pub.publish(img_msg)
        
        cv2.imshow('transform_result', img)
        
        cv2.waitKey(1)

if __name__=="__main__":
    rospy.init_node("lidar_transform")

    transformer = LIDAR2CAMTransform(parameters_cam, parameters_lidar)
    rospy.wait_for_message('/voxel', PointCloud2)
    rospy.wait_for_message('/carla/hero/front/image', Image)
    rospy.loginfo('start')
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        transformer.main()
        rate.sleep()
