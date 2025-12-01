#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from numpy import dtype
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_4ch_net import *
from skimage.transform import resize
from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int8, Int32
from std_srvs.srv import SetBool, SetBoolResponse
import csv
import time
import tf
import subprocess
from datetime import datetime

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "use_dl_output")
        self.action_num = 1
        self.dl = deep_learning(n_action=self.action_num, n_channel=4)
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.callback_right_camera)
        
        self.mask_center_sub = rospy.Subscriber('/segmentation/ground_mask_center', Image, self.callback_mask_center)
        self.mask_left_sub = rospy.Subscriber('/segmentation/ground_mask_left', Image, self.callback_mask_left)
        self.mask_right_sub = rospy.Subscriber('/segmentation/ground_mask_right', Image, self.callback_mask_right)

        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback_path)
        self.tracker_sub = rospy.Subscriber("/tracker", Odometry, self.callback_tracker)

        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.episode_pub = rospy.Publisher("/nav_cloning_node/episode", Int32, queue_size=1)
        self.srv = rospy.Service('/training', SetBool, self.callback_dl_training)

        self.min_distance = 0.0
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.path_pose = PoseArray()

        self.cv_image = np.zeros((480, 640, 3), np.uint8)
        self.cv_left_image = np.zeros((480, 640, 3), np.uint8)
        self.cv_right_image = np.zeros((480, 640, 3), np.uint8)
        self.cv_mask_center = np.zeros((48, 64), np.uint8)
        self.cv_mask_left = np.zeros((48, 64), np.uint8)
        self.cv_mask_right = np.zeros((48, 64), np.uint8)

        self.learning = True
        self.select_dl = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.is_started = False

        # --- 保存パス設定 ---
        base_subpath = '4chdrive/白環境黒ガレージ/障害物配置/0,0,0/'
        self.path = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), 'data/修論データ/', base_subpath)
        self.output_dir = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), 'data/修論データ/チャンネルファイル/', base_subpath)
        camera_base_dir = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), 'data/修論データ/カメラ画像', base_subpath)
        
        self.front_dir = os.path.join(camera_base_dir, 'front')
        self.left_dir = os.path.join(camera_base_dir, 'left')
        self.right_dir = os.path.join(camera_base_dir, 'right')
        self.mask_front_dir = os.path.join(camera_base_dir, 'mask_front')
        self.mask_left_dir = os.path.join(camera_base_dir, 'mask_left')
        self.mask_right_dir = os.path.join(camera_base_dir, 'mask_right')
        
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/model_' + str(self.mode) + '/'
        
        os.makedirs(self.path + self.start_time, exist_ok=True)
        os.makedirs(self.output_dir, exist_ok=True)
        for d in [self.front_dir, self.left_dir, self.right_dir, 
                  self.mask_front_dir, self.mask_left_dir, self.mask_right_dir]:
            os.makedirs(d, exist_ok=True)

        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M")
        self.output_file = os.path.join(self.output_dir, f"channel_means_{current_time}.csv")
        
        self.save_interval = 1

        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "front_r_mean", "front_g_mean", "front_b_mean",
                             "left_r_mean", "left_g_mean", "left_b_mean",
                             "right_r_mean", "right_g_mean", "right_b_mean"])
        
        with open(self.path + self.start_time + '/' + 'training.csv', 'w') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(['step', 'mode', 'loss', 'angle_error(rad)', 'distance(m)', 'x(m)', 'y(m)', 'the(rad)'])

    def callback(self, data):
        try: self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e: print(e)

    def callback_left_camera(self, data):
        try: self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e: print(e)

    def callback_right_camera(self, data):
        try: self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e: print(e)

    def callback_mask_center(self, data):
        try: self.cv_mask_center = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e: print(e)

    def callback_mask_left(self, data):
        try: self.cv_mask_left = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e: print(e)

    def callback_mask_right(self, data):
        try: self.cv_mask_right = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e: print(e)

    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_dl_training(self, data):
        resp = SetBoolResponse()
        self.learning = data.data
        resp.message = "Training: " + str(self.learning)
        resp.success = True
        return resp

    def callback_tracker(self, data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        rot = data.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        self.pos_the = angle[2]

    def callback_path(self, data):
        self.path_pose = data

    def callback_pose(self, data):
        distance_list = []
        pos = data.pose.pose.position
        for pose in self.path_pose.poses:
            path = pose.pose.position
            distance = np.sqrt(abs((pos.x - path.x)**2 + (pos.y - path.y)**2))
            distance_list.append(distance)
        if distance_list:
            self.min_distance = min(distance_list)

    def to_4ch(self, bgr_img, ground_mask_img_0_255, out_hw=(48, 64)):
        H, W = out_hw
        img = resize(bgr_img, (H, W), mode='constant')
        b, g, r = cv2.split(img)
        r = r.astype(np.float32)
        g = g.astype(np.float32)
        b = b.astype(np.float32)
        mask01 = (ground_mask_img_0_255 / 255.0).astype(np.float32)
        img4 = np.asanyarray([r, g, b, mask01], dtype=np.float32)
        return img4

    def loop(self):
        if self.cv_image.size != 640 * 480 * 3: return
        if self.cv_left_image.size != 640 * 480 * 3: return
        if self.cv_right_image.size != 640 * 480 * 3: return
        if self.vel.linear.x != 0: self.is_started = True
        if self.is_started == False: return

        imgobj       = self.to_4ch(self.cv_image, self.cv_mask_center)
        imgobj_left  = self.to_4ch(self.cv_left_image, self.cv_mask_left)
        imgobj_right = self.to_4ch(self.cv_right_image, self.cv_mask_right)

        front_r_mean = np.mean(imgobj[0]); front_g_mean = np.mean(imgobj[1]); front_b_mean = np.mean(imgobj[2])
        left_r_mean = np.mean(imgobj_left[0]); left_g_mean = np.mean(imgobj_left[1]); left_b_mean = np.mean(imgobj_left[2])
        right_r_mean = np.mean(imgobj_right[0]); right_g_mean = np.mean(imgobj_right[1]); right_b_mean = np.mean(imgobj_right[2])

        with open(self.output_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.episode,
                front_r_mean, front_g_mean, front_b_mean,
                left_r_mean, left_g_mean, left_b_mean,
                right_r_mean, right_g_mean, right_b_mean
            ])

        #if self.episode == 6250:　もとのステップ
        if self.episode == 6250:
            spawn_model_script_path = '/home/ciel/catkin_ws/src/nav_cloning/model_script/my_cylinder/spawn_model.py'
            subprocess.run(['python3', spawn_model_script_path])

        #if self.episode == 6262: もとのステップ
        if self.episode == 6262:
            self.learning = False
            self.dl.save(self.save_path)
        
        if self.episode == 8500:
            os.system('killall roslaunch')
            sys.exit()

        if self.learning:
            target_action = self.action
            distance = self.min_distance
            loss = 0
            angle_error = 0

            if self.mode == "change_dataset_balance" or self.mode == "selected_training":
                action = self.dl.act(imgobj)
                angle_error = abs(action - target_action)
                if angle_error > 0.05:
                    action, loss = self.dl.act_and_trains(imgobj, target_action)
                    if abs(target_action) < 0.1:
                        action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                        action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                
                if distance > 0.1: self.select_dl = False
                elif distance < 0.05: self.select_dl = True
                if self.select_dl and self.episode >= 0: target_action = action
            
            elif self.mode == "use_dl_output":
                action, loss = self.dl.act_and_trains(imgobj, target_action)
                if abs(target_action) < 0.1:
                    action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                    action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                angle_error = abs(action - target_action)
                if distance > 0.1: self.select_dl = False
                elif distance < 0.05: self.select_dl = True
                if self.select_dl and self.episode >= 0: target_action = action

            print(str(self.episode) + ", training, loss: " + str(loss) + ", angle_error: " + str(angle_error) + ", distance: " + str(distance))

            if self.episode % self.save_interval == 0:
                cv2.imwrite(os.path.join(self.front_dir, f"front_{self.episode}.png"), self.cv_image)
                cv2.imwrite(os.path.join(self.left_dir, f"left_{self.episode}.png"), self.cv_left_image)
                cv2.imwrite(os.path.join(self.right_dir, f"right_{self.episode}.png"), self.cv_right_image)
                cv2.imwrite(os.path.join(self.mask_front_dir, f"mask_front_{self.episode}.png"), self.cv_mask_center)
                cv2.imwrite(os.path.join(self.mask_left_dir, f"mask_left_{self.episode}.png"), self.cv_mask_left)
                cv2.imwrite(os.path.join(self.mask_right_dir, f"mask_right_{self.episode}.png"), self.cv_mask_right)

            self.episode += 1
            line = [str(self.episode), "training", str(loss), str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the)]
            with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)

            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
            self.nav_pub.publish(self.vel)

        else:
            target_action = self.dl.act(imgobj)
            distance = self.min_distance
            print(str(self.episode) + ", test, angular:" + str(target_action) + ", distance: " + str(distance))

            self.episode += 1
            angle_error = abs(self.action - target_action)
            line = [str(self.episode), "test", "0", str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the)]
            with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            
            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
            self.nav_pub.publish(self.vel)

        # --- 画像表示 (GUI) ---
        disp_img = cv2.merge((imgobj[2], imgobj[1], imgobj[0]))
        disp_left = cv2.merge((imgobj_left[2], imgobj_left[1], imgobj_left[0]))
        disp_right = cv2.merge((imgobj_right[2], imgobj_right[1], imgobj_right[0]))

        # ★変更点: ここでサイズを小さく (128, 96) に固定し、ウィンドウサイズも強制変更
        display_size = (128*2, 96*2)

        # 左RGB
        resized_img_left = cv2.resize(disp_left, display_size)
        cv2.namedWindow("Resized Left Image", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Resized Left Image", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Resized Left Image", resized_img_left)
        cv2.moveWindow("Resized Left Image", 100, 70)

        # 中央RGB
        resized_img = cv2.resize(disp_img, display_size)
        cv2.namedWindow("Resized Image", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Resized Image", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Resized Image", resized_img)
        cv2.moveWindow("Resized Image", 360, 70)
        
        # 右RGB
        resized_img_right = cv2.resize(disp_right, display_size)
        cv2.namedWindow("Resized Right Image", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Resized Right Image", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Resized Right Image", resized_img_right)
        cv2.moveWindow("Resized Right Image", 620, 70)

        # マスク画像の準備
        mask_front_disp = (imgobj[3] * 255).astype(np.uint8)
        mask_left_disp  = (imgobj_left[3] * 255).astype(np.uint8)
        mask_right_disp = (imgobj_right[3] * 255).astype(np.uint8)

        # 左マスク
        resized_mask_left = cv2.resize(mask_left_disp, display_size)
        cv2.namedWindow("Mask Left", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Mask Left", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Mask Left", resized_mask_left)
        cv2.moveWindow("Mask Left", 100, 300) # RGBの下に配置

        # 中央マスク
        resized_mask_front = cv2.resize(mask_front_disp, display_size)
        cv2.namedWindow("Mask Front", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Mask Front", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Mask Front", resized_mask_front)
        cv2.moveWindow("Mask Front",360, 300) # RGBの下に配置

        # 右マスク
        resized_mask_right = cv2.resize(mask_right_disp, display_size)
        cv2.namedWindow("Mask Right", cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow("Mask Right", display_size[0], display_size[1]) # 強制リサイズ
        cv2.imshow("Mask Right", resized_mask_right)
        cv2.moveWindow("Mask Right", 620, 300) # RGBの下に配置

        cv2.waitKey(1)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
