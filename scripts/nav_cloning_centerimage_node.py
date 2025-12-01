#!/usr/bin/env python3
from __future__ import print_function
from numpy import dtype#githubのやつにあったが、今までの自分にはなかった
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_net import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.srv import DeleteModel #追加。特定のモデルの削除ができる
from gazebo_msgs.srv import DeleteModel #add
import csv
import os
import time
import copy
import sys
import tf
import subprocess#追加した。サブプロセスを起動するため
import subprocess
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from datetime import datetime#追加した。日付を含めたファイルの作成のため

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "use_dl_output")
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.callback_right_camera)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.srv = rospy.Service('/training', SetBool, self.callback_dl_training)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback_path)
        self.min_distance = 0.0
        self.action = 0.0
        #add
        #self.episode_pub = rospy.Publisher("/nav_cloning_node/episode", Int32, queue_size=1)
        self.episode_pub = rospy.Publisher("/nav_cloning_node/episode", Int32, queue_size=1)
        #end
        self.episode = 0
        self.vel = Twist()
        self.path_pose = PoseArray()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.learning = True
        self.select_dl = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        # 共通のベースパス
        base_frontpath = 'data/修論データ'
        base_backpath = '6262/白廊下黒ガレ(shading)/障害物配置/0,0,0'
	# 各ディレクトリ
        self.path = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'),base_frontpath,  base_backpath) #ノーマルデータ
        self.output_dir = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), base_frontpath, 'チャンネルファイル', base_backpath) #チャンネルデータ
    # カメラごとのディレクトリパスを作成
        camera_base_dir = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), base_frontpath, 'カメラ画像', base_backpath)

        self.front_dir = os.path.join(camera_base_dir, 'front')
        self.left_dir = os.path.join(camera_base_dir, 'left')
        self.right_dir = os.path.join(camera_base_dir, 'right')
        #self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/修論データ/6262/環境光変化/1(投稿データ)/通常ガレージ/白ロボット/0,0,0/'
        #self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/修論データ/6262/環境光変化/1(投稿データ)/青廊下＆赤ガレージ/障害物配置/自己発光/0,0,0追実験2/'
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/model_'+str(self.mode)+'/'
        self.previous_reset_time = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_the = 0.0
        self.is_started = False
        self.start_time_s = rospy.get_time()
        os.makedirs(self.path + self.start_time)
        # 現在の日時を取得してフォーマット
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M")
        # ファイル名に日時を埋め込む
        #self.output_dir = '/home/ciel/catkin_ws/src/nav_cloning/data/tamesi/'
        self.output_file = os.path.join(self.output_dir, f"channel_means_{current_time}.csv")
        # ディレクトリの存在確認と作成
        os.makedirs(self.output_dir, exist_ok=True)
        
        # ここに画像保存設定を追加
        # 保存間隔の設定
        self.save_interval = 100  # 100ステップごとに保存
       
        # ディレクトリ作成
        for d in [self.front_dir, self.left_dir, self.right_dir]:
            os.makedirs(d, exist_ok=True)
        
        #Initialize channel means CSV
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "front_r_mean", "front_g_mean", "front_b_mean",  # Front
                             "left_r_mean", "left_g_mean", "left_b_mean",  # Left
                             "right_r_mean", "right_g_mean", "right_b_mean"  # Right
                           ])
        print(f"CSV file created: {self.output_file}")
        
        
        with open(self.path + self.start_time + '/' +  'training.csv', 'w') as f:#ｃｓｖファイルの中の一番上の場所の名前変更
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(['step', 'mode', 'loss', 'angle_error(rad)', 'distance(m)','x(m)','y(m)', 'the(rad)'])
        self.tracker_sub = rospy.Subscriber("/tracker", Odometry, self.callback_tracker)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

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

    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_dl_training(self, data):
        resp = SetBoolResponse()
        self.learning = data.data
        resp.message = "Training: " + str(self.learning)
        resp.success = True
        return resp
       


    def loop(self):
        if self.cv_image.size != 640 * 480 * 3:
            return
        if self.cv_left_image.size != 640 * 480 * 3:
            return
        if self.cv_right_image.size != 640 * 480 * 3:
            return
        if self.vel.linear.x != 0:
            self.is_started = True
        if self.is_started == False:
            return
        img = resize(self.cv_image, (48, 64), mode='constant')
        r, g, b = cv2.split(img)
        imgobj = np.asanyarray([r,g,b])
        
        # Calculate channel means
        front_r_mean = np.mean(imgobj[2])  # R channel mean
        front_g_mean = np.mean(imgobj[1])  # G channel mean
        front_b_mean = np.mean(imgobj[0])  # B channel mean

        img_left = resize(self.cv_left_image, (48, 64), mode='constant')
        r, g, b = cv2.split(img_left)
        imgobj_left = np.asanyarray([r,g,b])
        
        # Calculate channel means
        left_r_mean = np.mean(imgobj_left[2])  # R channel mean
        left_g_mean = np.mean(imgobj_left[1])  # G channel mean
        left_b_mean = np.mean(imgobj_left[0])  # B channel mean

        img_right = resize(self.cv_right_image, (48, 64), mode='constant')
        r, g, b = cv2.split(img_right)
        imgobj_right = np.asanyarray([r,g,b])

        # Calculate channel means
        right_r_mean = np.mean(imgobj_right[2])  # R channel mean
        right_g_mean = np.mean(imgobj_right[1])  # G channel mean
        right_b_mean = np.mean(imgobj_right[0])  # B channel mean

        # Save channel means to CSV
        with open(self.output_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.episode,
                front_r_mean, front_g_mean, front_b_mean,  # Front
                left_r_mean, left_g_mean, left_b_mean,  # Left
                right_r_mean, right_g_mean, right_b_mean  # Right
            ])
            
        ros_time = str(rospy.Time.now())
        


        #if (self.episode ==200):
        #if (self.episode % 1200 == 0 and self.episode <=3700) or (self.episode == 200):
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/spawn_model.py'
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/random_change_shape.py'
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/random_change_color.py'
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/random_change_color_shape.py'
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/random_change_size.py'
            #delete_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/delete_model.py'
            #subprocess.run(['python3', delete_model_script_path])
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/random_spawn_model.py'
            #subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {spawn_model_script_path }'], shell=False)#別のターミナルで実行する
            
        #if (self.episode ==3900):
            #os.system('pkill -f spawn_model.py')#spawnスクリプト解除
            #os.system('pkill -f random_change_shape.py')#shapeスクリプト解除
            #os.system('pkill -f random_change_color.py')#colorスクリプト解除
            #delete_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/delete_model.py'
            #subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {delete_model_script_path }'], shell=False)#別のターミナルで実行する
                #ここからの３行がいつも使うスポーンモデル(2025/2/6記入)
        if self.episode == 6100:
            spawn_model_script_path = '/home/ciel/catkin_ws/src/nav_cloning/model_script/my_cylinder/spawn_model.py'
            subprocess.run(['python3', spawn_model_script_path])
            
        if self.episode == 6000: #6262から変更
            self.learning = False
            self.dl.save(self.save_path)
            
            #使うプログラムによって有効化する
            #os.system('pkill -f random_change_shape.py')           
            #delete_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/delete_model.py'
            #subprocess.run(['python3', delete_model_script_path])



        #if self.episode == 100:
            #spawn_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/spawn_model_only.py'
            #subprocess.run(['python3', spawn_model_script_path])
        
        #if self.episode == 3600:
        #if self.episode == 6300:
            #self.learning = False
            #self.dl.save(self.save_path)
            #self.dl.load(self.load_path)
            
        #if self.episode == 6400:
            #delete_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/delete_model.py'
            #subprocess.run(['python3', delete_model_script_path])
               

        
        #新しくモデルを生成する条件
        #if (self.episode - 100) % 1600 == 0 and self.episode > 50 and self.episode <= 3320:
        #if self.episode == 6420:
            #move_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/moving_color_date.py'#変更点
            #subprocess.run(['python3', move_model_script_path])
            
            #subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {move_model_script_path}'], shell=False)#add
#end

           
        #if self.episode == 3350:
            #move_model_script_path = '/home/ciero/catkin_ws/src/my_models/my_cylinder/move_model2.py'
            #subprocess.run(['python3', move_model_script_path])
        #end
        

        if self.episode == 8500:
            #os.system('pkill -f moving_color_date.py')

            os.system('killall roslaunch')
            sys.exit()

        if self.learning:
            target_action = self.action
            distance = self.min_distance

            if self.mode == "manual":
                if distance > 0.1:
                    self.select_dl = False
                elif distance < 0.05:
                    self.select_dl = True
                if self.select_dl and self.episode >= 0:
                    target_action = 0
                action, loss = self.dl.act_and_trains(imgobj, target_action)
                if abs(target_action) < 0.1:
                    action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                    action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                angle_error = abs(action - target_action)

            elif self.mode == "zigzag":
                action, loss = self.dl.act_and_trains(imgobj, target_action)
                if abs(target_action) < 0.1:
                    action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                    action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                angle_error = abs(action - target_action)
                if distance > 0.1:
                    self.select_dl = False
                elif distance < 0.05:
                    self.select_dl = True
                if self.select_dl and self.episode >= 0:
                    target_action = 0

            elif self.mode == "use_dl_output":
                action, loss = self.dl.act_and_trains(imgobj, target_action)
                if abs(target_action) < 0.1:
                    action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                    action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                angle_error = abs(action - target_action)
                if distance > 0.1:
                    self.select_dl = False
                elif distance < 0.05:
                    self.select_dl = True
                if self.select_dl and self.episode >= 0:
                    target_action = action

            elif self.mode == "follow_line":
                action, loss = self.dl.act_and_trains(imgobj, target_action)
                if abs(target_action) < 0.1:
                    action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                    action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                angle_error = abs(action - target_action)

            elif self.mode == "selected_training":
                action = self.dl.act(imgobj)
                angle_error = abs(action - target_action)
                loss = 0
                if angle_error > 0.05:
                    action, loss = self.dl.act_and_trains(imgobj, target_action)
                    if abs(target_action) < 0.1:
                        action_left,  loss_left  = self.dl.act_and_trains(imgobj_left, target_action - 0.2)
                        action_right, loss_right = self.dl.act_and_trains(imgobj_right, target_action + 0.2)
                if distance > 0.1:
                    self.select_dl = False
                elif distance < 0.05:
                    self.select_dl = True
                if self.select_dl and self.episode >= 0:
                    target_action = action

            # end mode

            print(str(self.episode) + ", training, loss: " + str(loss) + ", angle_error: " + str(angle_error) + ", distance: " + str(distance))
            # === ここに画像保存機能を追加！ ===
            if self.episode % self.save_interval == 0:
            # 各カメラ画像を保存
                cv2.imwrite(os.path.join(self.front_dir, f"front_{self.episode}.png"), self.cv_image)
                cv2.imwrite(os.path.join(self.left_dir, f"left_{self.episode}.png"), self.cv_left_image)
                cv2.imwrite(os.path.join(self.right_dir, f"right_{self.episode}.png"), self.cv_right_image)

    # ===============================
    
            self.episode += 1
            line = [str(self.episode), "training", str(loss), str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the)]
            with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:#学習時のデータ保存場所の指定
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)

            
            self.vel.linear.x = 0.2#元の速度
            #self.vel.linear.x = 0.25#duration0.6で速度0.2に対する0.4の速度(0.3だと学習時に衝突することがある。）
            #self.vel.linear.x = 0.1 #duration 0.4
            #self.vel.linear.x = 0.4
            self.vel.angular.z = target_action
            self.nav_pub.publish(self.vel)

        else:
            target_action = self.dl.act(imgobj)
            distance = self.min_distance
            print(str(self.episode) + ", test, angular:" + str(target_action) + ", distance: " + str(distance))

            self.episode += 1
            angle_error = abs(self.action - target_action)
            line = [str(self.episode), "test", "0", str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the)]
            with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:#テスト時のデータ保存場所の指定
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            self.vel.linear.x = 0.2#元の速度
            #self.vel.linear.x = 0.3#duration0.6で速度0.2に対する0.4の速度
            #self.vel.linear.x = 0.25#duration0.6で速度0.2に対する0.4の速度(0.3だと学習時に衝突することがある。）
            #self.vel.linear.x = 0.1 #duration0.4
            self.vel.angular.z = target_action
            self.nav_pub.publish(self.vel)

        #temp = copy.deepcopy(img)
        #cv2.imshow("Resized Image", temp)
        #temp = copy.deepcopy(img_left)
        #cv2.imshow("Resized Left Image", temp)
        #temp = copy.deepcopy(img_right)
        #cv2.imshow("Resized Right Image", temp)
        #add
        
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


        cv2.waitKey(1)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2#もともと0.2だったが、0.4に上げたらCPU使用率が約半分ほどになった。
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()


