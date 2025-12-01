#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
# from cv_bridge import CvBridge # ★削除: cv_bridgeは競合するため使いません

import torch
from mmcv import Config
from mmseg.apis import init_segmentor as init_model, inference_segmentor as inference_model

# ▼▼▼ 1. 受信用の変換関数 (cv_bridgeの代わり) ▼▼▼
def manual_ros_to_cv2(msg):
    """ ROS Image -> OpenCV画像 (BGR) """
    dtype = np.uint8
    img_buf = np.frombuffer(msg.data, dtype=dtype)
    img = img_buf.reshape(msg.height, msg.width, -1)
    
    # ROSのrgb8をOpenCVのBGRに変換
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1]
    return img

# ▼▼▼ 2. 送信用の変換関数 (cv_bridgeの代わり) ▼▼▼
def manual_cv2_to_ros(img, encoding='mono8'):
    """ OpenCV画像 -> ROS Image """
    msg = Image()
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = img.shape[1] * (1 if encoding == 'mono8' else 3)
    msg.data = img.tobytes()
    return msg

class SegmentationServer:
    # ===== 設定 =====
    IN_CENTER_TOPIC  = '/camera/rgb/image_raw'
    IN_LEFT_TOPIC    = '/camera_left/rgb/image_raw'
    IN_RIGHT_TOPIC   = '/camera_right/rgb/image_raw'
    OUT_MASK_CENTER_TOPIC = '/segmentation/ground_mask_center'
    OUT_MASK_LEFT_TOPIC   = '/segmentation/ground_mask_left'
    OUT_MASK_RIGHT_TOPIC  = '/segmentation/ground_mask_right'
    
    # MMSegモデル
    # newsegprojectの方（公式フォルダ）を指定
    CFG = '/home/ciel/catkin_ws/src/nav_cloning/mmseg/configs/deeplabv3/deeplabv3_r101-d8_512x512_160k_ade20k.py'
    CKP = '/home/ciel/catkin_ws/src/nav_cloning/mmseg/checkpoints/deeplabv3_r101-d8_512x512_160k_ade20k_20200615_105816-b1f72b3b.pth'
    DEVICE    = 'cuda:0'
    SCALE_HW  = (48, 64)
    RATE_HZ   = 8
    GROUND_NAMES = {
        "floor", "road", "sidewalk", "path", "runway", "earth", "land",
        "field", "sand", "rug", "carpet", "mat", "stage", "bridge", "stairs", "step"
    }
    WARMUP_ITERS = 2
    # ===============================================

    def __init__(self):
        rospy.init_node('segmentation_server', anonymous=True)
        # self.bridge = CvBridge() # ★削除

        # モデル読み込み
        rospy.loginfo('[segmentation_server] loading model... (cfg=%s)', self.CFG)
        self.model = init_model(self.CFG, self.CKP, device=self.DEVICE)
        self.model.eval()

        # ===== dataset_meta が無いモデルへの対応 =====
        ade_classes = [
            'wall', 'building', 'sky', 'floor', 'tree', 'ceiling', 'road', 'bed', 'windowpane', 'grass',
            'cabinet', 'sidewalk', 'person', 'earth', 'door', 'table', 'mountain', 'plant', 'curtain',
            'chair', 'car', 'water', 'painting', 'sofa', 'shelf', 'house', 'sea', 'mirror', 'rug',
            'field', 'armchair', 'seat', 'fence', 'desk', 'rock', 'wardrobe', 'lamp', 'bathtub', 'railing',
            'cushion', 'base', 'box', 'column', 'signboard', 'chest of drawers', 'counter', 'sand',
            'sink', 'skyscraper', 'fireplace', 'refrigerator', 'grandstand', 'path', 'stairs', 'runway',
            'case', 'pool table', 'pillow', 'screen door', 'stairway', 'river', 'bridge', 'bookcase',
            'blind', 'coffee table', 'toilet', 'flower', 'book', 'hill', 'bench', 'countertop', 'stove',
            'palm', 'kitchen island', 'computer', 'swivel chair', 'boat', 'bar', 'arcade machine', 'hovel',
            'bus', 'towel', 'light', 'truck', 'tower', 'chandelier', 'awning', 'streetlight', 'booth',
            'television receiver', 'airplane', 'dirt track', 'apparel', 'pole', 'land', 'bannister',
            'escalator', 'ottoman', 'bottle', 'buffet', 'poster', 'stage', 'van', 'ship', 'fountain',
            'conveyer belt', 'canopy', 'washer', 'plaything', 'swimming pool', 'stool', 'barrel', 'basket',
            'waterfall', 'tent', 'bag', 'minibike', 'cradle', 'oven', 'ball', 'food', 'step', 'tank',
            'trade name', 'microwave', 'pot', 'animal', 'bicycle', 'lake', 'dishwasher', 'screen', 'blanket',
            'sculpture', 'hood', 'sconce', 'vase', 'traffic light', 'tray', 'ashcan', 'fan', 'pier',
            'crt screen', 'plate', 'monitor', 'bulletin board', 'shower', 'radiator', 'glass', 'clock', 'flag'
        ]

        name2id = {n: i for i, n in enumerate(ade_classes)}
        self.ground_ids = np.array(
            [name2id[n] for n in self.GROUND_NAMES if n in name2id],
            dtype=np.int32
        )
        unknown = [n for n in self.GROUND_NAMES if n not in name2id]
        if unknown:
            rospy.logwarn(f'[segmentation_server] unknown ground names in this model: {unknown}')

        rospy.loginfo('[segmentation_server] model loaded.')

        # ===== ROS通信設定 =====
        self.pub_mask_center = rospy.Publisher(self.OUT_MASK_CENTER_TOPIC, Image, queue_size=1)
        self.pub_mask_left   = rospy.Publisher(self.OUT_MASK_LEFT_TOPIC,   Image, queue_size=1)
        self.pub_mask_right  = rospy.Publisher(self.OUT_MASK_RIGHT_TOPIC,  Image, queue_size=1)

        rospy.Service('/segmentation/warmup', Trigger, self._srv_warmup)

        self.min_period = rospy.Duration(1.0 / max(1, self.RATE_HZ))
        self.last_pub_center = rospy.Time(0)
        self.last_pub_left   = rospy.Time(0)
        self.last_pub_right  = rospy.Time(0)

        self.sub_center = rospy.Subscriber(self.IN_CENTER_TOPIC, Image, self._cb_center, queue_size=1, buff_size=2**24)
        self.sub_left   = rospy.Subscriber(self.IN_LEFT_TOPIC,   Image, self._cb_left,   queue_size=1, buff_size=2**24)
        self.sub_right  = rospy.Subscriber(self.IN_RIGHT_TOPIC,  Image, self._cb_right,  queue_size=1, buff_size=2**24)

        # ウォームアップ
        self._do_warmup(self.WARMUP_ITERS)

        rospy.loginfo('[segmentation_server] ready. rate=%sHz', self.RATE_HZ)
        rospy.loginfo(f'[segmentation_server] Center: {self.IN_CENTER_TOPIC} -> {self.OUT_MASK_CENTER_TOPIC}')
        rospy.loginfo(f'[segmentation_server] Left:   {self.IN_LEFT_TOPIC} -> {self.OUT_MASK_LEFT_TOPIC}')
        rospy.loginfo(f'[segmentation_server] Right:  {self.IN_RIGHT_TOPIC} -> {self.OUT_MASK_RIGHT_TOPIC}')

    def _do_warmup(self, iters=1):
        if iters <= 0:
            return
        H, W = 240, 320
        rgb = np.zeros((H, W, 3), dtype=np.uint8)
        with torch.no_grad():
            for _ in range(iters):
                _ = inference_model(self.model, rgb)

    def _srv_warmup(self, _req):
        self._do_warmup(self.WARMUP_ITERS)
        return TriggerResponse(success=True, message='warmed')

    # ===== コールバック =====
    @torch.no_grad()
    def _cb_center(self, msg: Image):
        now = rospy.Time.now()
        if now - self.last_pub_center < self.min_period:
            return
        self.last_pub_center = now
        out_mask_msg = self._process_image(msg)
        if out_mask_msg:
            self.pub_mask_center.publish(out_mask_msg)

    @torch.no_grad()
    def _cb_left(self, msg: Image):
        now = rospy.Time.now()
        if now - self.last_pub_left < self.min_period:
            return
        self.last_pub_left = now
        out_mask_msg = self._process_image(msg)
        if out_mask_msg:
            self.pub_mask_left.publish(out_mask_msg)

    @torch.no_grad()
    def _cb_right(self, msg: Image):
        now = rospy.Time.now()
        if now - self.last_pub_right < self.min_period:
            return
        self.last_pub_right = now
        out_mask_msg = self._process_image(msg)
        if out_mask_msg:
            self.pub_mask_right.publish(out_mask_msg)

    # ===== メイン推論処理 =====
    def _process_image(self, msg: Image):
        # 1. 画像の受信変換 (manual_ros_to_cv2を使用)
        try:
            # bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            bgr = manual_ros_to_cv2(msg) # ★書き換え
        except Exception as e:
            rospy.logwarn_throttle(2.0, f'[segmentation_server] image conversion fail: {e}')
            return None
            
        if bgr is None or bgr.size == 0:
            rospy.logwarn_throttle(2.0, '[segmentation_server] empty frame')
            return None

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        try:
            # 2. 推論実行
            res = inference_model(self.model, rgb)
            
            # ★重要: バージョンv0.30.0では res はリスト形式で返ってきます
            # pred_sem_seg ではなく res[0] を使います
            pred = res[0] 
            
        except Exception as e:
            rospy.logerr_throttle(2.0, f'[segmentation_server] inference error: {e}')
            return None
            
        # 地面(ground_ids)が含まれる場所を判定
        is_ground = np.isin(pred, self.ground_ids)
        
        # np.whereを使って色を反転させます
        # 地面(True)なら黒(0)、それ以外(False)なら白(255)
        mask255 = np.where(is_ground, 0, 255).astype(np.uint8)

        H, W = self.SCALE_HW
        if mask255.shape[:2] != (H, W):
            mask255 = cv2.resize(mask255, (W, H), interpolation=cv2.INTER_NEAREST)

        # 3. 画像の送信変換 (manual_cv2_to_rosを使用)
        # out = self.bridge.cv2_to_imgmsg(mask255, encoding='mono8')
        out = manual_cv2_to_ros(mask255, encoding='mono8') # ★書き換え
        
        out.header = msg.header
        return out


if __name__ == '__main__':
    SegmentationServer()
    rospy.spin()
