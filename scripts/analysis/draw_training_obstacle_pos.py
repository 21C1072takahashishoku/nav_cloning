#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import csv
import math
from matplotlib import pyplot
from matplotlib.patches import Circle, Polygon
import numpy as np
from PIL import Image
import argparse

def draw_training_pos(num_coordinates):
    rospy.init_node('draw_training_pos_node', anonymous=True)
    path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/analysis/'
    image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/map.png').convert("L")
    arr = np.asarray(image)
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    ax.imshow(arr, cmap='gray', extent=[-10, 50, -10, 50])
    vel = 0.2
    arrow_dict = dict(arrowstyle="->", color="black")
    
    with open(path + 'training.csv', 'r') as f:
        is_first = True
        for i, row in enumerate(csv.reader(f)):
            if is_first:
                is_first = False
                continue
            str_step, str_mode, str_loss, str_angle_error, str_distance, str_x, str_y, str_the = row
            x, y, the = float(str_x), float(str_y), float(str_the)
            if 1 <= i <= 3750:
                patch = Circle(xy=(x, y), radius=0.03, facecolor="gray")
            elif 3751 <= i <= 5400:
                patch = Circle(xy=(x, y), radius=0.03, facecolor="red")
            else:
                continue
            ax.add_patch(patch)
    
    cylinders_coordinates = [
        #((18.5, 8.5), "black"),
    ]
    
    # ファイルから座標を読み込む
    obstacle_file_path = '/home/ciero/catkin_ws/src/nav_cloning/data/vel_0.2&duration_0.3/step5400/範囲２/障害物配置箇所/'
    with open(obstacle_file_path + 'spawned_models.csv', 'r') as f:
        csv_reader = csv.reader(f)#CSVファイルの各行を順番に読み取るためのオブジェクト
        next(csv_reader)#csvファイルの最初の行をスキップするために使用
        for i, row in enumerate(csv_reader):
            if i >= num_coordinates:
                break
            _, x, y, _ = row
            cylinders_coordinates.append(((float(x), float(y)), "red"))
    
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)
    
    ax.set_xlim([-5, 30])
    ax.set_ylim([-5, 15])
    pyplot.show()

def draw_cylinders_at_coordinates(ax, coordinates_list):
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(xy=(x, y), radius=0.5, facecolor=color)
        ax.add_patch(cylinder_patch)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='トレーニング位置と障害物を描画する')
    parser.add_argument('--num_coordinates', type=int, default=5, help='障害物ファイルから読み込む座標の数')
    args = parser.parse_args()
    
    draw_training_pos(args.num_coordinates)
