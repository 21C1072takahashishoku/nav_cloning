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
import os


def draw_training_pos():
    rospy.init_node('draw_training_pos_node', anonymous=True)
    path = '/home/ciel/catkin_ws/src/nav_cloning/data/修論データ/6262/環境光変化/1(投稿データ)/青廊下＆赤ガレージ/障害物配置/自己発光/0,0,0/'
    #image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/map.png').convert("L")#willowgarage
    image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/cit_3f_map.pgm').convert("L")#津田沼２号館３階
    arr = np.asarray(image)
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    ax.imshow(arr, cmap='gray', extent=[-100,100,-100,100])
    
    # 再帰的にフォルダを探索し、CSVファイルを見つける関数を呼び出す
    for csv_file in find_csv_files(path):
        with open(csv_file, 'r') as f:
            is_first = True
            for i, row in enumerate(csv.reader(f)):
                if is_first:
                    is_first = False
                    continue
                str_step, str_mode, str_loss, str_angle_error, str_distance, str_x, str_y, str_the = row
                x, y, the = float(str_x), float(str_y), float(str_the)
                
                if 9000 <= i <= 9000:
                    patch = Circle(xy=(x, y), radius=0.3, facecolor="black")
                elif 9001 <= i <= 11999:
                    patch = Circle(xy=(x, y), radius=0.03, facecolor="red")
                elif 12000 <= i <= 12000:
                    patch = Circle(xy=(x, y), radius=0.1, facecolor="blue")
                else:
                    continue
                ax.add_patch(patch)
    
    # 関数を呼び出して円柱を描画
    cylinders_coordinates = [
      ((18.5, 8.5), "black"),
     ]
    
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)
    #ここまで
    
    #ax.set_xlim([-5, 30])#x軸の表示範囲
    #ax.set_ylim([-5, 15])#y軸の表示範囲
    ax.set_xlim([-15, 50])  # x軸の表示範囲
    ax.set_ylim([-15, 50])  # y軸の表示範囲
    pyplot.show()

def find_csv_files(directory):
    # 再帰的にディレクトリを探索してCSVファイルを見つける
    csv_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.csv'):
                csv_files.append(os.path.join(root, file))
    return csv_files

def draw_cylinders_at_coordinates(ax, coordinates_list):
    # 指定された座標に円柱を描画する
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(xy=(x, y), radius=1.5, facecolor=color)
        ax.add_patch(cylinder_patch)

if __name__ == '__main__':
    draw_training_pos()

