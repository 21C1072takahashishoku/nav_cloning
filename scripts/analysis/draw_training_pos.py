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



def draw_training_pos():
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

            #if 0<= i <= 1700:
                #patch = Circle(xy=(x, y), radius=0.03, facecolor="black")
            if 40<= i <= 1570 and i % 10 == 0:
                patch = Circle(xy=(x, y), radius=0.08, facecolor="black")
            #elif 6261 <= i <= 6261:
                #patch = Circle(xy=(x, y), radius=0.3, facecolor="black")
            elif 6262 <= i <= 8400:
                patch = Circle(xy=(x, y), radius=0.03, facecolor="red")
            #elif 8400 <= i <= 8500:
                #patch = Circle(xy=(x, y), radius=0.3, facecolor="blue")
            #elif 6262 <= i <= 8400:
            #    patch = Circle(xy=(x, y), radius=0.03, facecolor="red")
            #elif 8400 <= i <= 8500:
            #    patch = Circle(xy=(x, y), radius=0.3, facecolor="blue")

            
            else:
                continue
            ax.add_patch(patch)  
            

    
    #関数を呼び出して円柱を出現させ
    cylinders_coordinates = [
     ((18.5, 8.5), "black"),
    ]
    # #赤色と青色の円柱を描画する関数を呼び出します
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)
    
    ax.set_xlim([-5, 30])#x軸の表示範囲
    ax.set_ylim([-5, 15])#y軸の表示範囲
    #ax.set_xlim([-10, 55])#x軸(横軸)の表示範囲
    #ax.set_ylim([-5, 45])#y軸(縦軸)の表示範囲
    #ax.set_xlim([-15, 50])  # x軸の表示範囲
    #ax.set_ylim([-15, 50])  # y軸の表示範囲
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Position y (m)")


    pyplot.show()

#def draw_cylinders_at_coordinates(ax, coordinates_list):
#     # 座標リストの各座標に赤色と青色の円柱を描画する関数
#    for (x, y), color in coordinates_list:
#         cylinder_patch = Circle(xy=(x, y), radius=1,facecolor='none', #edgecolor=color,linestyle='--', linewidth=1  )
#         ax.add_patch(cylinder_patch)


def draw_cylinders_at_coordinates(ax, coordinates_list):
    # 座標リストの各座標に赤色と青色の円柱（円）を描画する関数
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(
            xy=(x, y),
            radius=1,
            facecolor='none',
            edgecolor=color,
            linewidth=1,
            linestyle=(0, (3, 6))  # (dash_length=5, gap_length=10)
        )
        ax.add_patch(cylinder_patch)



if __name__ == '__main__':
    draw_training_pos()
