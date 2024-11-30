#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import csv
import numpy as np
from matplotlib import pyplot
from matplotlib.patches import Circle
from PIL import Image

def draw_training_pos():
    rospy.init_node('draw_training_pos_node', anonymous=True)
    path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/analysis/'
    image = Image.open(roslib.packages.get_pkg_dir('nav_cloning') + '/maps/square_road.png').convert("L")
    arr = np.asarray(image)
    
    fig, ax = pyplot.subplots()
    ax.imshow(arr, cmap='gray', extent=[-20, 10, -35, 20])  # extentを1000ピクセルに設定

    with open(path + 'training.csv', 'r') as f:
        is_first = True
        for i, row in enumerate(csv.reader(f)):
            if is_first:
                is_first = False
                continue
            str_step, str_mode, str_loss, str_angle_error, str_distance, str_x, str_y, str_the = row
            x, y, the = float(str_x), float(str_y), float(str_the)

            if 1 <= i <= 10000:
                patch = Circle(xy=(x, y), radius=0.03, facecolor="Gray")
            elif 10600 <= i <= 10601:
                patch = Circle(xy=(x, y), radius=0.5,facecolor="red")
            #elif 3751 <= i <= 5400:
            #elif 4001 <= i <= 6000:
            #elif 2001 <= i <= 3000:#duration２倍
            #elif 12001 <= i <= 15000:
                #patch = Circle(xy=(x, y), radius=0.1,facecolor="red")
            else:
                continue
            ax.add_patch(patch)

    cylinders_coordinates = [((0.0, 13.0), "black")]
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)

    ax.set_xlim([-50, 50])  # x軸の範囲
    ax.set_ylim([-50, 50])  # y軸の範囲
    pyplot.show()

def draw_cylinders_at_coordinates(ax, coordinates_list):
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(xy=(x, y), radius=0.5, facecolor=color)
        ax.add_patch(cylinder_patch)

if __name__ == '__main__':
    draw_training_pos()

