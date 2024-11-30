#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
from matplotlib import pyplot
from matplotlib.patches import Circle, Rectangle  # Rectangleをインポート
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

    # 青い円柱を描画
    
    cylinders_coordinates = [
        ((18.5, 8.5), "black"),
    ]
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)
    
    # 赤と青の囲みを描画
    rect_red = Rectangle((16.5, 6.5), 4, 4, linewidth=2, edgecolor='red', facecolor='none')
    rect_blue = Rectangle((17.5, 7.5), 2, 2, linewidth=2, edgecolor='blue', facecolor='none')
    rect_grey = Rectangle((15.5, 5.5), 6, 6, linewidth=2, edgecolor='grey', facecolor='none')
<<<<<<< HEAD
    #rect_green = Rectangle((14.5, 4.5), 8, 8, linewidth=2, edgecolor='green', facecolor='none')
=======
    rect_green = Rectangle((14.5, 4.5), 8, 8, linewidth=2, edgecolor='green', facecolor='none')
>>>>>>> 7708ec1d40aa1fa7033accb2c6659e4537e1d56a

    ax.add_patch(rect_red)
    ax.add_patch(rect_blue)
    ax.add_patch(rect_grey)
<<<<<<< HEAD
    #ax.add_patch(rect_green)
=======
    ax.add_patch(rect_green)
>>>>>>> 7708ec1d40aa1fa7033accb2c6659e4537e1d56a

    ax.set_xlim([-5, 30])
    ax.set_ylim([-5, 15])
    pyplot.show()

def draw_cylinders_at_coordinates(ax, coordinates_list):
    # 青い円柱を描画する関数
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(xy=(x, y), radius=0.5, facecolor=color)
        ax.add_patch(cylinder_patch)

if __name__ == '__main__':
    draw_training_pos()

