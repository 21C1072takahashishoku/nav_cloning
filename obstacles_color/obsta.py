#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
from matplotlib import pyplot
from matplotlib.patches import Circle
import numpy as np
from PIL import Image

def draw_cylinders_at_coordinates(ax, coordinates_list):
    # 座標リストの各座標に赤色と青色の円柱を描画する関数
    for (x, y), color in coordinates_list:
        cylinder_patch = Circle(xy=(x, y), radius=0.1, facecolor=color)
        ax.add_patch(cylinder_patch)

if __name__ == '__main__':
    rospy.init_node('draw_cylinder_node', anonymous=True)
    path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/analysis/'
    image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/map.png').convert("L")
    arr = np.asarray(image)
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    ax.imshow(arr, cmap='gray', extent=[-10,50,-10,50])

    # 複数の円柱を表示させたい座標と色のリストを指定します
    #成功は赤、失敗は青
    cylinders_coordinates = [
        ((15.2805442160836,9.10878733981172), "red"),
        ((19.7819192821497,9.5698728461363), "red"),
        ((17.7682052335177,6.8786383403068), "red"),
        ((17.8317236308661,10.4181875852569), "red"),
        ((18.6987364527239,7.6611658730348), "red"),
        ((19.0113063848277,6.85680928294236), "red"),
        ((19.3783374841596,9.55633062947862), "red"),
        ((20.2124464344608,7.74062005745955), "red"),
        ((16.2225577756082,6.30950916344294), "red"),
        ((20.3116519142054,10.4931973176242), "red"),
        ((20.40890585709,6.20889191166881), "red"),
        ((18.3310409442262,10.2539511781257), "red"),
        ((15.0416852335071,5.7807998556447), "red"),
        ((19.2209389984518,9.08104334237307), "red"),
        ((17.0538288706621,6.10033571644436), "red"),
        ((18.024768474878,6.21168798425828), "red"),
        ((17.6597993749429,6.65260080950618), "red"),
        ((19.2411341432687,5.16737328646583), "red"),

        ((20.8689345948812,6.15122516584461), "blue"),
        ((15.4353644604049,10.1296991491237), "blue"),
        ((19.4673286782958,8.34830058344164), "blue"),
        ((15.8871373686339,8.92768547202551), "blue"),
        ((20.2107059131491,6.95784351966187), "blue"),
        ((15.1313693263532,9.42265347509006), "blue"),
        ((15.4630916208508,8.96064414227716), "blue"),
        ((19.8988517981066,5.36968787934432), "blue"),
        ((17.3279100836006,10.1592844170902), "blue"),
        ((19.8756661430041,8.70987182079573), "blue"),
        ((16.5933656656287,6.79178005341489), "blue"),
        ((19.8998608232911,8.49763289347633), "blue"),



    ]

    # 赤色と青色の円柱を描画する関数を呼び出します
    draw_cylinders_at_coordinates(ax, cylinders_coordinates)

    ax.set_xlim([-5, 30])
    ax.set_ylim([-5, 15])
    pyplot.show()


