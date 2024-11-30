#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    # ノードの初期化
    rospy.init_node('tf_listener_node')

    # TF2リスナーの作成
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # リスンするTFのフレームを指定
            trans = tf_buffer.lookup_transform('target_frame', 'source_frame', rospy.Time())
            # 取得したトランスフォーメーションの出力
            rospy.loginfo(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF Exception")
            pass

        rate.sleep()

if __name__ == '__main__':
    main()

