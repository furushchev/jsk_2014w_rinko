#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PointStamped
from jsk_rviz_plugins.msg import OverlayText

prev_msg = None
pub = None
r = None

send_msg = OverlayText()
send_msg.text = "*"
send_msg.width = 140
send_msg.height = 200
send_msg.text_size = 100
send_msg.line_width = 2
send_msg.font = "DejaVu Sans Mono"
send_msg.bg_color = ColorRGBA(0,0,0,0)
send_msg.fg_color = ColorRGBA(255, 0, 0, 1)


def gaze_callback(msg):
    # global prev_msg
    x = msg.data[0]
    y = msg.data[1]
    send_msg.left = x
    send_msg.top = y
    pub.publish(send_msg)
    # if prev_msg:
    #     prev_msg.action = 1
    #     pub.publish(send_msg)
    # prev_msg = send_msg
    # r.sleep()

if __name__ == '__main__':
    global pub, r
    rospy.init_node('gaze_visualizer_rviz')
    pub = rospy.Publisher('/gaze_overlay', OverlayText)
    sub = rospy.Subscriber('/gaze', PointStamped, gaze_callback)
    r = rospy.Rate(30)
    rospy.spin()
