#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import ColorRGBA
import tf
from tf import TransformListener


# create a publisher for the color
color_pub = None


# main
if __name__ == '__main__':
    rospy.init_node('fake_publish_color')

    # create a publisher for the color
    color_pub = rospy.Publisher(
        '/rvr_driver/ground_color', ColorRGBA, queue_size=10)

    # create a transform listener
    tf_listener = TransformListener()

    # create color publisher rate
    color_rate = rospy.Rate(10.0)

    # publish color
    while not rospy.is_shutdown():
        # create color message
        color_msg = ColorRGBA()
        # color_msg.header.frame_id = 'color_source'
        # color_msg.header.stamp = rospy.Time.now()

        # get the map position of the color sensor
        try:
            (color_position, color_orientation) = tf_listener.lookupTransform(
                '/color_source', '/base_link', rospy.Time(0))

            # change color if the base_link is within 1 meter of the color sensor
            if math.sqrt(color_position[0] ** 2 + color_position[1] ** 2) < 1.0:
                color_msg.r = 255
                color_msg.g = 0
                color_msg.b = 0
                color_msg.a = 255
            else:
                color_msg.r = 150
                color_msg.g = 150
                color_msg.b = 150
                color_msg.a = 255

            # publish color
            color_pub.publish(color_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get color position")

        # sleep
        color_rate.sleep()
