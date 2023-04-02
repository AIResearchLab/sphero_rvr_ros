#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import Illuminance
import tf
from tf import TransformListener


# create a publisher for the illuminance
illuminance_pub = None


# main
if __name__ == '__main__':
    rospy.init_node('fake_publish_illuminance')

    # create a publisher for the illuminance
    illuminance_pub = rospy.Publisher(
        '/rvr_driver/ambient_light', Illuminance, queue_size=10)

    # create a transform listener
    tf_listener = TransformListener()

    # create illuminance publisher rate
    illuminance_rate = rospy.Rate(10.0)

    # publish illuminance
    while not rospy.is_shutdown():
        # create illuminance message
        illuminance_msg = Illuminance()
        illuminance_msg.header.frame_id = 'illuminance_source'
        illuminance_msg.header.stamp = rospy.Time.now()

        # get the map position of the illuminance sensor
        try:
            (illuminance_position, illuminance_orientation) = tf_listener.lookupTransform(
                '/illuminance_source', '/base_link', rospy.Time(0))

            # calculate illuminance based on distance from map origin
            illuminance_msg.illuminance = 1000.0 / \
                math.sqrt(illuminance_position[0] ** 2 +
                          illuminance_position[1] ** 2)

            # publish illuminance
            illuminance_pub.publish(illuminance_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get illuminance position")

        # sleep
        illuminance_rate.sleep()
