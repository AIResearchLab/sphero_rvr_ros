#!/usr/bin/env python3

import random
import rospy
from tf import TransformBroadcaster


# randomly assign a new location for the light source
def randomize_light_source_location(seed: int):
    # randomly assign a new location for the light source
    # must be higher than the ground plane
    z = 1.2

    # must be within the map bounds
    map_bounds = [-2, 4, -2, 4]

    # create a random seed
    random.seed(seed)

    # vector translation
    x = random.uniform(map_bounds[0], map_bounds[1])
    y = random.uniform(map_bounds[2], map_bounds[3])

    return (x, y, z)


# main
if __name__ == '__main__':
    # init ros node
    rospy.init_node('static_light_source_tf')

    # create a transform broadcaster
    tf_broadcaster = TransformBroadcaster()

    # create a transform broadcaster rate
    tf_rate = rospy.Rate(100.0)

    # randomly assign a new location for the light source
    (x, y, z) = randomize_light_source_location(0)
    # log the location
    rospy.loginfo("Light source location: (%f, %f, %f)", x, y, z)

    # publish transform
    while not rospy.is_shutdown():
        # publish transform
        tf_broadcaster.sendTransform(
            (x, y, z),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            'illuminance_source',
            'map'
        )

        # sleep
        tf_rate.sleep()
