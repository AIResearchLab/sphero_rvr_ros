#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Twist


# create a publisher for the command velocity
cmd_vel_pub = None

# create a test timer
test_timer = None

# create a illuminance difference container
starting_illuminance = 0.0
highest_test_illuminance = 0.0

# current test command
test_cmd = Twist()
selection_cmd = Twist()

# is testing
is_testing = True

# control update rate
control_rate = None

speed = 0.1
pub_times = 15


def publish_cmd_vel_n_times(cmd: Twist, n: int):
    global control_rate
    global cmd_vel_pub
    for i in range(n):
        cmd_vel_pub.publish(cmd)
        control_rate.sleep()

    for i in range(n):
        control_rate.sleep()


def run_test_command(cmd: Twist):
    # run a test command
    publish_cmd_vel_n_times(cmd, pub_times)

    # reverse the command
    reverse_cmd = Twist()
    reverse_cmd.linear.x = -cmd.linear.x
    reverse_cmd.angular.z = -cmd.angular.z
    publish_cmd_vel_n_times(reverse_cmd, pub_times)


def sniff_illuminance(event):
    # test three directions of illuminance using different twist commands
    rospy.loginfo("Testing illuminance...")

    # set is testing to true
    global is_testing
    is_testing = True

    global selection_cmd
    selection_cmd = Twist()

    global highest_test_illuminance
    highest_test_illuminance = 0.0

    # create a command velocity message
    global test_cmd
    test_cmd = Twist()

    # send a command velocity to the Sphero RVR
    test_cmd.linear.x = speed  # forward
    run_test_command(test_cmd)

    # send a command velocity to the Sphero RVR
    test_cmd.angular.z = 2.0  # turn right
    run_test_command(test_cmd)

    # send a command velocity to the Sphero RVR
    test_cmd.angular.z = -2.0  # turn left
    run_test_command(test_cmd)

    # go backwards
    test_cmd.linear.x = -speed
    test_cmd.angular.z = 0.0
    run_test_command(test_cmd)

    # send a command velocity to the Sphero RVR
    is_testing = False
    rospy.loginfo("Selected illuminance command: {}".format(selection_cmd))
    run_test_command(selection_cmd)

    # restart the test timer
    global test_timer
    test_timer.shutdown()
    test_timer = rospy.Timer(rospy.Duration(
        1.0), sniff_illuminance, oneshot=True)


def illuminance_cb(msg: Illuminance):
    # Read the illuminance data from the Sphero RVR
    light = msg.illuminance

    # get the starting light
    global starting_illuminance
    global is_testing
    if not is_testing:
        starting_illuminance = light
        rospy.loginfo("Starting illuminance: {}".format(starting_illuminance))
        return

    # compare the illuminance difference to the maximum illuminance difference
    global highest_test_illuminance
    if light > highest_test_illuminance:
        rospy.loginfo("New highest illuminance: {}".format(light))
        highest_test_illuminance = light
        # store the current test command to use for control
        global test_cmd
        global selection_cmd
        selection_cmd = test_cmd


if __name__ == '__main__':
    rospy.init_node("illuminance_follower_node")

    control_rate = rospy.Rate(10.0)

    # create a publisher for the command velocity
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # create a subscriber for illuminance data
    light_pub = rospy.Subscriber(
        "/rvr_driver/ambient_light", Illuminance, illuminance_cb, queue_size=1)

    # create a test timer
    test_timer = rospy.Timer(rospy.Duration(
        1.0), sniff_illuminance, oneshot=True)

    try:
        rospy.spin()

    except Exception as e:
        print(e)
