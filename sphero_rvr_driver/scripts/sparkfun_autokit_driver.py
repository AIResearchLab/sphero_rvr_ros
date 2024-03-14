#!/usr/bin/env python3

"""
sparkfun rvr autonomus kit ROS driver

controls the sparkfun rvr autonomus kit including:
- servos
- gps
- distance sensor
"""

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
import qwiic_titan_gps
import qwiic_vl53l1x


class RVRAutoKitDriver:
    def __init__(self):
        # get the parameters
        self.loop_rate = rospy.get_param('~loop_rate', 4.0)

        # get auto kit resources
        self.qwiic_gps = qwiic_titan_gps.QwiicTitanGps()
        self.qwiic_vl53l1x = qwiic_vl53l1x.QwiicVL53L1X()

        # ros data variables
        self.laser_scan = LaserScan()
        self.nav_sat_fix = NavSatFix()
        self.time_reference = TimeReference()

        # check resources operational
        if self.qwiic_gps.connected is False:
            rospy.logerr("Qwiic Titan GPS not connected")

        if not self.qwiic_vl53l1x.sensor_init():
            rospy.logerr("Qwiic VL53L1X not connected")

        # set up the publishers
        self.laser_scan_pub = rospy.Publisher(
            '~scan', LaserScan, queue_size=1)
        self.nav_sat_fix_pub = rospy.Publisher('~gps', NavSatFix, queue_size=1)
        self.time_reference_pub = rospy.Publisher(
            '~time_ref', TimeReference, queue_size=1)

        # start auto kit resources
        self.qwiic_gps.begin()

        # create timer to check sensor data
        self.timer = rospy.Timer(rospy.Duration(
            1.0 / self.loop_rate), self.check_sensor_data)

    def check_sensor_data(self, event):
        """
        check the sensor data and publish it
        """

        # get distance sensor data
        self.qwiic_vl53l1x.start_ranging()
        rospy.Rate(1.0 / 0.005).sleep()
        distance = self.qwiic_vl53l1x.get_distance() / 1000.0
        rospy.Rate(1.0 / 0.005).sleep()
        self.qwiic_vl53l1x.stop_ranging()

        # get laser scan data
        self.laser_scan.header.frame_id = 'laser'
        self.laser_scan.header.stamp = rospy.Time.now()
        self.laser_scan.angle_min = 0.0
        self.laser_scan.angle_max = 0.0
        self.laser_scan.angle_increment = 0.0
        self.laser_scan.time_increment = 0.0
        self.laser_scan.scan_time = 0.0
        self.laser_scan.range_min = 0.0
        self.laser_scan.range_max = 0.0
        self.laser_scan.ranges = [distance]
        self.laser_scan.intensities = [1.0]

        # get gps data
        if self.qwiic_gps.get_nmea_data() is True:
            self.nav_sat_fix.header.frame_id = 'gps'
            self.nav_sat_fix.header.stamp.secs = self.qwiic_gps['Time']
            self.nav_sat_fix.header.stamp.nsecs = self.qwiic_gps['Time'] * 1e9
            self.nav_sat_fix.latitude = self.qwiic_gps['Latitude']
            self.nav_sat_fix.longitude = self.qwiic_gps['Longitude']
            self.nav_sat_fix.altitude = self.qwiic_gps['Altitude']
            self.nav_sat_fix.position_covariance = [0.0] * 9
            self.nav_sat_fix.position_covariance_type = 0

            # time reference
            self.time_reference.header.frame_id = 'gps'
            self.time_reference.header.stamp = rospy.Time.now()
            self.time_reference.time_ref = self.nav_sat_fix.header.stamp

        # publish the data
        self.laser_scan_pub.publish(self.laser_scan)
        self.nav_sat_fix_pub.publish(self.nav_sat_fix)
        self.time_reference_pub.publish(self.time_reference)


# main
if __name__ == '__main__':
    # initialize the node
    rospy.init_node('rvr_autokit_driver')

    # create the driver
    driver = RVRAutoKitDriver()

    # spin
    rospy.spin()
