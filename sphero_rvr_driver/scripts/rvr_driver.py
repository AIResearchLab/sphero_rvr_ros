#!/usr/bin/env python3

"""
copyright rafftod
sourced from rafftod/rvr_ros on Github

modified to work ith navigation stack as a temperary python controller which will be replaced by a ros_control implementation

The purpose of this script is to make the robot
turn around, to test that UART can work properly for a longer
operating time when using the treads and sensors.

To stop the script, use Ctrl+C.
"""


# rvr
import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
from sphero_sdk import RvrLedGroups
from sphero_sdk import Colors

# ros
import rospy
import itertools
from typing import Dict, List, FrozenSet
from std_msgs.msg import Float32MultiArray, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu, Illuminance
import tf_conversions
from math import pi


# a global rvr object
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)


async def blink_leds():
    for i in range(0, 6):
        # headlight leds on
        await rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_left.value,
            led_brightness_values=[255, 124, 0]
        )
        await rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_right.value,
            led_brightness_values=[255, 124, 0]
        )

        await asyncio.sleep(0.2)

        # headlight leds off
        await rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_right.value,
            led_brightness_values=[0, 0, 0]
        )
        await rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_left.value,
            led_brightness_values=[0, 0, 0]
        )

        await asyncio.sleep(0.2)


class RVRDriver():
    # robot API sensor streaming interval (ms)
    SENSOR_STREAMING_INTERVAL: int = 2 * int(0.200 * 1000)
    # Wheel settings

    # speed in m/s
    speed: float = 0

    # LEDs settings
    ACTIVE_COLOR: Colors = Colors.white
    INACTIVE_COLOR: Colors = Colors.off
    BACK_COLOR: Colors = Colors.red

    def __init__(self) -> None:
        # get ros params
        # wheel base dimensions
        self.radius = rospy.get_param("wheel_radius", 0.05)
        self.separation = rospy.get_param("wheel_separation", 0.13)

        # Loop settings
        # main loop callback interval (seconds)
        self.loop_hz = rospy.get_param("~loop_hz", 10.0)
        self.loop_rate = rospy.Rate(self.loop_hz)

        # initial speed
        self.speed_params: Dict[str, float] = {
            "left_velocity": 0,
            "right_velocity": 0,
        }
        # initial LED settings
        self.led_settings: Dict[int, List[int]] = {
            # left headlight
            RvrLedGroups.headlight_left: self.ACTIVE_COLOR.value,
            # right headlight
            RvrLedGroups.headlight_right: self.ACTIVE_COLOR.value,
            # left side LED, left half
            RvrLedGroups.battery_door_front: self.INACTIVE_COLOR.value,
            # left side LED, right half
            RvrLedGroups.battery_door_rear: self.INACTIVE_COLOR.value,
            # right side LED, left half
            RvrLedGroups.power_button_front: self.INACTIVE_COLOR.value,
            # right side LED, right half
            RvrLedGroups.power_button_rear: self.INACTIVE_COLOR.value,
            # back LED
            RvrLedGroups.brakelight_left: self.BACK_COLOR.value,
            RvrLedGroups.brakelight_right: self.BACK_COLOR.value,
        }
        # sensor values
        # battery
        self.battery_percentage: float = 0
        self.latest_instruction: int = 0
        # accelerometer
        self.accelerometer_reading: Dict[str, float] = {"X": 0, "Y": 0, "Z": 0}
        # ground color sensor
        self.ground_color: Dict[str, int] = {"R": 0, "G": 0, "B": 0}
        # gyroscope
        self.angular_velocity: Dict[str, float] = {"X": 0, "Y": 0, "Z": 0}
        # IMU angles in degrees
        self.imu_reading: Dict[str, float] = {"Pitch": 0, "Roll": 0, "Yaw": 0}
        # light sensor
        self.ambient_light: float = 0
        # locator
        self.location: Dict[str, float] = {"X": 0, "Y": 0}
        # quaternion
        self.quat_reading: Dict[str, float] = {"W": 0, "X": 0, "Y": 0, "Z": 0}
        # velocity
        self.velocity_reading: Dict[str, float] = {"X": 0, "Y": 0}

        rospy.loginfo("setting things up...")
        rospy.loginfo("publishers")
        # Ground color as RGB
        self.ground_color_pub = rospy.Publisher(
            "~ground_color", ColorRGBA, queue_size=2
        )
        # IMU message includes :
        # - imu orientation
        # - gyroscope velocities
        # - linear acceleration
        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=2)
        # Ambient light
        self.light_pub = rospy.Publisher(
            "~ambient_light", Illuminance, queue_size=2
        )
        # Odometry message includes :
        # - Pose : position (locator) and orientation (quaternion)
        # - Twist : angular (gyro) and linear (velocity) velocities
        self.odom_pub = rospy.Publisher("~odom", Odometry, queue_size=2)

        rospy.loginfo("subscribers")
        # wheels speed subscriber
        self.wheel_sub = rospy.Subscriber(
            "~wheels/speed",
            Float32MultiArray,
            self.wheels_speed_callback,
            queue_size=1,
        )

        # cmd vel subscriber
        self.vel_sub = rospy.Subscriber(
            "~cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

        # rgb leds subscriber
        self.led_hl_sub = rospy.Subscriber("~led/headlight/left", ColorRGBA,
                                           self.headlight_left_led_cb, queue_size=1)

        self.led_hr_sub = rospy.Subscriber("~led/headlight/right", ColorRGBA,
                                           self.headlight_right_led_cb, queue_size=1)

        self.led_bl_sub = rospy.Subscriber("~led/brake/left", ColorRGBA,
                                           self.brake_left_led_cb, queue_size=1)

        self.led_br_sub = rospy.Subscriber("~led/brake/right", ColorRGBA,
                                           self.brake_right_led_cb, queue_size=1)

        # XXX TODO: add the other leds for the side panels

    async def rvr_loop(self):
        try:
            print("waking rvr")
            await rvr.wake()

            # Give RVR time to wake up
            await asyncio.sleep(2)
            print("rvr connected")

            # lights off
            await rvr.led_control.turn_leds_off()

            # lights blinking
            print("blink LEDS")
            await blink_leds()

            print("Enabling sensors...")
            rvr.enable_color_detection(is_enabled=True)
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.accelerometer,
                handler=self.accelerometer_handler,
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.color_detection,
                handler=self.ground_sensor_handler,
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.gyroscope, handler=self.gyroscope_handler
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.imu, handler=self.imu_handler
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.ambient_light, handler=self.light_handler
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.locator, handler=self.locator_handler
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.quaternion, handler=self.quaternion_handler
            )
            rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.velocity, handler=self.velocity_handler
            )

            print("sensor streaming on")
            await rvr.sensor_control.start(interval=250)

            await rvr.reset_yaw()

            # front lights green
            await rvr.set_all_leds(
                led_group=RvrLedGroups.headlight_left.value,
                led_brightness_values=[0, 255, 0]
            )
            await rvr.set_all_leds(
                led_group=RvrLedGroups.headlight_right.value,
                led_brightness_values=[0, 255, 0]
            )

            await asyncio.sleep(2)

            while not rospy.is_shutdown():

                # ambient light response on sides
                side_colour = Colors.white.value - int(
                    Colors.white.value * max((self.ambient_light / 1000.0), 1.0))
                self.led_settings[RvrLedGroups.battery_door_front] = side_colour
                self.led_settings[RvrLedGroups.battery_door_rear] = side_colour
                self.led_settings[RvrLedGroups.power_button_front] = side_colour
                self.led_settings[RvrLedGroups.power_button_rear] = side_colour

                # apply led values
                await rvr.led_control.set_multiple_leds_with_rgb(
                    leds=list(self.led_settings.keys()),
                    colors=list(itertools.chain(
                        *self.led_settings.values())),
                )

                # calculate the needed commands
                [dl, l, dr, r] = self.calculate_wheel_commands()

                # apply motor command values
                await rvr.raw_motors(
                    dl,
                    l,
                    dr,
                    r,
                    timeout=1.0 / self.loop_hz
                )

                # publish data
                self.publish_color()
                self.publish_imu()
                self.publish_light()
                self.publish_odom()

                # spin once
                self.loop_rate.sleep()
                # rospy.spin()

        except Exception as e:
            print(e)

        finally:
            await rvr.sensor_control.clear()
            await asyncio.sleep(0.5)
            await rvr.close()
            # Delay to allow RVR issue command before closing
            await asyncio.sleep(0.5)
            print("exitting")

    """ Robot Handlers """

    def battery_percentage_handler(self, bp: Dict[str, float]) -> None:
        self.battery_percentage = bp.get("percentage")

    def accelerometer_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.accelerometer_reading.update(
            (k, data["Accelerometer"][k])
            for k in self.accelerometer_reading.keys() & data["Accelerometer"].keys()
        )

    def ground_sensor_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.ground_color.update(
            (k, data["ColorDetection"][k])
            for k in self.ground_color.keys() & data["ColorDetection"].keys()
        )

    def gyroscope_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.angular_velocity.update(
            (k, data["Gyroscope"][k])
            for k in self.angular_velocity.keys() & data["Gyroscope"].keys()
        )

    def imu_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.imu_reading.update(
            (k, data["IMU"][k]) for k in self.imu_reading.keys() & data["IMU"].keys()
        )

    def light_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.ambient_light = data["AmbientLight"]["Light"]

    def locator_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.location.update(
            (k, data["Locator"][k])
            for k in self.location.keys() & data["Locator"].keys()
        )

    def quaternion_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.quat_reading.update(
            (k, data["Quaternion"][k])
            for k in self.quat_reading.keys() & data["Quaternion"].keys()
        )

    def velocity_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.velocity_reading.update(
            (k, data["Velocity"][k])
            for k in self.velocity_reading.keys() & data["Velocity"].keys()
        )

    ''' ros publishers '''

    def publish_color(self) -> None:
        """Sends the stored ground color as an RGBA ROS message."""
        color_msg = ColorRGBA()
        color_msg.r = self.ground_color["R"]
        color_msg.g = self.ground_color["G"]
        color_msg.b = self.ground_color["B"]
        color_msg.a = 255
        self.ground_color_pub.publish(color_msg)

    def publish_imu(self):
        """Sends the stored IMU data as an Imu ROS message.
        This message includes :
        - orientation (pitch, roll, yaw) from the IMU sensor
        - angular velocities (x, y, z) from the gyroscope
        - linear acceleration (x, y, z) from the accelerometer"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        # build quaterion by converting degrees to radians
        imu_msg.orientation = Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(
                self.imu_reading.get("Roll") * pi / 180,
                self.imu_reading.get("Pitch") * pi / 180,
                self.imu_reading.get("Yaw") * pi / 180,
            )
        )
        # convert degrees/sec to radians/sec
        imu_msg.angular_velocity = Vector3(
            **{k.lower(): v * pi / 180 for k, v in self.angular_velocity.items()}
        )
        # convert values from g to m/s^2
        imu_msg.linear_acceleration = Vector3(
            **{k.lower(): v * 9.81 for k, v in self.accelerometer_reading.items()}
        )
        self.imu_pub.publish(imu_msg)

    def publish_light(self):
        """Publishes the light illuminance in Lux."""
        light_msg = Illuminance()
        light_msg.header.stamp = rospy.Time.now()
        light_msg.illuminance = self.ambient_light
        self.light_pub.publish(light_msg)

    def publish_odom(self):
        """Publishes the odometry data as a ROS message.
        This message includes :
        - the current location of the robot in the map frame from the locator
        - the current orientation of the robot in the map frame from the quaternion
        - the current velocity of the robot in the map frame from the velocity sensor
        - the current angular velocity of the robot in the map frame from the gyroscope"""
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = self.location.get("X")
        odom_msg.pose.pose.position.y = self.location.get("Y")
        odom_msg.pose.pose.position.z = 0
        odom_quat = Quaternion(
            **{k.lower(): v for k, v in self.quat_reading.items()})
        odom_msg.pose.pose.orientation = odom_quat
        # convert degrees/sec to radians/sec
        odom_msg.twist.twist.angular = Vector3(
            **{k.lower(): v * pi / 180 for k, v in self.angular_velocity.items()}
        )
        odom_msg.twist.twist.linear = Vector3(
            **{k.lower(): v for k, v in self.velocity_reading.items()}
        )
        self.odom_pub.publish(odom_msg)

    ''' ros subscriber callbacks '''

    def wheels_speed_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback for wheels speed subscriber.
        """
        print(msg.data)
        self.speed_params["left_velocity"] = round(msg.data[0], 2)
        self.speed_params["right_velocity"] = round(msg.data[1], 2)

    def cmd_vel_cb(self, msg: Twist):
        # safety constraints
        x = self.constrain(
            msg.linear.x, -1.0, 1.0)
        yaw = self.constrain(
            msg.angular.z, -6.0, 6.0)

        # apply x velocity evenly
        # apply yaw on top with kinematic model
        self.speed_params["left_velocity"] = x - \
            (yaw * self.separation / 2.0) / self.radius

        self.speed_params["right_velocity"] = x + \
            (yaw * self.separation / 2.0) / self.radius

        # round
        self.speed_params["left_velocity"] = round(
            self.speed_params["left_velocity"], 2)

        self.speed_params["right_velocity"] = round(
            self.speed_params["right_velocity"], 2)

    # leds

    def headlight_left_led_cb(self, msg: ColorRGBA) -> None:
        # left headlight
        self.led_settings[RvrLedGroups.headlight_left] = [
            int(msg.r),
            int(msg.g),
            int(msg.b),
        ]

    def headlight_right_led_cb(self, msg: ColorRGBA) -> None:
        # right headlight
        self.led_settings[RvrLedGroups.headlight_right] = [
            int(msg.r),
            int(msg.g),
            int(msg.b),
        ]

    def brake_left_led_cb(self, msg: ColorRGBA) -> None:
        # back LED
        self.led_settings[RvrLedGroups.brakelight_left] = [
            int(msg.r),
            int(msg.g),
            int(msg.b),
        ]

    def brake_right_led_cb(self, msg: ColorRGBA) -> None:
        self.led_settings[RvrLedGroups.brakelight_right] = [
            int(msg.r),
            int(msg.g),
            int(msg.b),
        ]

    ''' helpers '''

    def calculate_wheel_commands(self):
        # get magnitude
        l = abs(self.speed_params["left_velocity"])
        r = abs(self.speed_params["right_velocity"])

        # get direction
        dl = 0
        dr = 0

        if l > 0:
            dl = int(self.speed_params["left_velocity"] / l)
            if dl < 0:
                dl = 2

        if r > 0:
            dr = int(self.speed_params["left_velocity"] / r)
            if dr < 0:
                dr = 2

        # scale values into motor space
        max_wheel_speed = 6.0
        l = int(255 * (l / max_wheel_speed))
        r = int(255 * (r / max_wheel_speed))

        # apply a deadzone
        deadzone = 25
        if l < deadzone and r < deadzone:
            l = 0
            dl = 0
            r = 0
            dr = 0

        return [dl, l, dr, r]

    def constrain(val, min_val, max_val):
        return min(max_val, max(min_val, val))


# main
if __name__ == "__main__":
    # init ROS node
    rospy.init_node("rvr_driver")

    rvr_driver = RVRDriver()

    # rvr loop
    loop.run_until_complete(
        asyncio.gather(
            rvr_driver.rvr_loop()
        )
    )

    print("exit rvr driver")
