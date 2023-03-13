import time
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import ColorRGBA
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish


# Define the MQTT topic and broker details
BROKER_HOSTNAME = "localhost"
LED_BLINK_TOPIC = "spheroRVR/led/blink"
LED_GREEN_TOPIC = "spheroRVR/led/green"
topic = "spheroRVR/speed"
hostname = BROKER_HOSTNAME
port = 1883

client = mqtt.Client("rover")

global blink_enabled
blink_enabled = 0
global green_enabled
green_enabled = 0


def speed_forward_cb(msg):
    # Read the speed data from the Sphero RVR
    speed = msg.data

    # Publish the speed data to the Mosquitto broker
    publish.single(topic, speed, hostname=hostname,
                   port=port, client_id="rover_speed_pub")
    print("published speed data: {}".format(speed))


# mqtt stuff
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")

        global Connected  # Use global variable
        Connected = True  # Signal connection

    else:
        print("Connection failed")


def on_message(client, userdata, message):
    print("mqtt message received! -> " + str(message.payload))

    # if blink message
    if message.topic == LED_BLINK_TOPIC:
        print("blinking leds")
        global blink_enabled
        blink_enabled = 1

    elif message.topic == LED_GREEN_TOPIC:
        print("setting green leds")
        global green_enabled
        green_enabled = 1


if __name__ == '__main__':
    rospy.init_node("rvr_fleet_logger")

    # create a sub for forwarding speed
    speed_sub = rospy.Subscriber(
        "/rvr_driver/speed", Float32, speed_forward_cb, queue_size=1)

    # create a publisher for led control
    led_pub = rospy.Publisher(
        "/rvr_driver/led/headlight", ColorRGBA, queue_size=1)

    # Set up the MQTT client
    client = mqtt.Client("rvr_fleet_logger")
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the MQTT broker
    client.connect(BROKER_HOSTNAME, 1883)
    client.loop_start()  # start the loop

    # subscribe to topics
    client.subscribe(LED_BLINK_TOPIC)
    client.subscribe(LED_GREEN_TOPIC)

    rate = rospy.Rate(10.0)

    try:
        if blink_enabled:
            # blink LEDS
            print("blink leds command activated!")

            rgb = ColorRGBA()
            rgb.a = 1.0
            rgb.b = 0.0

            for i in range(0, 6):
                rgb.r = 0.5
                rgb.g = 0.25
                led_pub.publish(rgb)
                time.sleep(0.2)

                rgb.r = 0.0
                rgb.g = 0.0
                led_pub.publish(rgb)
                time.sleep(0.2)

            blink_enabled = 0
            print("blink leds command done!")

        if green_enabled:
            # green LEDS
            print("green leds command activated!")
            rgb = ColorRGBA()
            rgb.r = 0.0
            rgb.g = 1.0
            rgb.b = 0.0
            rgb.a = 1.0

            led_pub.publish(rgb)
            green_enabled = 0

            print("green leds command done!")

        # rospy.spin()
        rate.sleep()

    except Exception as e:
        print(e)

    finally:
        # stop mqtt
        client.disconnect()
        client.loop_stop()
