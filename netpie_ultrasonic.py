# netpie_ultrasonic.py
# dew.ninja  June 2021
# measure distance and alert user when
# distance is less than specified value

import machine
import time
from machine import Pin, Timer

#import paho.mqtt.client as mqtt
from umqtt.robust import MQTTClient
import ujson
import network
from hc_sr04 import HCSR04

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""


MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""

PUBLISH_PERIOD = 2000  # milliseconds
led = Pin(2, Pin.OUT)
alert_status = 0  # 0 = off, 1 = on
mindist = 20  # initial minimum distance
distance = 0  # actual distance

sensor = HCSR04(trigger_pin=4, echo_pin=15,echo_timeout_us=1000000) # initialize ultrasonic sensor
sensor_data = {'min_distance': 0, 'distance': 0, 'alert': 0}

cmdtime_current = 0  # This delay is needed for nodered 
cmdtime_prev = 0
CMD_DELAY = 1000 

def hcrs04_isr(event):
    global distance
    distance = sensor.distance_cm()
    #print(round(distance,2))
    
blink_timer = Timer(1)
blink_timer.init(period=250, mode=Timer.PERIODIC, callback=hcrs04_isr)


def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_ssid, wifi_pwd)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

def init_client():
    global client

    print("Trying to connect to mqtt broker.")

    try:
        client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, port=1883, user=MQTT_USER,
                            password=MQTT_PWD)
        #client.set_callback(callback)
        client.connect()
        print("Connected to ",MQTT_BROKER)
        topic_sub = b"@msg/cmd"
        print("Subscribed to ",topic_sub)
        client.set_callback(sub_cb)
        client.subscribe(topic_sub)
        
    except:
        print("Trouble to init mqtt.") 

def sub_cb(topic, msg):
    global cmdtime_current, cmdtime_prev
    print((topic, msg))
    if topic == b'@msg/cmd':
        rcvdstrs = str(msg).split("'") # get rid of b'
        rcvdstr = rcvdstrs[1]
        # this delay is needed for nodered implementation
        cmdtime_current = time.ticks_ms()
        delta_cmdtime = cmdtime_current - cmdtime_prev
        if delta_cmdtime > CMD_DELAY:
            cmdtime_prev = cmdtime_current
            cmdInt(rcvdstr)
    #print(topic)

#command interpreter function
def cmdInt(userstr):
    global mindist
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    #print(cmdstr)
    #print(parmstr)
    if cmdstr.lower() == "mindist":
        # set minimum distance 
        
        if noparm==1:
            print("Current minimum distance = {} cm".format(mindist))
        else:            
            mindist = float(parmstr)
            if mindist > 100.0:    # limit range 
                mindist = 100.0
            elif mindist < 10.0:
                mindist = 10.0

    else:
        print("Invalid command")    


wifi_connect()  # connect to WiFi network
init_client()


# set publish period
time_prev = 0
time_current = 0


try:
    while True:
        client.check_msg()
        # actually, for this program the shadow data is not used,
        # since data is sent to freeboard directly using @msg/update
        # this code is reserved in case you want to write something to shadow
        time_current = time.ticks_ms()
        publish_delta = time_current - time_prev
        if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
            time_prev = time_current
            
            if distance < mindist:
                alert_status = 1
                led.on()
            else:
                alert_status = 0
                led.off()
            
            sensor_data['min_distance'] = mindist
            sensor_data['distance'] = distance
            sensor_data['alert'] = alert_status
            publish_str = ujson.dumps({"data": sensor_data})
            print(publish_str)
            client.publish("@shadow/data/update", publish_str)
        #time.sleep_ms(led_period)
        

except KeyboardInterrupt:
    pass

client.disconnect()
