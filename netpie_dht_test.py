# NETPIE_dht_test.py
# dew.ninja  June 2021
# basic upython program on NETPIE 2020
#import sys
import dht
import machine
from machine import Pin
from utime import sleep_ms

#import paho.mqtt.client as mqtt
from umqtt.simple import MQTTClient
import ujson
import network

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""


MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""
sensor_data = {'temperature': 0, 'humidity': 0}
d = dht.DHT22(machine.Pin(4))
led = Pin(2, Pin.OUT)

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
    #wifi.connect()
    try:
        client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, port=1883, user=MQTT_USER,
                            password=MQTT_PWD)
        #client.set_callback(callback)
        client.connect()
        print("Connected to ",MQTT_BROKER)
        #t = config.mqtt_topic + b"/#"
        #client.subscribe(t)
        #print("Subscribed to %s topic" % t)
    except:
        print("Trouble to init mqtt.") 


#def on_connect(client, userdata, flags, rc):
#    print("Result from connect: {}".format(
#        mqtt.connack_string(rc)))
    # client.subscribe("@shadow/data/updated")


wifi_connect()  # connect to WiFi network
init_client()

#client = mqtt.Client(protocol=mqtt.MQTTv311,
#                     client_id=CLIENT_ID, clean_session=True)
#client.username_pw_set(DEVICE_TOKEN)
#client.on_connect = on_connect
#client.on_message = on_message
#client.connect(NETPIE_HOST, 1883)
#client.loop_start()



try:
    while True:
        if led.value():  # toggle led
            led.off()
        else:
            led.on()
        d.measure()
        
        sensor_data['temperature'] = d.temperature()
        sensor_data['humidity'] = d.humidity()
        publish_str = ujson.dumps({"data": sensor_data})
        print(publish_str)
        client.publish("@shadow/data/update", publish_str)
        sleep_ms(2000)
        

except KeyboardInterrupt:
    pass

#client.loop_start()
client.disconnect()
