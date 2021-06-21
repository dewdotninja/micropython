'''
lag3_ol_netpie.py
dew.ninja  June 2021
implement open-loop plant commanded from netpie
can read output from lag3 board
or set psim=1 for simulation on ESP32
'''

import machine
import time
from machine import Pin, PWM, ADC, DAC

from umqtt.robust import MQTTClient
import ujson
import network

online = 1  # set to 1 to connect with NETPIE

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""


MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""

PUBLISH_PERIOD = 2000  # milliseconds

# range variables
PWMMAX = 1023  # maximum PWM value
PWMMID = 511
PWMMIN = 0
DACMAX = 255
DACMID = 127
DACMIN = 0
ADCMAX = 4095
ADCMID = 2047
ADCMIN = 0

led = Pin(2, Pin.OUT)  # on-board led
rled = PWM(Pin(19))  # red led
gled = PWM(Pin(18))  # green led
bled = PWM(Pin(17))  # blue led

pwm_out = PWM(Pin(16))  # output to lag3 board
pwm_out.freq(5000) # set PWM frequency to 5 KHz

dac_out = DAC(Pin(25))

y_in = ADC(Pin(39)) # ADC1_CH3
y_in.atten(ADC.ATTN_11DB) # range 0 - 3.6 V

adc2v = 3.6/ADCMAX  # max of ADC is set to 3.6 v by the package
v2pwm = int(PWMMAX/3.3)
v2dac = int(DACMAX/3.3)
### ------- variables for plant and controller -----
r = 1  # reference cmd
r_old = 1  # previous value
y = 0  # plant output
y_adc = 0  # plant output in adc unit
u = 1  # controller output
u_pwm = u*v2pwm
sensor_data = {'r': r, 'y': y, 'u': u}
plantsim = 0 # plant simulation mode
datasize = 200  # number of points in output capture
capture = 0  # capture mode 0 = off, 1 = on
capture_flag = 0 

T = 0.05  # sampling period
T_ms = int(1000*T)
t_data = 0  # time data output
t_current = 0 
t_previous = 0
dt = 0
# variables for plant simulation 
a = 2+T
b = T-2
y_states = [0.0]*6
u_states = [0.0]*6
### -------------------------------------------------
cmdtime_current = 0  # This delay is needed for nodered 
cmdtime_prev = 0
CMD_DELAY = 1000

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

#command interpreter function
def cmdInt(userstr):
    global r,r_old,plantsim,datasize,capture, capture_flag
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
    if cmdstr.lower() == "r" or cmdstr.lower() == "step":
        # toggle_mode = 0  # turn off toggle mode
        
        if noparm==1: # perform unit step
            r_old = r
            r = 1
        else:
            r_old = r
            r = float(parmstr)
            if r > 3.0:  # limit range 
                r = 3.0
            elif r < 0.0: 
                r = 0.0
        if capture:  # capture mode on
            capture_flag = 1
            print("datamat = np.array([")  # head of data matrix
        update_freeboard()
    elif cmdstr.lower() == "psim":
        if noparm==1:
            print("Current plant simulation = {}".format(plantsim))
        else:            
            plantsim = int(parmstr)
            if plantsim > 1:
                plantsim = 1
            elif plantsim < 0:
                plantsim = 0
            update_freeboard()
    elif cmdstr.lower() == "capture":
        if noparm==1:
            print("Current capture mode = {}".format(capture))
        else:            
            capture = int(parmstr)
            if capture > 1:
                capture = 1
            elif capture < 0:
                capture = 0
            update_freeboard()
            
    elif cmdstr.lower() == "datasize":
        if noparm==1:
            print("Current datasize = {}".format(datasize))
        else:            
            datasize = int(parmstr)
            if datasize > 2000: # set maximum datasize
                datasize = 2000
            elif datasize < 10: # minimum datasize
                datasize = 10
            update_freeboard()

    else:
        print("Invalid command")

def update_freeboard():
    
    updatestr = "{},{},{},{}".format(r, plantsim, datasize, capture)
    #print(updatestr)
    client.publish('@msg/update', updatestr)    


# adjust RGB color according to plant output y
def y2rgb():
    global y
    if y <= 1.5:
        yt = y/1.5
        rval = 0
        bval = int(PWMMAX*(1-yt))
        gval = int(PWMMAX*yt)
    else:
        yt = (y-1.5)/1.5
        if yt>1:
            yt = 1
        bval = 0
        gval = int(PWMMAX*(1-yt))
        rval = int(PWMMAX*yt)
    lidRGBled(rval, gval, bval)

def lidRGBled(rval, gval, bval):
    rled.duty(rval)
    gled.duty(gval)
    bled.duty(bval)

# plant simulation
def lag3(a,b,T, u, u_states, y_states):
    for k in range(3):
        y_states[2*k] = y_states[2*k+1]
        u_states[2*k] = u_states[2*k+1]
        if k == 0:
            u_states[2*k+1] = u
        else:
            u_states[2*k+1] = y_states[2*k-1]
        y_states[2*k+1] = (1/a)*(-b*y_states[2*k]+T*(u_states[2*k+1]+u_states[2*k]))
    return y_states[5]

if online:
    wifi_connect()  # connect to WiFi network
    init_client()


# set publish period
time_prev = 0
time_current = 0

data_idx = 0  # data index
update_freeboard()

try:
    while True:
        t_current = time.ticks_ms()
        dt = t_current - t_previous
        if dt> T_ms : # executes only when sampling period passed
            
            t_previous = t_current
            if online:
                client.check_msg()
            # perform plant output read and plant simulation
            if plantsim==0:
                y_adc = y_in.read()  # read from ADC
                y = y_adc*adc2v  # convert to volt
            else:
                y = lag3(a,b,T,u, u_states, y_states) # in volt
            y2rgb()
            # -- write your controller here, or set u = r for open-loop
            u = r
            u_pwm = int(u*v2pwm)
            pwm_out.duty(u_pwm)  # send output via PWM pin 16
            u_dac = int(u*v2dac)
            dac_out.write(u_dac) # also send output to DAC1
            #print("r = {}, y = {}, u = {}".format(r,y,u))
            # if capture = 1, send output to shell
            if capture_flag:
                print("[{},{},{},{}],".format(round(t_data,2),u,y,u))
                t_data+=T
                data_idx+=1
                if data_idx==datasize:
                    data_idx = 0
                    t_data = 0
                    capture_flag = 0  # stop capture data 
                    print("])")  # end of np.array([ command
            
            if online:
                # write data to shadow
                time_current = time.ticks_ms()
                publish_delta = time_current - time_prev
                if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
                    led.value(not led.value()) # blink led
                    time_prev = time_current
                    sensor_data['r'] = r
                    sensor_data['y'] = y
                    sensor_data['u'] = u
                    publish_str = ujson.dumps({"data": sensor_data})
                    #print(publish_str)
                    client.publish("@shadow/data/update", publish_str)
            #time.sleep_ms(led_period)
        

except KeyboardInterrupt:
    pass

client.disconnect()

