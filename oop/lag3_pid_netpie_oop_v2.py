'''
lag3_pid_netpie_oop.py
dew.ninja  october 2022

Version 2 :
- PID is implemented as class in pid.py
- NETPIE is implemented as class in netpie.py

implement PID control with lag3 plant
commanded from netpie
can read output from lag3 board
or set psim=1 for simulation on ESP32
'''

import machine
import time
from machine import Pin, PWM, ADC, DAC

from umqtt.robust import MQTTClient
import ujson
import network
from pid import PID
from netpie import NETPIE

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
UMAX = 3.2  # limit max controller output to 3.2 vols
UMID = 1.65
UMIN = 0.1 # min controller output

led = Pin(5, Pin.OUT)  # on-board led
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
ulim = 1  # limited controller output
u_pwm = u*v2pwm
sensor_data = {'r': r, 'y': y, 'u': u}
plantsim = 0 # plant simulation mode
datasize = 200  # number of points in output capture
capture = 0  # capture mode 0 = off, 1 = on
capture_flag = 0

# ---- controller -----
feedback = 1  # 0 = open-loop, 1 = close-loop
# This parameter set is tuned by Ziegler-Nichols method
kp = 4.8  # PID gains
ki = 2.74
kd = 2.1
kt = 0  # back calculaiton gain
wp = 1  # proportional weight
wd = 1  # derivative weight
N = 50  # derivative filter coefficient

T = 0.05  # sampling period
T_ms = int(1000*T)

# create PID instance
pid = PID(kp,ki,kd,kt,N,wp,wd,T)
pid.setulim(UMAX)
pid.enableulim()  
pid.enableoffset()


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
# cmdtime_current = 0  # This delay is needed for nodered 
# cmdtime_prev = 0
# CMD_DELAY = 1000

#command interpreter function
def cmdInt(userstr):
    global r,r_old,plantsim,datasize,capture,capture_flag,feedback,kp,ki,kd,kt,wp,wd,N
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
    elif cmdstr.lower() == "feedback":
        if noparm==1:
            print("Current feedback mode = {}".format(feedback))
        else:            
            feedback = int(parmstr)
            if feedback > 1:
                feedback = 1
            elif feedback < 0:
                feedback = 0
            update_freeboard()

    elif cmdstr.lower() == "kp":
        if noparm==1:
            print("Current kp = {}".format(pid.getkp()))
        else:            
            kp = float(parmstr)
            if kp > 100: # set maximum kp
                kp = 100
            elif kp < 0: # minimum kp
                kp = 0
            pid.setkp(kp)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "ki":
        if noparm==1:
            print("Current ki = {}".format(pid.getki()))
        else:            
            ki = float(parmstr)
            if ki > 100: # set maximum ki
                ki = 100
            elif ki < 0: # minimum ki
                ki = 0
            pid.setki(ki)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "kd":
        if noparm==1:
            print("Current kd = {}".format(pid.getkd()))
        else:            
            kd = float(parmstr)
            if kd > 100: # set maximum kd
                kd = 100
            elif kd < 0: # minimum kd
                kd = 0
            pid.setkd(kd)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "kt":
        if noparm==1:
            print("Current kt = {}".format(pid.getkt()))
        else:            
            kt = float(parmstr)
            if kt > 100: # set maximum kt
                kt = 100
            elif kt < 0: # minimum kt
                kt = 0
            pid.setkt(kt)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "wp":
        if noparm==1:
            print("Current wp = {}".format(pid.getwp()))
        else:            
            wp = float(parmstr)
            if wp > 10: # set maximum wp
                wp = 10
            elif wp < 0: # minimum wp
                wp = 0
            pid.setwp(wp)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "wd":
        if noparm==1:
            print("Current wd = {}".format(pid.getwd()))
        else:            
            wd = float(parmstr)
            if wd > 10: # set maximum wd
                wd = 10
            elif wd < 0: # minimum wd
                wd = 0
            pid.setwd(wd)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    elif cmdstr.lower() == "n":
        if noparm==1:
            print("Current N = {}".format(pid.getn()))
        else:            
            N = float(parmstr)
            if N > 200: # set maximum N
                N = 200
            elif N < 2: # minimum N
                N = 2
            pid.setn(N)
            pid.update()
            pid.reset() # reset controller states
            update_freeboard()

    else:
        print("Invalid command")

def update_freeboard():
    if online:
        updatestr = "{},{},{},{},{},{},{},{},{},{},{},{}".format(r, plantsim, datasize,
            capture, feedback, kp,ki,kd,kt,wp,wd,N)
        #print(updatestr)
        netpie.client.publish('@msg/update', updatestr)    


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
    #wifi_connect()  # connect to WiFi network
    #init_client()
    # create NETPIE instance
    netpie = NETPIE(wifi_ssid,wifi_pwd,MQTT_CLIENT, MQTT_USER, MQTT_PWD)


# set publish period
time_prev = 0
time_current = 0

data_idx = 0  # data index
#PID_update()

try:
    while True:
        t_current = time.ticks_ms()
        dt = t_current - t_previous
        if dt> T_ms : # executes only when sampling period passed
            
            t_previous = t_current
            if online:
                netpie.client.check_msg()
                if netpie.cmd_received:
                    cmdInt(netpie.retrieve_cmd())  
            # perform plant output read and plant simulation
            if plantsim==0:
                y_adc = y_in.read()  # read from ADC
                y = y_adc*adc2v  # convert to volt
            else:
                y = lag3(a,b,T,u, u_states, y_states) # simulate the output from lag3()
            y2rgb()
            # implement your controller here, or set u = r for open-loop
            if feedback:
                u = pid.out(r,y) # call pid object
            else:  # open-loop
                u = r
            ulim = u
            u_pwm = int(ulim*v2pwm)
            pwm_out.duty(u_pwm)  # send output via PWM pin 16
            u_dac = int(ulim*v2dac)
            dac_out.write(u_dac) # also send output to DAC1
            #print("r = {}, y = {}, u = {}".format(r,y,u))
            # if capture = 1, send output to shell
            if capture_flag:
                print("[{},{},{},{},{}],".format(round(t_data,2),r,y,u,ulim))
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
                    netpie.client.publish("@shadow/data/update", publish_str)
            #time.sleep_ms(led_period)
        

except KeyboardInterrupt:
    pass

if online:
    netpie.client.disconnect()

