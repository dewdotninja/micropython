# multitask_2cores.py
# dew.ninja  Nov 2022
# micropython version of the original Arduino program
# Mar 2023:
#  - fix genchirp() by putting it in a while loop

import _thread
import math
import time
from time import sleep, sleep_ms
from machine import Pin, PWM

PWMMAX = 1023
PWMMID = 511

blink_period = 1000

rled = PWM(Pin(19))  # red led
gled = PWM(Pin(18))  # green led
bled = PWM(Pin(17))  # blue led

obled = Pin(2, Pin.OUT)  # on-board led
button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch

def genchirp():  # check button status and generate chirp signal if pressed
    gc_print_period = 1000
    gc_time_prev = 0
    stepnum = 10
    sinstep = 6.28/stepnum
    sinangle = 0
    dt = 30
    channel = 0 # change to R,G,B
    cyclecomplete = False
    if not button.value():
        sleep_ms(100)  # debounce
        while not button.value(): # blocked if switch pressed
            gc_time_current = time.ticks_ms()
            if (gc_time_current - gc_time_prev)>gc_print_period:
                print("genchirp() running")
                gc_time_prev = gc_time_current
            sinangle+=sinstep
            if sinangle>6.28:
                cyclecomplete = True
                sinangle = 0
            chirpout = int(PWMMID*0.5*(math.sin(sinangle)+1))
            if channel == 0:
                rled.duty(chirpout)
            elif channel == 1:
                gled.duty(chirpout)
            elif channel == 2:
                bled.duty(chirpout)
            if cyclecomplete:
                dt-=1
                cyclecomplete = False
            if dt<2:
                dt = 30
                lidRGBled(0,0,0)
                channel += 1  # change color
                if channel>2:
                    channel = 0
            sleep_ms(dt)

def genchirp_forever():  # run genchirp() continuously
    while True:
        genchirp()

def lidRGBled(rval, gval, bval):
    rled.duty(rval)
    gled.duty(gval)
    bled.duty(bval)

# start a new thread
_thread.start_new_thread(genchirp_forever,())


lidRGBled(0,0,0)

time_prev = 0

while True:
    time_current = time.ticks_ms()
    if (time_current - time_prev)>blink_period:
        obled.value(not obled.value())  # blink on-boardled
        print("while loop running ...")
        time_prev = time_current
        