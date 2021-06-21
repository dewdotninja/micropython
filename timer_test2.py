# timer_test2.py
# dew.ninja June 2021
# use timer to blink on-board led
# write callback function

import machine
from machine import Pin, Timer        #importing pin, and timer class
led= Pin(2, Pin.OUT)              # GPIO2 as led output

led.value(0)              #LED is off
timer1=Timer(1)

def timer_isr(event): 
    led.value(not led.value())

timer1.init(period=1000, mode=Timer.PERIODIC, callback=timer_isr)   #initializing the timer