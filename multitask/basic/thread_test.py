# thread_test.py
# dew.ninja  Oct 2022
# run example from https://youtu.be/QeDnjcdGrpY

import _thread
from time import sleep, sleep_ms
from machine import Pin

led = Pin(2, Pin.OUT)
button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch

def blink_led():
    # blink the built-in LED on and then off
    led.value(0)
    sleep(1)
    led.value(1)
    sleep(1)
    print("Hello from thread")
    
def blink_forever():
    while True:
        blink_led()

# start a new thread
_thread.start_new_thread(blink_forever,())

# do somethine else
while True:
    print("Hello from main")
    sleep(2)
    if not button.value():
        sleep_ms(100)  # debounce
        print("Main thread stopped!")
        while not button.value(): # blocked if switch pressed
            sleep(2)
