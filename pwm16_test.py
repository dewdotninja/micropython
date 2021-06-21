# pwm16_test.py
# dew.ninja  June 2021
# needs lag3 board to measure output
# connect jumper on PWM side

from machine import Pin, ADC, PWM
from utime import sleep_ms

PWMMAX = 1023
adc = ADC(Pin(39)) # ADC1_CH3
adc.atten(ADC.ATTN_11DB) # range 0 - 3.6 V
pwm16 = PWM(Pin(16))
pwm16.freq(5000)
pwmval = int(PWMMAX/2)
pwm16.duty(pwmval)
while True:
    adcval = adc.read()
    print(adcval)
    sleep_ms(1000)