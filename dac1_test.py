# dac1_test.py
# dew.ninja  June 2021
# needs lag3 board to measure output
# connect jumper on DAC1 side

from machine import Pin, ADC, DAC
from utime import sleep_ms
DACMAX = 255
adc = ADC(Pin(39)) # ADC1_CH3
adc.atten(ADC.ATTN_11DB) # range 0 - 3.6 V
dac1 = DAC(Pin(25))
dac1.write(int(DACMAX/4)) 
while True:
    adcval = adc.read()
    print(adcval)
    sleep_ms(1000)