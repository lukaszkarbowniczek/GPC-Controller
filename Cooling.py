import numpy as np
import time
import sys
import RPi.GPIO as GPIO
from Shutdown import Shutdown
from w1thermsensor import W1ThermSensor

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
Heater = GPIO.PWM(12, 50)   # 1 PWM canal
Fan = GPIO.PWM(13, 50)  # 2 PWM canal
Sensor = W1ThermSensor.get_available_sensors()

def Temperature_Measurement():
    Air_Temp = Sensor[0].get_temperature()  
    Heater_Temp = Sensor[1].get_temperature()
    Diff_Temp = np.float32(np.round((Heater_Temp - Air_Temp), 3))
    return Air_Temp, Diff_Temp

# Cooling heater before turning controller off
def Heater_Cooling():
    Heater.start(0)
    Fan.start(60)
    while True:
        [Air_Temp, Diff_Temp] = Temperature_Measurement()
        if Diff_Temp > 3:
            [Air_Temp, Diff_Temp] = Temperature_Measurement()
            print("Cooling...\n", Diff_Temp)
            time.sleep(5)
        else:
            GPIO.cleanup()
            Shutdown()

try:
    Heater_Cooling()

except:
    Heater_Cooling()
