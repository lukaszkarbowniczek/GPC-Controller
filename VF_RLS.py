import time
import csv
import sys
import RPi.GPIO as GPIO
import numpy as np
from w1thermsensor import W1ThermSensor

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
Heater = GPIO.PWM(12, 50)   # 1 PWM canal
Fan = GPIO.PWM(13, 50)  # 2 PWM canal
Sensor = W1ThermSensor.get_available_sensors()  # ID temperature sensors

na = 2  # number of estimated polynominal A parameters
nb = 1  # number of estimated polynominal B parameters 
Ts = 10     # sampling period
lam = 0.97  # forgetting factor
L = 300     # number of samples
P = 100000*np.eye(na+nb)    # covariance matrix
phi = np.zeros((na+nb, 1), np.float32)  # regression vector
theta = np.zeros((na+nb, 1), np.float32)    # parameters vector
Y = np.zeros((1, L), np.float32)    # output signal
U = np.zeros((1, L), np.float32)    # control signal
x1 = 50     # number of samples, after which fan speed is changing
x2 = 15     # number of samples, after which heater power is changing
e = 0   # error

def Temperature_Measurement():
    Air_Temp = Sensor[0].get_temperature()  # air temperature in room
    Heater_Temp = Sensor[1].get_temperature()   # air temperature at the end of transport canal
    Diff_Temp = np.float32(round((Heater_Temp - Air_Temp), 3))  # difference
    return Air_Temp, Diff_Temp

def Init():
    global Power    # heater power value [%]
    global Speed    # fan speed value [%]
    Power = np.float32(27)
    Speed = np.float32(40)
    Heater.start(Power)
    Fan.start(Speed)

def Heater_Cooling():
    Heater.ChangeDutyCycle(0)
    Fan.ChangeDutyCycle(100)
    while True:
        [Air_Temp, Diff_Temp] = Temperature_Measurement()
        if Diff_Temp > 0.2:
            [Air_Temp, Diff_Temp] = Temperature_Measurement()
            print("Cooling...\n", Diff_Temp)
            time.sleep(5)
        else:
            GPIO.cleanup()
            sys.exit()

try:
    Init()
    print(" Fan:", Speed, " Ts:", Ts, " Lambda:", lam)
    for t in range(0, L):
        start = time.time()
        [Air_Temp, Diff_Temp] = Temperature_Measurement()
        Y[0, t] = Diff_Temp
        U[0, t] = Power
        # Change of fan speed
        if (t == x1):
            Speed = 60
            Fan.ChangeDutyCycle(Speed)
        if (t == 2*x1):
            Speed = 20
            Fan.ChangeDutyCycle(Speed)
        if (t == 3*x1):
            Speed = 80
            Fan.ChangeDutyCycle(Speed)
        if (t == 4*x1):
            Speed = 40
            Fan.ChangeDutyCycle(Speed)
        if (t == 5*x1):
            Speed = 100
            Fan.ChangeDutyCycle(Speed)
        if (t == x2):
            x2 = np.int(np.random.rand(1)[0] * 20 + 5)
            x2 = t + x2
            # New value of heater power
            Power = np.round(np.float32(np.random.rand(1)[0] * 60 + 10), 1)
            Heater.ChangeDutyCycle(Power)
        U[0, t] = Power
        # VF-RLS algorithm
        while t > na:
            phi = np.array([[-Y[0, t-1]], [-Y[0, t-2]],  [U[0, t-1]]])
            e = Y[0, t] - phi.T @ theta
            L = P @ phi / (lam + phi.T @ P @ phi)
            P = (P - L @ phi.T @ P) / lam
            theta = theta + L * e
            break
        # Saving to .csv
        with open('Online_RLS_097_2.csv', 'a') as csvFile:
            pomiary = csv.writer(csvFile)
            pomiary.writerow([t, U[0, t], Y[0, t], theta[0], theta[1], theta[2], e,  Air_Temp, Speed, Ts, lam])
            csvFile.close()
        print("t:", t, " U(t):", U[0, t], " Y(t):", Y[0, t], " Speed(t):", Speed,  " a1:", theta[0], " a2:", theta[1], " b0:", theta[2])
        end = time.time()
        time.sleep(Ts - end + start)    # uniform sampling period
    Heater_Cooling()

except(KeyboardInterrupt):
    Heater_Cooling()
