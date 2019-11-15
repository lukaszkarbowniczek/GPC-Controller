import numpy as np
import time
import csv
import sys
from numpy.linalg import inv
from Reference import Gentle_Reference
from Reference import Rectangle_Reference
from Speed import Gentle_Speed
from Speed import Rectangle_Speed
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from w1thermsensor import W1ThermSensor

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
Heater = GPIO.PWM(12, 50)    # 1 PWM canal
Fan = GPIO.PWM(13, 50)    # 2 PWM canal
Sensor = W1ThermSensor.get_available_sensors()  # ID of temperature sensors

a = np.array([1, -1.7500, 0.7599], np.float32)  # polynominal A parameters
b = np.float32(0.0036)  # plynominal B parameters
na = 2  # degree of A
H = 20  # prediction/control horizon
L = 300     # number of samples
Ts = 10     # sampling period

h = np.zeros((1, H), np.float32)    # elements of matrix Q
Q = np.zeros((2*H, 2*H), np.float32)    # matrix Q
Y = np.zeros((1, L+H), np.float32)  # output signal
U = np.zeros((1, L+H), np.float32)  # control signal
w = np.zeros((L+H, 1), np.float32)  # reference trajectory
# w = Gentle_Reference(L,H)
w = Rectangle_Reference(L,H)
V = Gentle_Speed(L,H)
# V = Rectangle_Speed(L,H)
f = np.zeros((1, H), np.float32)    # free plant response
Umin = 0    # minimal value of control signal
Umax = 90   # maximal value of control signal
ro = 0.7   # współczynnik wagowy przyrostu sterowania 

def Temperature_Measurement():
    Air_Temp = Sensor[0].get_temperature()  # air temperature in room
    Heater_Temp = Sensor[1].get_temperature()   # air temperature at the end of transport canal
    Diff_Temp = np.float32(np.round((Heater_Temp - Air_Temp), 3))   # difference
    return Air_Temp, Diff_Temp

def Init():
    global Power    # heater power value [%]
    global Speed    # fan speed value [%]
    Power = np.float32(0)
    Speed = np.float32(50)
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

# Matrix Q
h[0, 0] = b
h[0, 1] = -a[1] * h[0, 0] + b
for i in range(2, H):
    h[0, i] = -a[1] * h[0, i-1] - a[2] * h[0, i-2] + b

for j in range(0, H):
    for i in range(0, H):
        Q[j+i-1, i] = h[0, j]
Q = Q[0:H, 0:H]

try:
    Init()
    print(" Fan:", Speed, " Ts:", Ts, " H:", H, " ro:", ro)
    for t in range(na, L+na):
        start = time.time()
        [Air_Temp, Y[0, t]] = Temperature_Measurement()
        # Free plant response
        f0 = Y[0, t]
        f[0, 0] = (1 - a[1]) * f0 + (a[1] - a[2]) *Y[0, t-1] + a[2] * Y[0, t-2]
        f[0, 1] = (1 - a[1]) * f[0, 0] + (a[1] - a[2]) * f0 + a[2] * Y[0, t-1]
        f[0, 2] = (1 - a[1]) * f[0, 1] + (a[1] - a[2]) * f[0, 0] + a[2] * f0
        for i in range(3,H):
            f[0, i] = (1 - a[1]) * f[0, i-1] + (a[1] - a[2]) * f[0, i-2] + a[2] * f[0, i-3]
        # GPC algorithm
        dU = inv(Q.T @ Q + ro * np.eye(H)) @ Q.T @ (w[t:t + H] - f.T)
        U[0, t] = np.round((U[0, t - 1] + dU[0]), 3)
        # Boundaries of control signal
        if U[0, t] < Umin:
            U[0, t] = Umin
        elif U[0, t] > Umax:
            U[0, t] = Umax
        dU[0] = U[0, t] - U[0, t - 1]
        Yp = np.round((Q @ dU + f.T), 3)
        Fan.ChangeDutyCycle(V[0, t])
        Heater.ChangeDutyCycle(U[0, t])
        # Saving to .csv
        with open('P_GPC_zmienne_rect.csv', 'a') as csvFile:
            pomiary = csv.writer(csvFile)
            pomiary.writerow([t-na, U[0, t], Y[0, t], Yp[0][0], w[t][0], Air_Temp, V[0, t], Ts, H, ro])
            csvFile.close()
        print("t: ", t-na, " U(t):", U[0,t], " Y(t):", Y[0,t], " Yp(t+1|t):", Yp[0][0], " w(t):", w[t][0], " V(t):", V[0, t])
        end = time.time()
        time.sleep(Ts - end + start)    # uniform sampling period
    Heater_Cooling()

except(KeyboardInterrupt):
    Heater_Cooling()
