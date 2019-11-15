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
Heater = GPIO.PWM(12, 50)   # 1 PWM canal
Fan = GPIO.PWM(13, 50)  # 2 PWM canal
Sensor = W1ThermSensor.get_available_sensors()  # ID of temperature sensors

a = np.array([1, -1.75, 0.7599], np.float32)  # polynominal A starting parameters   
b = np.float32(0.0036)  # polynominal B starting parameters
na = 2  # number of estimated polynominal A parameters 
nb = 1  # number of estimated polynominal B parameters 
lam = 0.97  # forgetting factor
Ts = 10 # sampling period
T_start = 10    # start of adaptation process
H = 20  # prediction/control horizon
L = 400 # number of samples
P = 100000*np.eye(na+nb)    # covariance matrix
phi = np.zeros((na+nb, 1), np.float32)  # regression vector
theta = np.zeros((na+nb, 1), np.float32)    # parameters vector
h = np.zeros((1, H), np.float32)    # elements of matrix Q
Y = np.zeros((1, L+H), np.float32)  # output signal
U = np.zeros((1, L+H), np.float32)  # control signal
# w = Gentle_Reference(L+50)
w = Rectangle_Reference(L+50)
V = Gentle_Speed(L+50)
# V = Rectangle_Speed(L+50)
f = np.zeros((1, H), np.float32)    # free plant response
Yp = np.array([0])  # predicted output signal
Umin = 0    # minimal value of control signal
Umax = 90   # maximal value of control signal
ro = 0.7   # współczynnik wagowy przyrostu sterowania 
e = 0   # error

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

try:
    Init()
    print(" Fan:", Speed, " Ts:", Ts, " H:", H, " Lambda:", lam)
    U[0, na] = Power
    for t in range(na, L):
        start = time.time()
        [Air_Temp, Y[0, t]] = Temperature_Measurement()
        # VF-RLS algorithm
        phi = np.array([[-Y[0, t-1]], [-Y[0, t-2]], [U[0, t-1]]])
        e = Y[0,t] - phi.T @ theta
        L = P @ phi / (lam + phi.T @ P @ phi)
        P = (P - L @ phi.T @ P) / lam
        theta = theta + L * e
        while t > T_start:
            a = np.array([1, theta[0][0], theta[1][0]], np.float32)
            b = np.float32(theta[2][0])
            break     
        # Matrix Q
        h[0, 0] = b
        h[0, 1] = -a[1] * h[0, 0] + b
        for i in range(2, H):
            h[0, i] = -a[1] * h[0, i - 1] - a[2] * h[0, i - 2] + b
        Q = np.zeros((2 * H, 2 * H), np.float32)
        for j in range(0, H):
            for i in range(0, H):
                Q[j + i - 1, i] = h[0, j]
        Q = Q[0:H, 0:H]
        # Free plant response
        f0 = Y[0, t]
        f[0, 0] = (1 - a[1]) * f0 + (a[1] - a[2]) * Y[0, t - 1] + a[2] * Y[0, t - 2]
        f[0, 1] = (1 - a[1]) * f[0, 0] + (a[1] - a[2]) * f0 + a[2] * Y[0, t - 1]
        f[0, 2] = (1 - a[1]) * f[0, 1] + (a[1] - a[2]) * f[0, 0] + a[2] * f0
        for i in range(3, H):
            f[0, i] = (1 - a[1]) * f[0, i - 1] + (a[1] - a[2]) * f[0, i - 2] + a[2] * f[0, i - 3]
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
        with open('P_AGPC_A2_RT_GS_L97.csv', 'a') as csvFile:
            pomiary = csv.writer(csvFile)
            pomiary.writerow([t-na, U[0, t], Y[0, t], Yp[0][0], w[t][0], a[1], a[2], b,
                              theta[0][0], theta[1][0], theta[2][0], e, Air_Temp, V[0, t], Ts, H, lam])
            csvFile.close()
        print("t:", t, " U(t):", U[0, t], " Y(t):", Y[0, t], " Y(t+1|t):", Yp[0][0], " w(t):", w[t][0], " V(t):", V[0, t],
              " \na1:", a[1], " a2:", a[2], " b0:", b)
        end = time.time()
        time.sleep(Ts - end + start)    # uniform sampling period
    Heater_Cooling()

except(KeyboardInterrupt):
    Heater_Cooling()
