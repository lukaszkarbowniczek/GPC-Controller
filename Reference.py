import numpy as np

# Smooth reference trajectory
def Gentle_Reference(N,H):
    R = np.zeros((1, N+H))
    for i in range(0, 60):
        R[0, i] = 3
        R[0, 60+i] = 3+i/20
        if i < 30:
            R[0, 120+i] = 6
        else:
            R[0, 120+i] = 6
        R[0, 180+i] = np.round((6-(i/60)), 3)
        R[0, 240+i] = np.round((4.8+(i/30)**2)**1.1, 3)
        R[0, 300+i] = np.round((7.7-(i/20))**1.1, 3)
        R[0, 360+i] = 5.5
        R[0, 390+i] = 5.5
    R = R.T
    return R

# Step reference trajectory
def Rectangle_Reference(N,H):
    R = np.zeros((1, N+H+1), np.float32)
    R[0, 0:81] = 3.2
    R[0, 81:161] = 7.7
    R[0, 161:241] = 4.9
    R[0, 241:N+H+1] = 10.67
    R = R.T
    return R

