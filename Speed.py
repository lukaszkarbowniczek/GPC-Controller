import numpy as np

def Gentle_Speed(N):
    V = np.zeros((1, N), np.float32)
    for i in range(0, 51):
        V[0, i] = np.round(np.float32(32*i/70+20), 3)
        V[0, 51+i] = np.round(np.float32(32*(51+i)/70+20), 3)
        V[0, 101+i] = np.round(np.float32(32*(101+i)/70+20), 3)
        V[0, 151+i] = np.round(np.float32(89-i/2), 3)
        V[0, 201+i] = 64
        V[0, 251+i] = np.round(np.float32(89-(i+51)/2), 3)
        V[0, 301:450] = 39
    return V

def Rectangle_Speed(N,H):
    V = np.zeros((1, N+H+1), np.float32)
    V[0, 0:150] = 50
    V[0, 150:N+H+1] = 90
    return V