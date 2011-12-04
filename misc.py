#This file contains following functions:

#     sign 
#     <input: float>
#     <output: 1 if input is non negative, else -1>

#     atmosphere
#     <input: height>
#     <output: list[2]> list[0] Temperature, list[1] Pressure

#     transformation
#     <input: phi, theta, psi>
#     <output: matrix[3,3] Transformation matrix from earth to body frame> 

from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange, tan
from pylab import plot, legend, figure, subplot
from progressbar import ProgressBar, AnimatedProgressBar

def sign(x):

    
    if x >= 0:
        return 1
    else:
        return -1

def atmosphere(h):
    T = 288 - 6.5*h/1000.0
    R = 287.0
    g = 9.81
    P = 101325.0*exp(-g*h/R/T)
    return [T, P]

def transformation(phi, theta, psi):
    H = zeros([3,3])
    sinR = sin(phi)
    cosR = cos(phi)
    sinP = sin(theta)
    cosP = cos(theta)
    sinY = sin(psi)
    cosY = cos(psi)
    H[0,0] = cosP * cosY;
    H[0,1] = cosP * sinY;
    H[0,2] = -sinP;
    H[1,0] = sinR * sinP * cosY - cosR * sinY;
    H[1,1] = sinR * sinP * sinY + cosR * cosY;
    H[1,2] = sinR * cosP;
    H[2,0] = cosR * sinP * cosY + sinR * sinY;
    H[2,1] = cosR * sinP * sinY - sinR * cosY;
    H[2,2] = cosR * cosP;
    return matrix(H)

def stepprofile(x):
    if x < 250000:
        return 5895.32
    if x >= 250000 and x < 291405:
        return 5895.32 - tan(4*pi/180)*(x-250000)
    if x >= 291405 and x < 350000:
        return 3000
    if x >= 350000:
        return 3000 - tan(4*pi/180)*(x-350000)


def cdaprofile(x):
    if x < 308596.71:
        return 5895.32
    else:
        return 5895.32 - tan(4*pi/180)*(x-308596.71)
