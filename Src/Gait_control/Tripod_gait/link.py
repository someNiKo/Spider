from numpy import pi, cos, sin, arcsin, arccos

AD = 24
AB = 35.49
BC = 22.36
CD = 27
def BD(theta):
    return (AD**2 + AB**2 + 2*AD*AB*cos(theta))**0.5
def phi_1_1(theta):
    return arcsin(AD / BD(theta) * sin(theta))

def phi_1_2(theta):
    return arccos( ( (BD(theta))**2 + BC**2 - CD**2) / (2 * BD(theta) * BC) )

def phi_1(theta):
    return phi_1_1(theta) + phi_1_2(theta)

alpha_0 = pi/2 - phi_1(pi/2)

def phi(theta):
    return phi_1(theta) + alpha_0