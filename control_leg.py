import numpy as np
# import can_control as cc
#ab/ad is 1
#upper leg is 2
#lower leg is 3
#constants
r1 = 20
r2 = 20
L = 200

def goto(x,y,z):
    A = 51.5
    E = 200
    F = 200 #just an estimate
    D = np.sqrt(z**2 + y**2 - A**2)
    w1 = np.arctan2(z,y) + np.arctan2(D,A) - np.pi/2
    G = np.sqrt(D**2 + x**2)
    cos_theta3 = (G**2 - E**2 - F**2)/(-2*E*F)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    theta_3 = np.arccos(cos_theta3)
    w2 = np.arctan2(x,D) + np.arcsin(F*np.sin(theta_3)/G)
    w3 = theta_3 + w2 - np.pi
    # w3 = get_w3(w2, E, r1, L, r2)
    return w1,w2,w3


# #uses Freudenstein Equation and quadratic
# #r1 = ground r2 = input r3 = coupler r4 = output
# def get_w3(theta2, r1, r2, r3, r4):
#     k1 = r1/r2
#     k2 = (r1**2 + r2**2 - r3**2 + r4**2)/(2*r2*r4)
#     k3 = r1/r4

#     A = np.cos(theta2) - k1 - k3*np.cos(theta2) + k2
#     B = -2*np.sin(theta2)
#     C = k1 - (k3+1)*np.cos(theta2) + k2

#     disc = B**2 - 4*A*C

#     if disc < 0:
#         return None  # no real configuration

#     t1 = (-B + np.sqrt(disc))/(2*A)
#     t2 = (-B - np.sqrt(disc))/(2*A)

#     theta3_candidates = [2*np.arctan(t1), 2*np.arctan(t2)]

#     for theta3 in theta3_candidates:
#         # joint A (input)
#         Ax = r2*np.cos(theta2)
#         Ay = r2*np.sin(theta2)

#         # joint B (output)
#         Bx = r1 + r4*np.cos(theta3)
#         By = r4*np.sin(theta3)

#         # open linkage condition
#         if Ay > 0 and By > 0:
#             return theta3

#     return "ERROR INVALID THETA"

def rtod(radian):
    return radian * 180/np.pi

w1,w2,w3 = goto(20,400,-15)
print(w1,w2,w3)
print(rtod(w1),rtod(w2), rtod(w3))
