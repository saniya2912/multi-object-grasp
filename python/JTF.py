import numpy as np
from main_6sept import LeapNode_Poscontrol, LeapNode_Taucontrol
import time
import mujoco

leap = LeapNode_Poscontrol()

# Example usage
index1=[1.84994063e-04,1.03637883e+00, 4.67016263e-01, 6.80227245e-02]
[a,b,c,d]=index1
thumb1=[1.57001343e+00,-1.89591547e-04, 4.25434924e-01, -4.25023042e-01]
[e,f,g,h]=thumb1
index2=[ 2.07837393e-04, 1.34970901e+00, 4.72382367e-01, -2.50659767e-01]
[a1,b1,c1,d1]=index2
thumb2=[1.57001343e+00, -1.89591547e-04, 4.25434924e-01, -4.25023042e-01]
[e1,f1,g1,h1]=thumb2

q0 = np.zeros(16) # 16-element array
v0 = np.zeros(16)  # initial velocity (array of zeros)
q1 = np.array([a,b,c,d, 0,0,0,0,0,0,0,0,e,f,g,h])  # final positions
v1 = np.zeros(16)  # final velocity (array of zeros)
t0 = 4 # initial time
t1 = 12
t2=22
q2=np.array([a1,b1,c1,d1, 0,0,0,0,0,0,0,0,e1,f1,g1,h1])
v2=np.zeros(16)

start_time = time.time()

while time.time() - start_time < 4:
    leap.set_allegro(np.zeros(16))
    time.sleep(0.03)

while time.time() - start_time < t1 and time.time()>t0:
    current_time = time.time() - start_time
    # Ensure the current time falls within [t0, t1]
    if t0 <= current_time <= t1:
        qd = leap.cubic_trajectory(q0, v0, q1, v1, t0, t1, current_time)
        leap.set_allegro(qd)
    time.sleep(0.03)

# while time.time() - start_time < t2 and time.time()>t1:
#     current_time = time.time() - start_time
#     # Ensure the current time falls within [t0, t1]
#     if t1 <= current_time <= t2:
#         qd = leap.cubic_trajectory(q1, v1, q2, v2, t1, t2, current_time)
#         print("Trajectory Positions (qd):", qd)
#         leap.set_allegro(qd)
#     time.sleep(0.03)

# while True:
#     leap.set_allegro(qd)

index_J=np.array([[-1.14115164e-01, -4.64969878e-04, -6.80118100e-02, -3.19937787e-02],
 [ 0.00000000e+00, -6.96632983e-02,  7.61649532e-06,  4.60976973e-06],
 [-1.87688604e-02,  7.85599298e-04,  7.58854679e-03,  1.00198855e-02],
 [ 0.00000000e+00, -8.60565469e-01,  9.42248460e-05,  9.42248460e-05],
 [ 1.00000000e+00,  0.00000000e+00,  9.99999983e-01,  9.99999983e-01],
 [ 0.00000000e+00, -5.09339841e-01, -1.59199502e-04, -1.59199502e-04]])

thumb_J=([[-4.42184769e-20, -2.94453991e-04,  9.04419197e-02,  4.79958762e-02]
 [-1.21742115e-01, -2.92524279e-02, -5.75463041e-06,  1.25514587e-06]
 [ 1.99142316e-04, -2.29016929e-05,  2.92523850e-02,  1.00197735e-02]
 [-1.00000000e+00,  0.00000000e+00,  1.89591546e-04,  1.89591546e-04]
 [ 0.00000000e+00,  7.82898636e-04, -9.99999676e-01, -9.99999676e-01]
 [-2.22044605e-16, -9.99999694e-01, -7.82898622e-04, -7.82898622e-04]])