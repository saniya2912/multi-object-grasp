import numpy as np
from main_6sept import LeapNode_Poscontrol
import time

leap = LeapNode_Poscontrol()



# def cubic_trajectory(q0, v0, q1, v1, t0, t1, current_time):
#     # Ensure the current time is within the valid time range
#     current_time = np.clip(current_time, t0, t1)

#     # Define the matrix M for scalar time t0 and t1 (applies to all elements)
#     M = np.array([
#         [1, t0, t0**2, t0**3],
#         [0, 1, 2*t0, 3*t0**2],
#         [1, t1, t1**2, t1**3],
#         [0, 1, 2*t1, 3*t1**2]
#     ])

#     # Stack the q0, v0, q1, v1 values into a matrix (each as a 16-element array)
#     b = np.vstack([q0, v0, q1, v1])

#     # Solve for the coefficients a for each set of q0, v0, q1, v1
#     a = np.linalg.inv(M).dot(b)

#     # Compute position (qd), velocity (vd), and acceleration (ad) for each element
#     qd = a[0] + a[1]*current_time + a[2]*current_time**2 + a[3]*current_time**3
#     vd = a[1] + 2*a[2]*current_time + 3*a[3]*current_time**2
#     ad = 2*a[2] + 6*a[3]*current_time

#     return qd

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
        print("Trajectory Positions (qd):", qd)
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

while True:
    leap.set_allegro(qd)


# while 6 < time.time() - start_time < 20:
#     [a, b, c, d] = index2
#     [e, f, g, h] = thumb2
#     leap.set_allegro([a, b, c, d, 0, 0, 0, 0, 0, 0, 0, 0, e, f, g, h])
#     time.sleep(0.03)

# while 6 < time.time() - start_time < 20:
#     [a, b, c, d] = index2
#     [e, f, g, h] = thumb2
#     leap.set_allegro([a, b, c, d, 0, 0, 0, 0, 0, 0, 0, 0, e, f, g, h])
#     time.sleep(0.03)