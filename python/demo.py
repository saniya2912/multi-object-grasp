import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from main import LeapNode

leap = LeapNode()

# Define your target positions
index1=[1.84994063e-04,1.03637883e+00, 4.67016263e-01, 6.80227245e-02]
thumb1=[1.57001343e+00,-1.89591547e-04, 4.25434924e-01, -4.25023042e-01]
index2=[ 2.07837393e-04, 1.34970901e+00, 4.72382367e-01, -2.50659767e-01]
thumb2=[1.57001343e+00, -1.89591547e-04, 4.25434924e-01, -4.25023042e-01]

import numpy as np
import time

# def cubic_trajectory(q0, v0, q1, v1, t0, tf):
#     # Set up the time array
#     current_time = time.time()
#     t = np.linspace(t0, tf, 100*(tf-t0))
#     c = np.ones_like(t)

#     # Define the matrix M
#     M = np.array([
#         [1, t0, t0**2, t0**3],
#         [0, 1, 2*t0, 3*t0**2],
#         [1, tf, tf**2, tf**3],
#         [0, 1, 2*tf, 3*tf**2]
#     ])

#     # Define vector b
#     b = np.array([q0, v0, q1, v1])

#     # Solve for the coefficients a
#     a = np.linalg.inv(M).dot(b)

#     # Compute current position, velocity, and acceleration
#     current_time = np.clip(current_time, t0, tf)
#     qd = a[0] + a[1]*current_time + a[2]*current_time**2 + a[3]*current_time**3
#     # vd = a[1] + 2*a[2]*current_time + 3*a[3]*current_time**2
#     # ad = 2*a[2] + 6*a[3]*current_time

#     return qd

# Example usage
# q0, v0, q1, v1, t0, tf = 0, 0, 10, 0, 0, 10
# qd, vd, ad = cubic_trajectory(q0, v0, q1, v1, t0, tf)
# print("Position:", qd, "Velocity:", vd, "Acceleration:", ad)


# Record the start time
start_time = time.time()

while time.time() - start_time < 4:
    leap.set_allegro(np.zeros(16))
    time.sleep(0.03)

while 4 <= time.time() - start_time < 6:
    [a, b, c, d] = index1
    [e, f, g, h] = thumb1
    leap.set_allegro([a, b, c, d, 0, 0, 0, 0, 0, 0, 0, 0, e, f, g, h])
    time.sleep(0.03)

while 6 < time.time() - start_time < 2000000:
    [a, b, c, d] = index2
    [e, f, g, h] = thumb2
    leap.set_allegro([a, b, c, d, 0, 0, 0, 0, 0, 0, 0, 0, e, f, g, h])
    time.sleep(0.03)




#  #first object picking   

# result_index1 [1.03637883e+00 1.84994063e-04 4.67016263e-01 6.80227245e-02]
# result_thumb1 [ 1.57001343e+00 -1.89591547e-04  4.25434924e-01 -4.25023042e-01]
# result_index2 [ 1.34970901e+00  2.07837393e-04  4.72382367e-01 -2.50659767e-01]
# result_thumb2 [ 1.57001343e+00 -1.89591547e-04  4.25434924e-01 -4.25023042e-01]


# #push object with thumb
# result_push [ 0.51195958  0.21983941 -0.34789641  1.26686096]
# result_push [0.42515816 0.97037342 0.35358706 1.7871685 ]

# #hold onto first object
# result_grip_t [ 0.93512485 -0.11610709  1.14659567  0.93338226]
# result_grip_m [ 0.92345344 -0.1179225   1.15230915  0.9406219 ]
# result_grip_t [ 0.98540091 -0.08656483  1.13714751  0.88141461]
# result_grip_m [ 0.97582495 -0.09015934  1.13734818  0.88064051]
# result_grip_t [ 0.98343987 -0.08911568  1.13306338  0.87833856]
# result_grip_m [ 0.97322369 -0.09217513  1.13584675  0.88048616]
# result_grip_t [ 0.99545618 -0.08734627  1.13245347  0.87273073]
# result_grip_m [ 0.98550649 -0.0903911   1.13506841  0.87462809]
# result_grip_t [ 0.98941098 -0.05914647  1.23300826  0.97792993]
# result_grip_m [ 0.97771655 -0.06100874  1.23818657  0.98267638]

# #second object
# result_index0 [ 0.5741855   0.00391044  1.01473456 -0.07997615]
# result_thumb0 [ 1.57660345e+00  3.05378516e-04 -1.13998221e-01  1.76467628e-01]
# result_index1 [ 0.58034804  0.00393128  1.06045321 -0.13185685]
# result_thumb1 [ 1.57660334e+00  2.96228834e-04 -1.05423135e-02  7.28843375e-02]
# result_index2 [ 0.58880291  0.00396438  1.10203715 -0.18189506]
# result_thumb2 [ 1.57660334e+00  2.96228834e-04 -1.05423135e-02  7.28843375e-02]




