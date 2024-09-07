import numpy as np
from main_6sept import LeapNode_Poscontrol, LeapNode_Taucontrol
import time
import mujoco

index_J=np.array([[-1.14115164e-01, -4.64969878e-04, -6.80118100e-02, -3.19937787e-02],
 [ 0.00000000e+00, -6.96632983e-02,  7.61649532e-06,  4.60976973e-06],
 [-1.87688604e-02,  7.85599298e-04,  7.58854679e-03,  1.00198855e-02],
 [ 0.00000000e+00, -8.60565469e-01,  9.42248460e-05,  9.42248460e-05],
 [ 1.00000000e+00,  0.00000000e+00,  9.99999983e-01,  9.99999983e-01],
 [ 0.00000000e+00, -5.09339841e-01, -1.59199502e-04, -1.59199502e-04]])

thumb_J=np.array([[-4.42184769e-20, -2.94453991e-04,  9.04419197e-02,  4.79958762e-02],
 [-1.21742115e-01, -2.92524279e-02, -5.75463041e-06,  1.25514587e-06],
 [ 1.99142316e-04, -2.29016929e-05,  2.92523850e-02,  1.00197735e-02],
 [-1.00000000e+00,  0.00000000e+00,  1.89591546e-04,  1.89591546e-04],
 [ 0.00000000e+00,  7.82898636e-04, -9.99999676e-01, -9.99999676e-01],
 [-2.22044605e-16, -9.99999694e-01, -7.82898622e-04, -7.82898622e-04]])

F_index = np.reshape([-0.15, 0, 0, 0, 0, 0], [6, 1])
F_thumb=np.reshape([0.15, 0, 0, 0, 0, 0], [6, 1])

# Compute torque values
Tau_index = index_J.T @ F_index
Tau_index[[0, 1]] = Tau_index[[1, 0]]

Tau_thumb = thumb_J.T @ F_thumb

# Convert torque values to float
Tau_index = [float(torque[0]) for torque in Tau_index]
Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

leap_pos = LeapNode_Poscontrol()

# Example usage
index1=[1.84994063e-04,1.03637883e+00, 4.67016263e-01, 6.80227245e-02]
[a,b,c,d]=index1
thumb1=[1.57001343e+00,-1.89591547e-04, 4.25434924e-01, -4.25023042e-01]
[e,f,g,h]=thumb1
# index2=[ 2.07837393e-04, 1.34970901e+00, 4.72382367e-01, -2.50659767e-01]
# [a1,b1,c1,d1]=index2
# thumb2=[1.57001343e+00, -1.89591547e-04, 4.25434924e-01, -4.25023042e-01]
# [e1,f1,g1,h1]=thumb2

q0 = np.zeros(16) # 16-element array
v0 = np.zeros(16)  # initial velocity (array of zeros)
q1 = np.array([a,b,c,d, 0,0,0,0,0,0,0,0,e,f,g,h])  # final positions
v1 = np.zeros(16)  # final velocity (array of zeros)
t0 = 5 # initial time
t1 = 10
t2=20
t3=30
t4=40
# q2=np.array([a1,b1,c1,d1, 0,0,0,0,0,0,0,0,e1,f1,g1,h1])
# v2=np.zeros(16)

start_time = time.time()

while time.time() - start_time < t0:
    leap_pos.set_allegro(np.zeros(16))
    time.sleep(0.03)

while time.time() - start_time < t1 and time.time()>t0:
    current_time=time.time()
    # Ensure the current time falls within [t0, t1]
    
    qd = leap_pos.cubic_trajectory(q0, v0, q1, v1, t0, t1, current_time)
    leap_pos.set_allegro(qd)
    # time.sleep(0.03)

# while time.time() - start_time < t2 and time.time()>t1:
#     current_time = time.time() - start_time
#     # Ensure the current time falls within [t0, t1]
#     if t1 <= current_time <= t2:
#         qd = leap_pos.cubic_trajectory(q1, v1, q2, v2, t1, t2, current_time)
#         print("Trajectory Positions (qd):", qd)
#         leap_pos.set_allegro(qd)
#     time.sleep(0.03)
leap_torque=LeapNode_Taucontrol()
# while True:
#     leap_pos.set_allegro(qd)

# Apply torque
while time.time() - start_time < 15 and time.time() > t1:
#while True:
    leap_torque.set_desired_torque(Tau_index+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb)
    
    print("Torque given:", Tau_index+ [0,-0.02,0,0,0,-0.02,0,0]+Tau_thumb)

# while time.time() - start_time < 20 and time.time() > 15:
#     Tau_index[0]=0.006
#     Tau_thumb[0]=0.006

#     leap_torque.set_desired_torque(Tau_index+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb)
    
#     print("Torque given:", Tau_index+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb)
# # leap_pos = LeapNode_Poscontrol()
# while time.time() - start_time < t3 and time.time() > t2:
#     leap_pos.set_allegro(np.zeros(16))
# #while True:
    
# Scaling factor for gradual reduction
scale_factor = 0.99  # Reduce by 1% in each loop iteration

# Time setup for the force reduction loop
force_reduction_time = 20  # Total time for force reduction
start_time = time.time()

while time.time() - start_time < 30 and time.time() > 15:
    print("retract1")
    # Gradually reduce the forces
    F_index = scale_factor * F_index
    F_thumb = scale_factor * F_thumb

    # Recompute the torque values
    Tau_index = index_J.T @ F_index
    Tau_index[[0, 1]] = Tau_index[[1, 0]]

    Tau_thumb = thumb_J.T @ F_thumb

    # Convert torque values to float
    Tau_index = [float(torque[0]) for torque in Tau_index]
    Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

    # Tau_index[0]=0.006
    # Tau_thumb[0]=0.006

    # Apply the updated torque
    leap_torque.set_desired_torque(Tau_index + [0, -0.1, 0, 0, 0, -0.1, 0, 0] + Tau_thumb)

    # Print updated torque values for debugging

    # Sleep to control the loop rate (adjust as needed)
    time.sleep(0.03)

# force_step_index = np.array([0.0005, 0, 0, 0, 0, 0])  # Change this value as needed
# force_step_thumb = np.array([-0.0005, 0, 0, 0, 0, 0])  # Negative direction later

# # Time setup for the force reduction loop
# force_reduction_time = 20  # Total time for force reduction


# while time.time() - start_time < t3 and time.time() > t2:
#     # Incrementally reduce the forces
#     F_index = F_index - force_step_index  # Subtract force step for index
#     F_thumb = F_thumb - force_step_thumb  # Subtract force step for thumb

#     # Recompute the torque values
#     Tau_index = index_J.T @ F_index
#     Tau_index[[0, 1]] = Tau_index[[1, 0]]  # Swap first two elements as before

#     Tau_thumb = thumb_J.T @ F_thumb

#     # Convert torque values to float
#     Tau_index = [float(torque[0]) for torque in Tau_index]
#     Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

#     # Apply the updated torque
#     leap_torque.set_desired_torque(Tau_index + [0, -0.02, 0, 0, 0, -0.02, 0, 0] + Tau_thumb)

#     # Print updated torque values for debugging
#     print("Torque given:", Tau_index + [0, -0.02, 0, 0, 0, -0.02, 0, 0] + Tau_thumb)

#     # Sleep to control the loop rate (adjust as needed)
#     time.sleep(0.03)
# while time.time() - start_time < t4 and time.time() > t3:
#     print("retract2")
#     # Gradually reduce the forces
#     F_index = np.reshape([0.05, 0, 0, 0, 0, 0], [6, 1])
#     F_thumb =np.reshape([0,-0.05, 0, 0, 0, 0], [6, 1])
#     # Recompute the torque values
#     Tau_index = index_J.T @ F_index
#     Tau_index[[0, 1]] = Tau_index[[1, 0]]

#     Tau_thumb = thumb_J.T @ F_thumb

#     # Convert torque values to float
#     Tau_index = [float(torque[0]) for torque in Tau_index]
#     Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

#     # Apply the updated torque
#     leap_torque.set_desired_torque(Tau_index + [0, -0.1, 0, 0, 0, -0.1, 0, 0] + Tau_thumb)

#     # Print updated torque values for debugging

#     # Sleep to control the loop rate (adjust as needed)
#     time.sleep(0.03)

