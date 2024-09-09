import numpy as np
from main_6sept import LeapNode_Poscontrol, LeapNode_Taucontrol
import time
import mujoco

# [ 0.03073822  1.49108778  0.05834995 -0.02908696  0.01846634  1.39291321
#   0.74250542  0.80079661  0.01539837  1.35916553  0.69034992  0.78699075
#   1.51563154  0.10590325  0.05988394 -0.07357229]

leap_pos=LeapNode_Poscontrol()
start_time = time.time()

while time.time() - start_time < 5:
    leap_pos.set_allegro([0,np.pi/2,0,0,0,0,0,0,0,0,0,0,np.pi/2,0,0,0])
    time.sleep(0.03)

while time.time() - start_time < 10 and time.time()>5:
    
    # Ensure the current time falls within [t0, t1]
    
    leap_pos.set_allegro([ 0.06141792,  1.48955379,  0.42036949, -0.36196078, -0.00300921,  1.49108778,   #specific to milk can grasping position
  0.66120444,  0.76091324,  0.01386438,  1.4558066,   0.7240976,   0.64433061,
  1.49568974,  0.02920424,  0.20868002, -0.16407718])
    

leap_torque=LeapNode_Taucontrol()

middle_J=np.array([[-7.81501428e-02, -1.55539256e-04, -2.51796518e-02,  7.54582854e-03], #specific to milk can grasping position
 [ 0.00000000e+00, -3.54369962e-02,  2.48025991e-03,  1.83490060e-03],
 [ 6.27285238e-02,  2.20242886e-03,  6.56215493e-02,  5.03953729e-02],
 [ 0.00000000e+00, -9.97515578e-01,  2.59705727e-03,  2.59705727e-03],
 [ 1.00000000e+00,  0.00000000e+00,  9.99320225e-01,  9.99320225e-01],
 [ 0.00000000e+00, -7.04462393e-02, -3.67742140e-02, -3.67742140e-02]])

tertiary_J= np.array([[-8.88466905e-02, -1.70852794e-04, -3.60211914e-02, -6.41372384e-03], #specific to milk can grasping position
 [ 0.00000000e+00, -4.24803993e-02,  1.05205027e-03 , 7.20497073e-04],
 [ 6.69320190e-02,  1.47925465e-03,  7.22206642e-02,  5.15688378e-02],
 [ 0.00000000e+00, -9.93395963e-01,  1.59069924e-03,  1.59069924e-03],
 [ 1.00000000e+00,  0.00000000e+00,  9.99903891e-01,  9.99903891e-01],
 [ 0.00000000e+00, -1.14736483e-01, -1.37723779e-02, -1.37723779e-02]])

F_middle = np.reshape([0, 0, 0.20, 0, 0, 0], [6, 1])
F_tertiary = np.reshape([0, 0, 0.20, 0, 0, 0], [6, 1])

# Compute torque values
Tau_middle = middle_J.T @ F_middle
Tau_middle[[0, 1]] = Tau_middle[[1, 0]]

# Convert torque values to float
Tau_middle = [float(torque[0]) for torque in Tau_middle]

Tau_tertiary = tertiary_J.T @ F_tertiary
Tau_tertiary[[0, 1]] = Tau_tertiary[[1, 0]]

# Convert torque values to float
Tau_tertiary = [float(torque[0]) for torque in Tau_tertiary]

index_J=np.array([[-1.14115164e-01, -4.64969878e-04, -6.80118100e-02, -3.19937787e-02], #specific to milk can grasping position
    [ 0.00000000e+00, -6.96632983e-02,  7.61649532e-06,  4.60976973e-06],
    [-1.87688604e-02,  7.85599298e-04,  7.58854679e-03,  1.00198855e-02],
    [ 0.00000000e+00, -8.60565469e-01,  9.42248460e-05,  9.42248460e-05],
    [ 1.00000000e+00,  0.00000000e+00,  9.99999983e-01,  9.99999983e-01],
    [ 0.00000000e+00, -5.09339841e-01, -1.59199502e-04, -1.59199502e-04]])

thumb_J=np.array([[-4.42184769e-20, -2.94453991e-04,  9.04419197e-02,  4.79958762e-02], #specific to milk can grasping position
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
    

    
while time.time() - start_time < 15 and time.time()>10:
    leap_torque.set_desired_torque([0,0,0,0] + Tau_middle + Tau_tertiary+ [0,0,0,0])

while time.time() - start_time < 50 and time.time()>115:
    
    leap_torque.set_desired_torque(Tau_index + Tau_middle + Tau_tertiary+ Tau_thumb)



#leap_torque=LeapNode_Taucontrol()
# # while True:
# #     leap_pos.set_allegro(qd)

# # Apply torque
# while time.time() - start_time < 15 and time.time() > t1:
# #while True:
#     leap_torque.set_desired_torque(Tau_index+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb)
    
#     print("Torque given:", Tau_index+ [0,-0.02,0,0,0,-0.02,0,0]+Tau_thumb)


# scale_factor = 0.999  # Reduce by 1% in each loop iteration



# while 15 <= (time.time() - start_time) <= 17:
#     print("retract1")
#     # Gradually reduce the forces
#     F_index = scale_factor * F_index
#     F_thumb = scale_factor * F_thumb

#     # Recompute the torque values
#     Tau_index = index_J.T @ F_index
#     Tau_index[[0, 1]] = Tau_index[[1, 0]]

#     Tau_thumb = thumb_J.T @ F_thumb

#     # Convert torque values to float
#     Tau_index = [float(torque[0]) for torque in Tau_index]
#     Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

# leap_pos=LeapNode_Poscontrol()

# while 17 <= (time.time() - start_time) <= 23:
    
#     leap_pos.set_allegro([0,np.pi/2,0,0,0,0,0,0,0,0,0,0,np.pi/2,0,0,0])

   
