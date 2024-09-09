import numpy as np
import time
from main_6sept import LeapNode_Poscontrol
import numpy as np
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import mujoco
# Assuming LeapNode_Poscontrol is defined as above

# Create an instance of the LeapNode_Poscontrol class
leap_hand = LeapNode_Poscontrol()
pos=leap_hand.read_pos_leap()
print(pos)
# motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
# try:
#     dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
#     dxl_client.connect()
# except Exception as e:
#     print("[DEBUG]", e)
#     try:
#         dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
#         dxl_client.connect()
#     except Exception:
#         dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
#         dxl_client.connect()

# try:
#     while True:
#         # Read the current motor positions
#         current_positions = dxl_client.read_pos()-(np.ones(16)*3.14)

#         # Print the motor positions
#         print("Current Motor Positions:", current_positions)

#         # Add a delay between readings (e.g., 100ms)
#         time.sleep(0.1)

# except KeyboardInterrupt:
#     print("Position reading stopped.")

def J(model,data,site_name):
        # model=mujoco.MjModel.from_xml_path(xml_path)
        # data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        jacp = np.zeros((3, model.nv))  # translation jacobian
        jacr = np.zeros((3, model.nv)) 

        site_id=model.site(site_name).id
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

        return np.vstack((jacp, jacr))

tertiary_model_path = '/home/saniya/leap_hand_mujoco/leap_hand_mujoco/model/leap hand/tertiary.xml'

tertiary_m = mujoco.MjModel.from_xml_path(tertiary_model_path)
tertiary_d = mujoco.MjData(tertiary_m)

tertiary_d.qpos=[1.4558066, 0.01386438,  0.7240976,   0.64433061]
mujoco.mj_forward(tertiary_m, tertiary_d)
tertiary_J=J(tertiary_m,tertiary_d,'contact_tertiary')
print(tertiary_J)
