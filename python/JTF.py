import numpy as np
import time
from main_6sept import LeapNode_Poscontrol
import numpy as np
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
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
