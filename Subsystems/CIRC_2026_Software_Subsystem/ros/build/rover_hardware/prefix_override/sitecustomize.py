import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yurmne/Documents/Coding/School/nbrobotics/CIRC_2025_roverROS/rover_ws/install/rover_hardware'
