import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marilene/github/Biomimicry_group6/ROS/install/imu_pkg'
