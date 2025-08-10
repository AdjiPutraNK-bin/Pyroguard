import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adji714/ros2_ws/src/pyroguard/install/pyroguard'
