import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ymzz/YMbot_ROS2/install/vr_receiver_tcp'
