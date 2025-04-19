import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/downink4/WWU/CS497/ros-lab-2/install/py_pubsub'
