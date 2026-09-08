import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hamr/hamster_ws/src/hamr_control/install/hamr_control'
