import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bohn/FishRos/chapt2/demo_python_pkg/install/demo_python_pkg'
