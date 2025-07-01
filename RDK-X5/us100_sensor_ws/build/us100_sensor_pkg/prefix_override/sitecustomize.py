import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/us100_sensor_ws/install/us100_sensor_pkg'
