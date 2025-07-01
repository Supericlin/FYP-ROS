import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/mqtt_goal_ws/install/mqtt_navigation'
