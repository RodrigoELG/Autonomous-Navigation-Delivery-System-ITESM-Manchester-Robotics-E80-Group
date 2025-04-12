import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/shared-folder/ros2_ws/src/puzzlebot_sim/install/puzzlebot_sim'
