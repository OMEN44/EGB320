import sys
if sys.prefix == '/home/pi/egb320/.pixi/envs/default':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/egb320/install/robot'
