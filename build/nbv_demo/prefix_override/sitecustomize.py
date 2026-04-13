import sys
if sys.prefix == '/home/shinobi-wolf/anaconda3/envs/nerfly':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shinobi-wolf/PhD/NeRFly/install/nbv_demo'
