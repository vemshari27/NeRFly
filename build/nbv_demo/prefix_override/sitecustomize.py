import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shinobi-owl/PhD/nerf/NeRFly/install/nbv_demo'
