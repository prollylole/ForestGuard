import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yut/git/ForestGuard/forest_guard_sim/install/forest_guard_sim'
