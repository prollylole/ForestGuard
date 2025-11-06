import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yut/git/rs1-2025s-ytu77/john_branch/install/forestguard_colour'
