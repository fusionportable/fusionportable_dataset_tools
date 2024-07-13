import os
import sys

path_file = os.path.abspath(__file__)
path_dir = os.path.dirname(path_file)
sys.path.append(path_dir)

sys.path.append('cfg')
sys.path.append('tools')
sys.path.append('sensor')