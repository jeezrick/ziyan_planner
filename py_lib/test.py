import sys
sys.path.append('.')
from py_lib.astar import get_path as astar_get_path
from py_lib.dijkstra import get_path as dijk_get_path

import numpy as np


start = (2, 4)
end = (45, 6)

map0 = np.fromfile('./data/map_0.bin', dtype=np.uint8)
map_x, map_y = 50, 30
map0 = map0.reshape(map_y, map_x)

path = astar_get_path(map0, start, end)
path = dijk_get_path(map0, start, end)
print(path)