from py_lib.aux import *
from py_lib.logger import Logger

import heapq
import numpy as np
import cv2
import ipdb


def get_path(map0, start, end, h_multiplier=1.0):
    logger = Logger("./data/py_node_history.txt")
    que = []
    cost_so_far = {}
    come_from = {}
    visited = set()

    heapq.heappush(que, (0, start))
    come_from[start] = None
    cost_so_far[start] = 0

    # ipdb.set_trace()
    while que:
        vis_frontier = [a[1] for a in que]
        cur_cost, cur_node = heapq.heappop(que)
        logger.log(f"{cur_node=}, {cur_cost=}")

        # a node can be pushback twice
        has_common_ele = visited.intersection(set(vis_frontier))

        show_map(map0, start, end, np.array(vis_frontier), cur_node, np.asarray(list(visited)), map_name='dijkstra')
        cv2.waitKey(1)

        if cur_node in visited:
            assert has_common_ele 
            # not all the time has common elements cur_node in visited, 
            # it may not be the cur node, but other node
            logger.log("a node can pushback twice, already get best cost, visited and frontier have common elements, skip")
            continue

        visited.add(cur_node)

        if cur_node == end:
            print("success, early exit.")
            break

        costs, neighbors = get_node_neighbors_8(cur_node)

        for next_cost, next_node in zip(costs, neighbors):
            if map0[next_node[1], next_node[0]] == 1:
                # obstacle
                continue
            if next_node[0] >= map0.shape[1] or next_node[1] >= map0.shape[0]:
                # x < width and y < height, out of map
                continue
            if next_node in visited:
                continue

            # new_cost = cur_cost + next_cost # !!wrong, use cost without hcost
            new_cost = cost_so_far[cur_node] + next_cost

            # only add node to queue if it is not visited or has a lower cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost # use new_cost without hcost, important
                heapq.heappush(que, (new_cost, next_node))
                come_from[next_node] = cur_node
    
    # retrive path
    path = []
    curr = end
    while curr != start: # without start and end
        curr = come_from[curr]
        path.append(curr)
    path.reverse()

    show_map(map0, start, end, np.array(vis_frontier), visited=np.asarray(list(visited)), path=np.asarray(path), map_name='dijkstra')
    cv2.waitKey(0)
    
    return path