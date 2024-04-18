import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2

def get_node_neighbors_4(node):
    neighbors = [
        (node[0] + 1, node[1]), 
        (node[0] - 1, node[1]), 
        (node[0], node[1] + 1), 
        (node[0], node[1] - 1)]
    cost = [1, 1, 1, 1]
    return cost, neighbors

def get_node_neighbors_8(node):
    neighbors = [
        (node[0] + 1, node[1]), 
        (node[0] - 1, node[1]), 
        (node[0], node[1] + 1), 
        (node[0], node[1] - 1),
        (node[0] + 1, node[1] + 1),
        (node[0] + 1, node[1] - 1),
        (node[0] - 1, node[1] + 1),
        (node[0] - 1, node[1] - 1)]
    cost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]
    return cost, neighbors

def get_node_idx(node, map0):
    return node[0]  + node[1] * map0.shape[1]

def vis_01map(map_2d: np.ndarray, save_path: str | None = None, is_vis = True) -> np.ndarray:
    map_vis = (map_2d[:, :, None] * 255).astype(np.uint8)
    map_vis = np.tile(map_vis, [1, 1, 3])

    if save_path is not None:
        Image.fromarray(map_vis).save(save_path)
    if is_vis:
        target_height = 300
        target_width = 500
        resized_image = cv2.resize(map_vis, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('obstacle_map', resized_image)
    return map_vis 

def show_map(
    map0, 
    start=None, end=None, 
    frontier=None, 
    cur_node=None, 
    visited=None, 
    path=None, 
    map_name:str = 'obstacle_map', 
):
    map_vis = vis_01map(map0, is_vis=False)
    if visited is not None and len(visited.shape) == 2:
        map_vis[visited[:, 1], visited[:, 0]] = [255, 255, 0] # yellow
    if frontier is not None:
        map_vis[frontier[:, 1], frontier[:, 0]] = [255, 0, 255] # purple
    if cur_node is not None:
        map_vis[cur_node[1], cur_node[0]] = [0, 255, 0] # green
    if end is not None:
        map_vis[end[1], end[0]] = [255, 0, 0] # red
    if start is not None:
        map_vis[start[1], start[0]] = [0, 0, 255] # blue
    if path is not None:
        map_vis[path[:, 1], path[:, 0]] = [0, 0, 255] # blue

    target_height = 300
    target_width = 500
    resized_image = cv2.resize(map_vis, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
    resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
    cv2.imshow(map_name, resized_image)


# def show_map(map0, start=None, end=None, path=None):
#     plt.imshow(map0, cmap='binary', interpolation='nearest')
#     if start is not None:
#         plt.scatter(start[0], start[1], c='g', marker='s', label='Start')  # 注意：这里需要交换坐标顺序
#     if end is not None:
#         plt.scatter(end[0], end[1], c='r', marker='s', label='End')  # 注意：这里需要交换坐标顺序
#     if path is not None:
#         plt.scatter(path[:, 0], path[:, 1], c='b', marker='o', label='path')
#     plt.legend()
#     plt.show()

def read_pgm(file_name):
    with open(file_name, 'rb') as f:
        # 读取文件头
        header = f.readline().decode().split()
        width = int(f.readline().decode().split()[0])
        height = int(f.readline().decode().split()[0])
        max_val = int(f.readline().decode().split()[0])
        # print(header, width, height, max_val)
        assert header[0] == 'P2', "Invalid PGM file format"
        # 读取图像数据
        data = [int(x) for x in f.read().split()]
    # 将数据转换成图像
    image = Image.new('L', (width, height))
    image.putdata(data)
    # 显示图像
    mapget = np.array(image)
    return mapget
