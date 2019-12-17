#!/usr/bin/env python
import numpy as np
import sys
RECURSION_LIM = 400
#sys.setrecursionlimit(RECURSION_LIM)
import csv
from Rico_picToARRAY import get_pic_array
import matplotlib.pyplot as plt


def get_connected_neighbors(pic, point):
    pic = np.array(pic)
    ret_neighbors = set()
    for row_increment in [-1, 0, 1]:
        for cln_increment in [-1, 0, 1]:
            new_point = (point[0]+row_increment, point[1]+cln_increment)
            if row_increment == 0 and cln_increment == 0:
                continue

            if new_point[0] >= ROW_NUM or new_point[0] < 0 or new_point[1]>=COLUMN_NUM or new_point[1]<0:
                continue
            if pic[new_point[0], new_point[1]] < 250:
                ret_neighbors.add(new_point)
    return ret_neighbors

def is_neighbor (a, b):
    if abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1:
        return True
    else:
        return False

def add_point_to_path_ls(path_list, start):
    if len(path_list) == 0:
        path_list.append([start])
    else:
        if is_neighbor(path_list[-1][-1], start):
            path_list[-1].append(start)
        else:
            path_list.append([start])
    return path_list


def dfs(counter, pic, start, path_list, visited=None):
    if visited is None:
        visited = set()

    if counter< RECURSION_LIM:
        # print "counter, ", counter
        if pic[start[0]][start[1]] <= 250:
            visited.add(start)
            connected_neighbors = get_connected_neighbors(pic, start)
            for next_pt in connected_neighbors - visited:
                if len(connected_neighbors - visited) != 0:
                    counter += 1
                    path_list, counter = dfs(counter, pic, next_pt, path_list, visited)

            #if there is only one point in the graph, omit it.
            path_list = add_point_to_path_ls(path_list, start)
    return path_list, counter


def wipe_off_visited_points(path_list, pic):
    for path in path_list:
        for pt in path:
            pic[pt[0]][pt[1]] = 255
    return pic


total_path_list = []

pic = get_pic_array("../images/IMG_4926_adjust_2.jpg")
# pic = np.array([
#     [1, 1, 1, 1],
#     [255, 1, 255, 255],
#     [255, 255, 255, 1],
#     [255, 255, 1, 255],
#     [255, 255, 255, 1],
#     [255, 255, 255, 255],
#     [255, 1, 255, 1],
#     [255, 1, 255, 1],
#     [255, 255, 255, 255]
# ])

ROW_NUM = len(pic)
COLUMN_NUM = len(pic[0])
path_count = 0
populate_count = 0

p = 255*np.ones((ROW_NUM, COLUMN_NUM))

for row in range(ROW_NUM):
    for cln in range(COLUMN_NUM):
        counter = 0
        path_list, counter = dfs(counter, pic, (row, cln), [])
        pic = wipe_off_visited_points(path_list,pic)
        path_count += 1
        total_path_list = total_path_list + path_list
        populate_count += 1



for index, path in enumerate(total_path_list):
    print index
    if len(path) == 1:
        total_path_list.pop(index)

#Test
for path in total_path_list:
    for pt in path:
        p[pt[0]][pt[1]] = 1
    populate_count += 1

print "populated: ", populate_count
plt.imshow(p, cmap='gray', vmin=0, vmax=255)
plt.show()
