from itertools import product
import csv
from benchmark_ecbs import plan
import numpy as np

def gridmap_to_adjlist_and_poses(gridmap, fname_adjlist, fname_nodepose):
    width = gridmap.shape[0]
    height = gridmap.shape[0]
    n_per_xy = {}
    i = 0
    
    with open(fname_nodepose, "w") as f_nodepose:
        nodepose_writer = csv.writer(f_nodepose, delimiter=' ')
        for (x, y) in product(range(width), range(height)):
            if gridmap[x,y] == 0:
                nodepose_writer.writerow([x, y])
                n_per_xy[(x, y)] = i
                i += 1

    with open(fname_adjlist, "w") as f_adjlist:
        adjlist_writer = csv.writer(f_adjlist, delimiter=' ')
        for (x, y) in product(range(width), range(height)):
            if (x, y) in n_per_xy.keys():
                targets = []
                for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if (x+dx, y+dy) in n_per_xy.keys():
                        targets.append(n_per_xy[(x+dx, y+dy)])
                if len(targets) > 0:
                    adjlist_writer.writerow([n_per_xy[(x,y)], ] + targets)
                    
    return n_per_xy


def plan_in_gridmap(gridmap: np.ndarray, starts: list(), goals: list()):
    FNAME_ADJLIST = "adjl.csv"
    FNAME_NP = "np.csv"
    n_per_xy = gridmap_to_adjlist_and_poses(gridmap, FNAME_ADJLIST, FNAME_NP)
    starts_nodes = [n_per_xy[tuple(s)] for s in starts]
    goals_nodes = [n_per_xy[tuple(s)] for s in goals]
    cost, time = plan(starts_nodes, goals_nodes, FNAME_ADJLIST, FNAME_NP)
    print("cost: %d, time: %f"%(cost, time))