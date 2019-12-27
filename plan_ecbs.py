import csv
import hashlib
import logging
import os
from itertools import product

import numpy as np
import yaml

from libMultiRobotPlanning.benchmark_ecbs import TMP_OUT_FNAME, plan

logger = logging.getLogger(__name__)

BLOCKS_STR = 'blocks'


def gridmap_to_adjlist_and_poses(gridmap, fname_adjlist, fname_nodepose):
    width = gridmap.shape[0]
    height = gridmap.shape[0]
    n_per_xy = {}
    i = 0

    if not os.path.exists(fname_nodepose):
        with open(fname_nodepose, "w") as f_nodepose:
            nodepose_writer = csv.writer(f_nodepose, delimiter=' ')
            for (x, y) in product(range(width), range(height)):
                if gridmap[x, y] == 0:
                    nodepose_writer.writerow([x, y])
                    n_per_xy[(x, y)] = i
                    i += 1
    else:
        for (x, y) in product(range(width), range(height)):
            if gridmap[x, y] == 0:
                n_per_xy[(x, y)] = i
                i += 1

    if not os.path.exists(fname_adjlist):
        with open(fname_adjlist, "w") as f_adjlist:
            adjlist_writer = csv.writer(f_adjlist, delimiter=' ')
            for (x, y) in product(range(width), range(height)):
                if (x, y) in sorted(n_per_xy.keys()):
                    targets = []
                    for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        if (x+dx, y+dy) in n_per_xy.keys():
                            targets.append(n_per_xy[(x+dx, y+dy)])
                    adjlist_writer.writerow([n_per_xy[(x, y)], ] + targets)

    return n_per_xy


def read_outfile(fname):
    with open(fname, 'r') as f:
        data = yaml.load(f)
    # print(data)
    # print('highLevelExpanded: %d'%data['statistics']['highLevelExpanded'])
    return data


def plan_in_gridmap(gridmap: np.ndarray, starts: list(), goals: list(),
                    timeout=30):
    gridmap.flags.writeable = False
    md5 = hashlib.md5(gridmap.data).hexdigest()
    FNAME_ADJLIST = "cache/" + str(md5) + ".adjl.csv"
    FNAME_NP = "cache/" + str(md5) + ".np.csv"
    n_per_xy = gridmap_to_adjlist_and_poses(gridmap, FNAME_ADJLIST, FNAME_NP)
    starts_nodes = [n_per_xy[tuple(s)] for s in starts]
    goals_nodes = [n_per_xy[tuple(s)] for s in goals]
    cost, time = plan(starts_nodes, goals_nodes, FNAME_ADJLIST,
                      FNAME_NP, remove_outfile=False,
                      suboptimality=1.2, timeout=timeout)
    logger.info("cost: %d, time: %f" % (cost, time))

    if os.path.exists(TMP_OUT_FNAME):
        data = read_outfile(TMP_OUT_FNAME)
        os.remove(TMP_OUT_FNAME)
        return data
    else:
        logger.warn("No results file")
