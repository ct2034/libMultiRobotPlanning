#!/usr/bin/env python3
import csv
from itertools import dropwhile
from matplotlib import pyplot as plt
import os
import random
import re
import logging
import subprocess
import sys
import time

GRAPH_AL_FNAME = "graph_adjlist.csv"
GRAPH_AL_UNDIR_FNAME = "graph_adjlist_undir.csv"
GRAPH_NP_FNAME = "graph_pos.csv"
TMP_JOBS_FNAME = "tmp_jobs.csv"
SUBOPTIMALITY = 2.
TIMEOUT_S = 20

logging.getLogger().setLevel(logging.INFO)


def max_vertex():
    max_so_far = 0
    with open(GRAPH_AL_FNAME, "r") as f:
        graphreader = csv.reader(f, delimiter=' ')
        for line in graphreader:
            if not line[0].startswith("#"):
                max_so_far = max(
                    max_so_far,
                    max(map(int, line))
                )
    return max_so_far


def get_unique(path):
    assert path.__class__ == list, "must be list"
    assert path[0].__class__ == list, "must be listof lists"
    list_length = len(path)
    last_vertex = path[-1][1]
    unique_reversed = list(dropwhile(
        lambda x: x[1] == last_vertex,
        reversed(path)))
    unique = (list(reversed(unique_reversed))
              + [[len(unique_reversed), last_vertex]])
    assert len(unique) <= list_length
    return unique, len(unique)


def plan_with_n_jobs(n_jobs, N, graph_fname):
    # random.seed(1)
    starts = list(range(N))
    goals = list(range(N))
    random.shuffle(starts)
    random.shuffle(goals)
    starts = starts[:n_jobs]
    goals = goals[:n_jobs]
    try:
        os.remove(TMP_JOBS_FNAME)
    except FileNotFoundError:
        pass
    with open(TMP_JOBS_FNAME, "w") as f:
        jobswriter = csv.writer(f, delimiter=' ')
        for j in range(n_jobs):
            jobswriter.writerow([starts[j], goals[j]])
    start_time = time.time()
    cmd = [
        "build/ecbs",
        "-a", graph_fname,
        "-p", GRAPH_NP_FNAME,
        "-j", TMP_JOBS_FNAME,
        "-o", "output.yaml",
        "-w", str(SUBOPTIMALITY)]
    logging.debug(" ".join(cmd))
    try:
        cp = subprocess.run(cmd,
            stdout=subprocess.PIPE,
            timeout=TIMEOUT_S
        )
        outstr = cp.stdout.decode('utf-8')
        logging.debug(outstr)
        rline = re.compile("done; cost: [0-9]+")
        mline = rline.findall(outstr)[0]
        logging.debug("mline: >" + mline)
        rnr = re.compile("[0-9]+")
        cost = float(
            rnr.findall(mline)[0]
        )
    except subprocess.TimeoutExpired:
        logging.warn("timeout")
        cost = TIMEOUT_S
    t = time.time() - start_time
    logging.debug("Took " + format(t, ".1f") + "s")
    os.remove(TMP_JOBS_FNAME)
    return cost, t


def make_undir_graph_file(graph_fname, graph_undir_fname):
    def update_graph_dict(d, a, b):
        for (start, end) in [(a, b), (b, a)]:
            if start not in d.keys():
                d[start] = tuple()
            d[start] = d[start] + (end,)
        return d
    with open(graph_fname, 'r') as grf:
        grreader = csv.reader(grf, delimiter=' ')
        edges = {}
        for node in grreader:
            if not node[0].startswith("#"):
                for target in node[1:]:
                    edges = update_graph_dict(
                        d=edges,
                        a=int(node[0]),
                        b=int(target)
                    )
    with open(graph_undir_fname, 'w') as gruf:
        grufwriter = csv.writer(gruf, delimiter=' ')
        nodes = list(edges.keys())
        nodes.sort()
        for node in nodes:
            grufwriter.writerow([node] + list(edges[node]))


def write_results(results):
    with open("results.csv", 'w') as f:
        reswriter = csv.writer(f, delimiter=' ')
        for res in results:
            reswriter.writerow(res)


def read_results():
    out = tuple()
    with open("results.csv", 'r') as res:
        resreader = csv.reader(res, delimiter=' ')
        for line in resreader:
            out = out + (list(map(float, line)),)
    return out


if __name__ == '__main__':
    if sys.argv[1] == "eval":
        N = max_vertex()
        # ns = [1, 2, 3, 5, 10, 20, 30, 50, 100]
        ns = range(1, 20)
        # ns = range(10, 120, 10)
        if not os.path.exists(GRAPH_AL_UNDIR_FNAME):
            make_undir_graph_file(GRAPH_AL_FNAME, GRAPH_AL_UNDIR_FNAME)
        results = (ns,)
        for graph_fname in [GRAPH_AL_UNDIR_FNAME, GRAPH_AL_FNAME]:
            cs = []
            ts = []
            logging.info("graph_fname: %s" % graph_fname)
            for n_jobs in ns:
                cost, t = plan_with_n_jobs(n_jobs, N, graph_fname)
                cs.append(cost)
                ts.append(t)
                logging.info("n_jobs: %3d | c: % 7.2f | t: % 7.2fs" %
                      (n_jobs, cost, t))
            assert len(cs) == len(ns), "all ns should have a cost"
            results = results + (cs, ts)
        write_results(results)
    elif sys.argv[1] == "plot":
        (ns, cs_u, ts_u, cs_d, ts_d) = read_results()
        x = range(len(ns))
        plt.style.use('bmh')
        fig, (ax_cost, ax_time) = plt.subplots(2, 1)
        fig.tight_layout()
        ax_cost.plot(x, cs_u, label='undirected')
        ax_cost.plot(x, cs_d, label='directed')
        ax_time.plot(x, ts_u, label='undirected')
        ax_time.plot(x, ts_d, label='directed')
        plt.setp(ax_cost, xticks=x, xticklabels=map(str, map(int, ns)),
                 title="Cost")
        plt.setp(ax_time, xticks=x, xticklabels=map(str, map(int, ns)),
                 title="Computation Time")
        ax_cost.legend()
        ax_time.legend()
        plt.show()
    else:
        assert False, "choose either eval or plot"
