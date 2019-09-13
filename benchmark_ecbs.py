#!/usr/bin/env python2
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
import yaml
from math import sqrt

GRAPH_AL_FNAME = "graph_adjlist.csv"
GRAPH_AL_UNDIR_FNAME = "graph_adjlist_undir.csv"
GRAPH_NP_FNAME = "graph_pos.csv"
TMP_JOBS_FNAME = "tmp_jobs.csv"
TMP_OUT_FNAME = "/tmp/tmp_out.yaml"
INIT_JOBS_FNAME = "init_jobs.csv"
SUBOPTIMALITY = 1.3
TIMEOUT_S = 120  # 2min
MAX_COST = 9999

logging.getLogger().setLevel(logging.DEBUG)


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


def create_initial_jobs_file(N, n_jobs):
    assert n_jobs <= N, "can only have as many jobs as nodes"
    starts_used = set()
    goals_used = set()
    starts = []
    goals = []
    for _ in range(n_jobs):
        ok = False
        while not ok:
            a_start = random.randint(0, N-1)
            a_goal = random.randint(0, N-1)
            if a_start in starts_used or a_goal in goals_used:
                ok = False
            else:
                c, t = plan([a_start], [a_goal], GRAPH_AL_FNAME, GRAPH_NP_FNAME, 5)
                if c != MAX_COST:
                    ok = True
                else:
                    ok = False
                    logging.warning("{} -> {} does not work".format(a_start, a_goal))
        starts.append(a_start)
        starts_used.add(a_start)
        goals.append(a_goal)
        goals_used.add(a_goal)
    with open(INIT_JOBS_FNAME, "w") as f:
        jobswriter = csv.writer(f, delimiter=' ')
        for j in range(n_jobs):
            jobswriter.writerow([starts[j], goals[j]])


def plan(starts, goals, graph_adjlist_fname, graph_pos_fname, timeout=TIMEOUT_S):
    n_jobs = len(starts)
    assert len(starts) == len(goals), "mus have as many starts as goals"
    with open(TMP_JOBS_FNAME, "w") as f:
        jobswriter = csv.writer(f, delimiter=' ')
        for j in range(n_jobs):
            jobswriter.writerow([starts[j], goals[j]])
    start_time = time.time()
    cmd = [
        os.path.dirname(__file__) + "/build/ecbs",
        "-a", graph_adjlist_fname,
        "-p", graph_pos_fname,
        "-j", TMP_JOBS_FNAME,
        "-o", TMP_OUT_FNAME,
        "-w", str(SUBOPTIMALITY)]
    logging.debug(" ".join(cmd))
    try:
        outstr = subprocess.check_output(cmd)
        logging.debug(outstr)
    except subprocess.CalledProcessError as e:
        logging.warn("CalledProcessError")
        logging.warn(e.output)
        return MAX_COST, 0
    try:
        os.remove(TMP_JOBS_FNAME)
    except FileNotFoundError:
        pass
    if not os.path.exists(TMP_OUT_FNAME):
        logging.error(outstr)
    cost = get_cost_from_outfile(TMP_OUT_FNAME)
    t = time.time() - start_time
    logging.info("cost: " + str(cost))
    try:
        os.remove(TMP_OUT_FNAME)
    except FileNotFoundError:
        pass
    return cost, t


def get_cost_from_outfile(fname):
    cost = 0
    with open(fname, 'r') as f:
        data = yaml.load(f)
    for agent in data['schedule']:
        last = None
        for node in data['schedule'][agent]:
            if last:
                cost += dist(node, last)
            last = node
    return cost / len(data['schedule'])


def dist(a, b):
    return sqrt(
        pow(a['x'] - b['x'], 2)
        + pow(a['y'] - b['y'], 2)
    )


def plan_with_n_jobs(n_jobs, N, graph_adjlist_fname):
    random.seed(2034)
    starts = list(range(N))
    goals = list(range(N))
    random.shuffle(starts)
    random.shuffle(goals)
    starts = starts[:n_jobs]
    goals = goals[:n_jobs]
    return plan(starts, goals, graph_adjlist_fname, GRAPH_NP_FNAME)


def make_undir_graph_file(graph_adjlist_fname, graph_undir_fname):
    def update_graph_dict(d, a, b):
        for (start, end) in [(a, b), (b, a)]:
            if start not in d.keys():
                d[start] = tuple()
            d[start] = d[start] + (end,)
        return d
    with open(graph_adjlist_fname, 'r') as grf:
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
        # ns = range(1, 20)
        ns = range(10, 190, 10)
        if not os.path.exists(GRAPH_AL_UNDIR_FNAME):
            make_undir_graph_file(GRAPH_AL_FNAME, GRAPH_AL_UNDIR_FNAME)
        results = (ns,)
        for n_jobs in ns:
            cs = []
            ts = []
            for graph_adjlist_fname in [GRAPH_AL_UNDIR_FNAME, GRAPH_AL_FNAME]:
                cost, t = plan_with_n_jobs(n_jobs, N, graph_adjlist_fname, )
                cs.append(cost)
                ts.append(t)
                logging.info("graph_adjlist_fname: % 24s | n_jobs: %3d | c: % 8.1f | t: % 6.2fs" %
                      (graph_adjlist_fname, n_jobs, cost, t))
            assert len(cs) == 2, "all graphs should have a cost"
            results = results + (cs, ts)
        write_results(results)
    elif sys.argv[1] == "jobsfile":
        create_initial_jobs_file(200, 100)
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