#!/usr/bin/env python3

import argparse
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import os
import pickle
from pylab import cm
from statistics import median
import sys

# Configurations

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple', 'tab:red', 'tab:olive', 'tab:cyan', 'tab:pink']
fill_color = 'white'

# Basic Parsing Utilities

def lmap(f, ls):
    return list(map(f, ls))

def not_at_end(fd):
    pos = fd.tell()
    l = fd.readline().strip()
    if l is None or l == "" or l[0] == '#':
        return False
    fd.seek(pos)
    return True

def ignore_leading_lines(fd):
    pos, line = None, ""
    while line == "" or line[0] == '#':
        pos = fd.tell()
        line = fd.readline().strip()
    fd.seek(pos)

def parse_id_set(fd):
    initial_id, time, overhead = fd.readline().split()
    times, ohds = [float(time)], [float(overhead)]
    while not_at_end(fd):
        pos = fd.tell()
        id, time, overhead = fd.readline().split()
        if id != initial_id:
            fd.seek(pos)
            break
        times.append(float(time))
        ohds.append(float(overhead))
    return id, times, ohds

def parse_all_sets(fd):
    dct = {}
    while not_at_end(fd):
        id, time, ovhds = parse_id_set(fd)
        dct[int(id)] = (time, ovhds)
    return dct

def parse_file(fn):
    with open(fn, 'r') as fd:
        ignore_leading_lines(fd)
        _ = fd.readline() # NOTE header line
        data_sets = parse_all_sets(fd)
    return data_sets

def from_pickled(fn):
    with open(fn, 'rb') as fd:
        data_set = pickle.load(fd)
    return data_set

def store_as_pickle(d, fn, force=False):
    if not force and os.path.isfile(fn):
        resp = input(f"Overwrite existing file: {fn}? [y/n] ")
        if resp.lower() not in ['y', 'yes']:
           return
    with open(fn, 'wb') as fd:
        pickle.dump(d, fd)

def maybe_warn(b, s):
    if b:
        print(f'\033[93m{s}\033[0m')

# Basic Plotting Utilities

# TODO configure the plots for a professional feel
def plot_init_settings():
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.linewidth'] = 2

def barplot(data_dicts, label_names, max_samples=100, title='Some Bar', oput_fn=None):
    width = 0.30
    labels = lmap(lambda s: s.split(".")[0], label_names)
    custom_lines = [Line2D([0], [0], color=color, lw=2) for color in colors]
    for idx, data_dict in enumerate(data_dicts):
        indices = np.arange(len(data_dict))
        values = np.array(lmap(lambda t: median(t[0]), data_dict.values()))
        values = np.transpose(values)
        plt.bar(indices + idx * width, values, width=width)
    # plt.xticks(ticks=indices)
    plt.xlabel("run index")
    plt.ylabel("median search time (ms)")
    plt.title(title)
    plt.savefig("plot.png")
    if oput_fn is not None:
        plt.savefig(oput_fn+'.png', bbox_inches='tight')

    plt.legend(custom_lines, labels, loc='upper right', prop={'size':7})
    plt.show()

# FIXME improve the quality of graph
def boxplot(data_dicts, label_names, max_samples=3, title='Some Boxplot', oput_fn=None):
    off = 0.2
    center = -0.2
    width = 0.15
    fig, ax = plt.subplots()
    ax.set_title(title)
    labels = lmap(lambda s: s.split(".")[0], label_names)
    custom_lines = [Line2D([0], [0], color=color, lw=2) for color in colors]
    for idx, data_dict in enumerate(data_dicts):
        times = np.array(lmap(lambda p: p[0], data_dict.values()))
        times = times[times.shape[0]-max_samples:times.shape[0],]
        data_array = np.transpose(times)
        pos = np.arange(data_array.shape[1]) + (idx * off) + center
        flierprops = dict(marker='o', markerfacecolor=colors[idx], markersize=1,
                          linestyle='none', markeredgecolor=colors[idx])
        bp = ax.boxplot(data_array, widths=width, patch_artist=True,
                        positions=pos, manage_ticks=False, showfliers=True,
                        flierprops=flierprops, meanline=True)
        for element in ['boxes', 'whiskers', 'fliers', 'medians', 'caps']:
            plt.setp(bp[element], color=colors[idx])
        for patch in bp['boxes']:
            patch.set(facecolor=fill_color)
    if oput_fn is not None:
        plt.savefig(oput_fn+'.png', bbox_inches='tight')
    # ax.legend(frameon=False, loc='upper right', ncol=2)
    plt.xticks(range(max_samples))
    # plt.xlabel()
    # plt.ylabel()
    plt.legend(custom_lines, labels, loc='upper right', prop={'size':7})
    # ax.legend(custom_lines, ['Cold', 'Medium', 'Hot'])
    plt.show()


# MAIN
parser = argparse.ArgumentParser(description='DPHPC data processing utility.')
parser.add_argument('files', nargs='+', metavar='FILENAME', type=str, help='data file to be parsed')
parser.add_argument('-pickled', dest='parser', action='store_const',
                    const=from_pickled, default=parse_file,
                    help='parse data from pickled object (default: parse file)')
parser.add_argument('-title', dest='title',
                    default='Default Title', help='plot title')
parser.add_argument('-p-out', dest='picklefn', nargs='?',
                    default=None, help='filename to store the parsed file as pickled object')
parser.add_argument('-g-out', dest='graphfn', nargs='?',
                    default=None, help='filename to store the generated plot')
parser.add_argument('-Y', dest='force', action='store_const',
                    const=True, default=False,
                    help='force output actions skipping confirmation on file overwrite')

arg = parser.parse_args()
maybe_warn(arg.force, 'Warning: Ouput files will be force overwritten.')
data_sets = lmap(arg.parser, arg.files)

if arg.picklefn is not None and len(arg.files) == 1:
    store_as_pickle(data_sets[0], arg.picklefn, arg.force)

# TODO add support for multiple graphs
plot_init_settings()

barplot(data_sets, arg.files, oput_fn=arg.graphfn, title=arg.title)
# boxplot(data_sets, arg.files, oput_fn=arg.graphfn, title=arg.title)
