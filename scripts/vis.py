#!/usr/bin/env python3

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
import sys
from pylab import cm

# Basic Parsing Utilities

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

def maybe_warn(b):
    if b:
        print('\033[93mWarning: Ouput files will be force overwritten.\033[0m')

# Basic Plotting Utilities

'''
# TODO configure the plots for a professional feel
def plot_init_settings():
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['font.size'] = 18
    plt.rcParams['axes.linewidth'] = 2
'''

# FIXME improve the quality of graph
def boxplot(data_dict, max_samples=10, title='Some Boxplot', oput_fn=None):
    fig, ax = plt.subplots()
    ax.set_title(title)
    times = np.array(list(map(lambda p: p[0], data_dict.values())))
    data_array = np.transpose(times[0:max_samples,])
    pos = np.arange(data_array.shape[1])
    ax.boxplot(data_array, patch_artist=True, positions=pos)
    if oput_fn is not None:
        plt.savefig(oput_fn+'.png', bbox_inches='tight')
    plt.show()

# MAIN
parser = argparse.ArgumentParser(description='DPHPC data processing utility.')
parser.add_argument('filename', metavar='FILENAME', type=str, help='data file to be parsed')
parser.add_argument('-pickled', dest='parser', action='store_const',
                    const=from_pickled, default=parse_file,
                    help='parse data from pickled object (default: parse file)')
parser.add_argument('-p-out', dest='picklefn', nargs='?',
                    default=None, help='filename to store the parsed file as pickled object')
parser.add_argument('-g-out', dest='graphfn', nargs='?',
                    default=None, help='filename to store the generated plot')
parser.add_argument('-Y', dest='force', action='store_const',
                    const=True, default=False,
                    help='force output actions skipping confirmation on file overwrite')

arg = parser.parse_args()
maybe_warn(arg.force)
data = arg.parser(arg.filename)
if arg.picklefn is not None:
    store_as_pickle(data, arg.picklefn, arg.force)
# TODO add support for multiple graphs
boxplot(data, oput_fn=arg.graphfn)
