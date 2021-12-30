#!/usr/bin/env python3

import argparse
import ast
import os
import pickle
from statistics import median

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
from matplotlib.lines import Line2D

# Configurations

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple', 'tab:red', 'tab:olive', 'tab:cyan', 'tab:pink']
fill_color = 'white'
group_bounds = [50, 150, 500, 1000, 100000000000000]

def lmap(f, ls):
    return list(map(f, ls))

# Basic Parsing Utilities

def parse_costs(fn):
    costs = []
    with open(fn, 'rb') as fd:
        while not at_end(fd):
            line = fd.readline()
            cost = int(line)
            costs.append(cost)
    return costs

def at_end(fd):
    pos = fd.tell()
    l = fd.readline().strip()
    pos = fd.seek(pos)
    return l is None or len(l) == 0 or l[0] == '#'

def split_with_head(line, on=":"):
    sides = line.split(on)
    try:
        return (sides[0].split()[-1], sides[1].split())
    except:
        return ("", sides)

def parse_machine_information(fd):
    info, pos = {}, None
    kws = ['sysname', 'nodename', 'release', 'version',
           'machine', 'execution', '(utc)', '(local)']
    while True:
        pos = fd.tell()
        (hd, tl) = split_with_head(fd.readline())
        hd = hd.lower()
        if hd in kws:
            info[hd] = ' '.join(tl)
        else:
            fd.seek(pos)
            return info

def parse_map_lengths(fd):
    def parse_lengths():
        lengths, pos = [], None
        while True:
            pos = fd.tell()
            (hd, tl) = split_with_head(fd.readline(), on="=")
            if hd.lower() == 'length':
                lengths.append(int(tl[0]))
            else:
                fd.seek(pos)
                return lengths

    def parse_scenarios(im):
        mi = {}
        while True:
            pos = fd.tell()
            (hd, tl) = split_with_head(fd.readline(), on="=")
            if hd.lower() == 'scenario':
                scenario = ast.literal_eval(' '.join(tl).replace("[", "(").replace("]", ")"))
                lengths = parse_lengths()
                im.append((map_name, scenario))
                mi[scenario] = { 'lengths': lengths }
            else:
                fd.seek(pos)
                return mi

    map_dict, id_map, pos = {}, [], None
    while True:
        pos = fd.tell()
        hd, tl = split_with_head(fd.readline(), on="=")
        if hd.lower() != 'map':
            fd.seek(pos)
            return (map_dict, id_map)
        else:
            map_name = tl[0].split(".")[0]
            map_info = parse_scenarios(id_map)
            map_dict[map_name] = map_info

def parse_and_include_times(fd, map_dict, ip_map):
    def parse_one():
        initial_id, time, overhead = fd.readline().split()
        times, ohds = [float(time)], [float(overhead)]
        while not at_end(fd):
            pos = fd.tell()
            new_id, time, overhead = fd.readline().split()
            if new_id != initial_id:
                fd.seek(pos)
                break
            times.append(float(time))
            ohds.append(float(overhead))
        map_name, scenario = ip_map[int(initial_id)]
        map_dict[map_name][scenario]['times'] = times
    while not at_end(fd):
        parse_one()

def verify(map_dict, records):
    print("\tVerifying parsed records ...")
    _or = records
    for scenarios in map_dict.values():
        for scenario in scenarios.values():
            times = scenario['times']
            lengths = scenario['lengths']
            assert len(times) == len(lengths), f"Sample lengths unequal:\n{len(times)}: {times}\n{len(lengths)}: {lengths}"
            records -= len(times)
    assert records == 0, f"Number of records incorrect, expected: {_or} but got: {records + _or}"

# The data format returned for each file has the follwoing structure:
#
# {
#   'system-info' : { ... }
#   'data' :
#     {
#       map-name :
#         {
#           scenario :
#             {
#               'lengths' : [int]
#               'times'   : [float]
#             }
#         }
#     }
# }
def parse_bench_file(filename):
    print(f"Parsing from file: {filename}")
    with open(filename, 'r') as fd:
        system_info = parse_machine_information(fd)
        (results, id_map) = parse_map_lengths(fd)
        _ = fd.readline() # reported time measurements ...
        _ = fd.readline() # pretty output format
        _ = fd.readline() # header line
        parse_and_include_times(fd, results, id_map)
        records = int(fd.readline().split()[-2])
        verify(results, records)
        return { 'system-info' : system_info, 'data' : results }

def from_pickled(fn):
    with open(fn, 'rb') as fd:
        data_set = pickle.load(fd)
    return data_set

def store_as_pickle(d, fn, force=False):
    fn = '-'.join(fn.split(".")[:-1]) + '.pickle'
    if not force and os.path.isfile(fn):
        resp = input(f"Overwrite existing file: {fn}? [y/n] ")
        if resp.lower() not in ['y', 'yes']:
           return
    print(f"Pickling data to: {fn}")
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

def grouped_bar(data_dicts, costs, label_names, max_samples=3, title='Some Boxplot', oput_fn=None):

    cost_idxs = np.argsort(costs[::20])
    costs = np.array(costs[::20])
    costs = costs[cost_idxs]
    def loop(dict):
        list_of_medians = lmap(lambda t: median(t[0]), dict.values())
        npa = np.array(list_of_medians)
        i = 0
        results = [[] for _ in group_bounds]
        current_group = []
        for j, median_rt in  enumerate(npa[cost_idxs]):
            if costs[j] >= group_bounds[i]:
                i += 1
            results[i].append(median_rt)

        results = lmap(median, results)
        return np.array(results)

    # [[A1, A2, ...],
    #  [A1, A2, ...],
    #  [A1, A2, ...],
    #           ...]
    grouped_runtimes = np.array(lmap(loop, data_dicts))

    off = 0.2
    center = -0.2
    width = 0.15
    labels = lmap(lambda s: s.split(".")[0], label_names)
    custom_lines = [Line2D([0], [0], color=color, lw=2) for color in colors]

    indices = np.arange(len(grouped_runtimes))
    values = grouped_runtimes
    # indices = np.arange(len(data_dicts))
    # values = np.array(lmap(lambda t: median(t[0]), data_dicts.values()))
    # values = np.transpose(values)
    # plt.bar(indices + idx * width, values, width=width)

    # plt.bar(indices + idx * width, values, width=width)
    for idx, algorithm_results in enumerate(values):
        indices = np.arange(len(group_bounds))
        plt.bar(indices + idx * width, algorithm_results, width=width)

    plt.title(title)
    plt.xticks(range(len(group_bounds)), ['<50', '<150', '<500', '<1000', '>=1000'])
    plt.xlabel("path length")
    plt.ylabel("runtime in microseconds")
    # plt.legend(custom_lines, labels, loc='upper left', prop={'size':7})
    if oput_fn is not None:
        plt.savefig(oput_fn+'.png', bbox_inches='tight')
    plt.show()

# calculates number of measurements needed for the (100*confidence)% confidence interval
# to be within (100*acceptable_deviaton)% of the mean
def needed_measurements(data_dicts, confidence, acceptable_deviaton):
    data = []

    for algorithm_data in data_dicts:
        n_measurements = []
        for runtimes, overheads in algorithm_data.values():
            n = len(runtimes)
            s = np.std(runtimes)

            ex = acceptable_deviaton * np.mean(runtimes)
            alpha = 1 - confidence

            n_needed = (s * scipy.stats.t.ppf(alpha/2, n-1) / ex)**2
            n_measurements.append(n_needed)

        min = np.ceil(np.min(n_measurements))
        mean = np.ceil(np.mean(n_measurements))
        max = np.ceil(np.max(n_measurements))
        data.append((min, mean, max))

    return data

def main():
    parser = argparse.ArgumentParser(description='DPHPC data processing utility')
    parser.add_argument('files', nargs='+', metavar='FILENAME', type=str, help='data file to be parsed')
    parser.add_argument('-make', dest='make_pickled', action='store_true', default=False,
                        help='parse data from file (default: parse pickled)')
    parser.add_argument('-title', dest='title',
                        default='Default Title', help='plot title')
    parser.add_argument('-costs', nargs=1, dest='costfn', help='todo')
    parser.add_argument('-g-out', dest='graphfn', nargs='?',
                        default=None, help='filename to store the generated plot')
    parser.add_argument('-Y', dest='force', action='store_true', default=False,
                        help='force output actions skipping confirmation on file overwrite')
    arg = parser.parse_args()
    parser = parse_bench_file if arg.make_pickled else from_pickled
    maybe_warn(arg.force, 'Warning: Ouput files will be force overwritten')
    parsed_sets = lmap(parser, arg.files)

    if arg.make_pickled:
        lmap(lambda d: store_as_pickle(d[0], d[1], force=arg.force), zip(parsed_sets, arg.files))
        print(f"Done, successfully pickled {len(arg.files)} files!")
        return

    # costs = arg.parser(arg.files)

    # TODO add support for multiple graphs
    # plot_init_settings()
    # barplot(data_sets, arg.files, oput_fn=arg.graphfn, title=arg.title)
    # boxplot(data_sets, arg.files, oput_fn=arg.graphfn, title=arg.title)
    # grouped_boxplot(data_sets, costs, arg.files, oput_fn=arg.graphfn, title=arg.title)
    # print(needed_measurements(data_sets, 0.99, 0.05))

if __name__ == '__main__':
    main()
