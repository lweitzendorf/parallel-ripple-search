#!/usr/bin/env python3

import argparse
import ast
import os
import pickle

import matplotlib.pyplot as plt
from matplotlib import cm

from tqdm import tqdm

import numpy as np
import scipy.stats


def lmap(f, ls):
    return list(map(f, ls))


# Basic Parsing Utilities

def format_fn(fn):
    chunks = fn.split('.')
    chunks = chunks[0].split('-')
    return '-'.join(chunks[1:])


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
                mi[scenario] = {'lengths': lengths}
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
            assert len(times) == len(
                lengths), f"Sample lengths unequal:\n{len(times)}: {times}\n{len(lengths)}: {lengths}"
            records -= len(times)
    assert records == 0, f"Number of records incorrect, expected: {_or} but got: {records + _or}"


# The data format returned for each file has the following structure:
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
        _ = fd.readline()  # reported time measurements ...
        _ = fd.readline()  # pretty output format
        _ = fd.readline()  # header line
        parse_and_include_times(fd, results, id_map)
        records = int(fd.readline().split()[-2])
        verify(results, records)
        return {'system-info': system_info, 'data': results}


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

            n_needed = (s * scipy.stats.t.ppf(alpha / 2, n - 1) / ex) ** 2
            n_measurements.append(n_needed)

        min = np.ceil(np.min(n_measurements))
        mean = np.ceil(np.mean(n_measurements))
        max = np.ceil(np.max(n_measurements))
        data.append((min, mean, max))

    return data


def ripple_comparison_3d_bar(file_names, parsed_sets, ref_idx):
    relevant_data = [(file, ds) for file, ds in zip(file_names, parsed_sets) if 'ripple-vec' in file]
    reference = parsed_sets[ref_idx]

    X = np.asarray(lmap(lambda d: int(d[0].split('-')[-1]), relevant_data))
    Y = np.asarray([0, 100, 250, 500, 1000, 2000])
    Z = np.zeros((X.size, Y.size))

    bin_idx = np.zeros(2200)
    for i in range(bin_idx.size):
        bin_idx[i] = -1
        for bound in Y:
            bin_idx[i] += (i >= bound)

    for x, (name, file) in enumerate(tqdm(relevant_data)):
        time_sum = np.zeros(Y.size)
        time_count = np.zeros(Y.size)

        for map in file['data']:
            for scenario in file['data'][map]:
                for ref_length, runtime in zip(reference['data'][map][scenario]['lengths'], file['data'][map][scenario]['times']):
                    bin_nr = int(bin_idx[ref_length])
                    time_sum[bin_nr] += runtime
                    time_count[bin_nr] += 1

        time_count[time_count == 0] = 1
        Z[x] = time_sum / time_count

    x_label = [str(x) for x in X]
    y_label = ['[0, 100)', '[100, 250)', '[250, 500)', '[500, 1000)', '[1000, 2000)', '[2000, ∞)']

    _x = np.arange(X.size)
    _y = np.arange(Y.size)
    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()

    top = Z.T.ravel()
    bottom = np.zeros_like(top)
    width = depth = 1

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.bar3d(x, y, bottom, width, depth, top, shade=True)

    ax.set_xticks(_x + 0.5)
    ax.set_xticklabels(x_label)

    ax.set_yticks(_y + 1.5)
    ax.set_yticklabels(y_label)

    ax.set_title('Average Ripple Search Runtime')
    ax.set_xlabel('Thread Count', labelpad=10)
    ax.set_ylabel('Optimal Path Length', labelpad=20)
    ax.set_zlabel('Time (µs)', labelpad=10)

    plt.show()


def ripple_comparison_3d_surface(file_names, parsed_sets, ref_idx):
    reference = parsed_sets[ref_idx]
    relevant_data = [(file, ds) for file, ds in zip(file_names, parsed_sets) if 'ripple-vec' in file]

    X = np.asarray(lmap(lambda d: int(d[0].split('-')[-1]), relevant_data))
    Y = np.arange(2200)
    Z = np.zeros((X.size, Y.size))

    for x, (name, file) in enumerate(tqdm(relevant_data)):
        time_sum = np.zeros(Y.size)
        time_count = np.zeros(Y.size)

        for map in file['data']:
            for scenario in file['data'][map]:
                for ref_length, runtime in zip(reference['data'][map][scenario]['lengths'], file['data'][map][scenario]['times']):
                    time_sum[ref_length] += runtime
                    time_count[ref_length] += 1

        time_count[time_count == 0] = 1
        Z[x] = time_sum / time_count

    X, Y = np.meshgrid(X, Y)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, Z.T, cmap=cm.coolwarm, linewidth=0, antialiased=False)

    ax.set_title('Average Ripple Search Runtime')
    ax.set_xlabel('Thread Count', labelpad=10)
    ax.set_ylabel('Optimal Path Length', labelpad=10)
    ax.set_zlabel('Time (µs)', labelpad=10)

    plt.show()


def variance_box_plots(file_names, parsed_sets):
    time_data = np.zeros((len(parsed_sets), 30))
    cost_data = np.zeros((len(parsed_sets), 30))
    data_count = 0

    for x, file in enumerate(tqdm(parsed_sets)):
        for map in file['data'].values():
            for scenario in map.values():
                time_data[x] += scenario['times']
                cost_data[x] += scenario['lengths']
                data_count += 1

    time_data /= 1e6
    cost_data /= data_count

    labels = [' '.join([chunk.capitalize() for chunk in name.split('-')]) for name in file_names]

    plt.title('Variance in Runtime')
    plt.boxplot(time_data.T, meanline=True)
    plt.xticks(np.arange(1, len(file_names) + 1), labels, rotation='vertical')
    plt.ylabel('Total Time (s)')
    plt.tight_layout()
    plt.show()

    plt.title('Variance in Path Length')
    plt.boxplot(cost_data.T, meanline=True)
    plt.xticks(np.arange(1, len(file_names) + 1), labels, rotation='vertical')
    plt.ylabel('Mean Length (Vertices)')
    plt.tight_layout()
    plt.show()


def performance_plots(file_names, parsed_sets, ref_idx):
    reference = parsed_sets[ref_idx]

    runtimes = np.zeros(parsed_sets.size)
    counts = np.zeros(parsed_sets.size)
    overheads = np.zeros(parsed_sets.size)

    min_path_length = 1000

    for x, file in enumerate(tqdm(parsed_sets)):
        ref_cost = 0
        cost = 0

        for map in file['data']:
            for scenario in file['data'][map]:
                for ref_length, length, runtime in zip(reference['data'][map][scenario]['lengths'],
                                                       file['data'][map][scenario]['lengths'],
                                                       file['data'][map][scenario]['times']):
                    ref_cost += ref_length
                    cost += length

                    if ref_length >= min_path_length:
                        runtimes[x] += runtime
                        counts[x] += 1

        overheads[x] = 100 * (cost / ref_cost - 1)

    runtimes /= counts

    colors = []

    ripple_color = 'tab:blue'
    ripple_vec_color = 'tab:green'

    for name in file_names:
        if 'boost-a-star' in name:
            colors.append('tab:orange')
        elif 'a-star' in name:
            colors.append('tab:brown')
        elif 'fringe-vec' in name:
            colors.append('tab:red')
        elif 'fringe' in name:
            colors.append('tab:purple')
        elif 'ripple-vec' in name:
            colors.append(ripple_vec_color)
        elif 'ripple' in name:
            colors.append(ripple_color)

    fringe_idx = np.where(file_names == 'fringe')[0][0]
    fringe_vec_idx = np.where(file_names == 'fringe-vec')[0][0]

    ripple_idx = np.where([('ripple' in name and 'ripple-vec' not in name) for name in file_names])[0]
    ripple_vec_idx = np.where(['ripple-vec' in name for name in file_names])[0]

    NUM_CORES = 12

    thread_counts = np.asarray(lmap(lambda f: int(f.split('-')[-1]), file_names[ripple_idx]))
    expected_speedup = np.ones(thread_counts.size + 1)

    for i in range(thread_counts.size):
        full_cores = min(thread_counts[i], NUM_CORES) - 1
        hyper_threads = max(0, thread_counts[i] - NUM_CORES)
        work = 1 if full_cores == 2 else 2
        expected_speedup[i+1] = (full_cores + 0.3*hyper_threads) / work

    expected_x = np.concatenate(([fringe_idx], ripple_idx))
    expected_runtime = runtimes[fringe_idx] / expected_speedup
    actual_speedup = runtimes[fringe_idx] / runtimes[expected_x]

    expected_x_vec = np.concatenate(([fringe_vec_idx], ripple_vec_idx))
    expected_runtime_vec = runtimes[fringe_vec_idx] / expected_speedup
    actual_speedup_vec = runtimes[fringe_vec_idx] / runtimes[expected_x_vec]

    thread_x = np.concatenate(([1], thread_counts-1))
    plt.title('Actual vs. Expected Speedup')
    plt.plot(thread_x, expected_speedup, color='black', marker='o', linestyle='dashed', label='Expected')
    plt.plot(thread_x[1:], actual_speedup[1:], color=ripple_color, marker='+', linestyle='none', label='Ripple')
    plt.plot(thread_x[1:], actual_speedup_vec[1:], color=ripple_vec_color, marker='x', linestyle='none', label='Ripple Vec')
    plt.xlabel('Search Thread Count')
    plt.ylabel('Speedup')
    plt.xticks(thread_x)
    plt.legend(loc='upper left')
    plt.show()

    line, = plt.plot(expected_x, expected_runtime, color='black', marker='_', linestyle='none', markersize=10, markeredgewidth=1.5)
    plt.plot(expected_x_vec, expected_runtime_vec, color='black', marker='_', linestyle='none', markersize=10, markeredgewidth=1.5)
    plt.legend(handles=[line], labels=['Expected Runtime'], loc='upper right')

    labels = [' '.join([chunk.capitalize() for chunk in name.split('-')]) for name in file_names]

    plt.title(rf'Average Runtime for Paths $\geq$ {min_path_length} Vertices')
    plt.bar(np.arange(runtimes.size), runtimes, color=colors)
    plt.ylabel('Time (µs)')
    plt.xticks(np.arange(file_names.size), labels, rotation='vertical')
    plt.tight_layout()
    plt.show()

    plt.title('Relative Path Length Error')
    plt.bar(np.arange(overheads.size-1), overheads[1:], color=colors[1:])
    plt.ylabel('Overhead in %')
    plt.xticks(np.arange(file_names.size-1), labels[1:], rotation='vertical')
    plt.tight_layout()
    plt.show()


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
    maybe_warn(arg.force, 'Warning: Output files will be force overwritten')
    parsed_sets = lmap(parser, arg.files)

    if arg.make_pickled:
        lmap(lambda d: store_as_pickle(d[0], d[1], force=arg.force), zip(parsed_sets, arg.files))
        print(f"Done, successfully pickled {len(arg.files)} files!")
        return

    pruned_names = np.asarray(lmap(format_fn, arg.files))
    parsed_sets = np.asarray(parsed_sets)
    ref_idx = np.where(pruned_names == 'a-star')[0][0]

    # print(needed_measurements(data_sets, 0.99, 0.05))
    plot_init_settings()
    performance_plots(pruned_names, parsed_sets, ref_idx)
    # ripple_comparison_3d_surface(pruned_names, parsed_sets, ref_idx)
    # ripple_comparison_3d_bar(pruned_names, parsed_sets, ref_idx)
    # variance_box_plots(pruned_names, parsed_sets)


if __name__ == '__main__':
    main()
