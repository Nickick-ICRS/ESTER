#!/usr/bin/env python
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
import matplotlib.pyplot as plt
import argparse
import sys
import os

import pandas as pd
import numpy as np


def find_shift(a, b):
    for i in range(len(a)):
        if a[i] == b[0]:
            return i
    raise RuntimeError("No overlap between A and B")


def intersect(a1, a2, b1, b2):
    s = np.vstack([a1, a2, b1, b2])
    h = np.hstack((s, np.ones((4, 1))))
    l1 = np.cross(h[0], h[1])
    l2 = np.cross(h[2], h[3])
    x, y, z = np.cross(l1, l2)
    if z == 0:
        return False, 0, 0
    return True, x/z, y/z


def main():
    df = pd.read_csv('.used_in_paper/cot_data_2024_09_16_05_19.csv')
    df_no_legs = pd.read_csv('.used_in_paper/cot_data_leg_only_2024_09_16_05_19.csv')

    colours = [
        '#4466aa', '#66ccee', '#228833', '#ccbb44', '#ee6677', '#aa3377',
        '#bbbbbb'
    ]
    strats = [
        'fixed', 'foot_dep', 'stiffness', 'time_opt', 'time_real'
    ]

    assert(len(colours) >= len(strats))

    def plot(USE_SPINE):
        for i, strat in enumerate(strats):
            colour = colours[i]
            # 'fixed' should be consistent
            if USE_SPINE or strat == 'fixed':
                filtered = df.filter(regex=strat)
            else:
                filtered = df_no_legs.filter(regex=strat)
            walking = filtered.filter(regex='walking')
            trotting = filtered.filter(regex='trotting')
            lin_vels_walk = walking.filter(regex='lin vel').to_numpy().flatten()
            lin_vels_trot = trotting.filter(regex='lin vel').to_numpy().flatten()
            cots_walk = walking.filter(regex='cot').to_numpy().flatten()
            cots_trot = trotting.filter(regex='cot').to_numpy().flatten()
            plt.plot(lin_vels_walk, cots_walk, color=colour, lw=3., label=strat)
            plt.plot(lin_vels_trot, cots_trot, color=colour, lw=3.)
            # Find intersection point
            shift = find_shift(lin_vels_walk, lin_vels_trot)
            end = np.argwhere(np.isnan(lin_vels_walk[shift:])).flatten()[0]
            idxs = np.argwhere(np.diff(np.sign(cots_walk[shift:shift+end] - cots_trot[:end]))).flatten()
            if len(idxs):
                for idx in idxs:
                    has_intersection, x, y = intersect(
                        [lin_vels_walk[idx+shift], cots_walk[idx+shift]],
                        [lin_vels_walk[idx+shift+1], cots_walk[idx+shift+1]],
                        [lin_vels_trot[idx], cots_trot[idx]],
                        [lin_vels_trot[idx+1], cots_trot[idx+1]])
                    if has_intersection:
                        plt.plot(x, y, 'x', color='red', mew=2, ms=10., zorder=3)

        plt.xlabel('velocity m/s')
        plt.ylabel('cost of transport')
        if USE_SPINE:
            plt.title('Cost of Transport for Different Spine Control Strategies\nat Different Velocities')
        else:
            plt.title('Cost of Transport for Different Spine Control Strategies\nat Different Velocities (Leg Power Only)')
        plt.legend()
        plt.show()


    plot(True)
    plot(False)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

