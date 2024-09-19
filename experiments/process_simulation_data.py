import numpy as np
import pandas as pd
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

import os
import re

this_path = os.path.dirname(os.path.realpath(__file__))

data_dir = os.path.join(this_path, 'raw_data')

all_files = os.listdir(data_dir)

files = {}
files['Trot'] = {}
files['Walk'] = {}
files['Turn'] = {}
files['Trot']['all'] = list([f for f in all_files if 'trot' in f])
files['Walk']['all'] = list([f for f in all_files if 'walk' in f and 'turning' not in f])
files['Turn']['all'] = list([f for f in all_files if 'turning' in f])

gaits = ['Trot', 'Walk', 'Turn']
strats = ['foot_dep', 'stiffness', 'time_opt', 'time_real', 'fixed']
colours = {
    'fixed': '#4466aa',
    'foot_dep': '#66ccee',
    'stiffness': '#228833',
    'time_opt': '#ccbb44',
    'time_real': '#ee6677',
}

for g in gaits:
    files[g]['fixed'] = list([f for f in files[g]['all'] if 'fixed' in f])
    files[g]['stiffness'] = list([f for f in files[g]['all'] if 'stiff' in f])
    files[g]['time_opt'] = list([f for f in files[g]['all'] if 'opt_time' in f])
    files[g]['time_real'] = list([f for f in files[g]['all'] if 'real_dog_time' in f])
    files[g]['foot_dep'] = list([f for f in files[g]['all'] if 'foot' in f])


def get_lin_vel(filename):
    pattern = r"^(?:trot|walk)_([0-9]*\.?[0-9]+)_"
    match = re.search(pattern, filename)
    if match:
        lin_vel = float(match.group(1))
        return lin_vel
    raise ValueError("No linear velocity found for file: " + filename)


def get_lin_ang_vel(filename):
    pattern = r"^(?:trot|walk)_([0-9]*\.?[0-9]+)_turning_([0-9]*\.?[0-9]+)_"
    match = re.search(pattern, filename)
    if match:
        lin_vel = float(match.group(1))
        ang_vel = float(match.group(2))
        return lin_vel, ang_vel
    raise ValueError("No angular velocity found for file: " + filename)


vels = {}
dfs = {}

for g in gaits:
    vels[g] = {}
    dfs[g] = {}
    if g == 'Turn':
        vels[g]['all'] = np.array([get_lin_ang_vel(f)[1] for f in files[g]['all']])
    else:
        vels[g]['all'] = np.array([get_lin_vel(f) for f in files[g]['all']])
    dfs[g]['all'] = list([pd.read_csv(os.path.join(data_dir, f)).rename(
        columns=lambda x: x.strip()) for f in files[g]['all']])
    dfs[g]['fixed'] = list([df for df, f in zip(dfs[g]['all'], files[g]['all']) if 'fixed' in f])
    dfs[g]['foot_dep'] = list([df for df, f in zip(dfs[g]['all'], files[g]['all']) if 'foot' in f])
    dfs[g]['stiffness'] = list([df for df, f in zip(dfs[g]['all'], files[g]['all']) if 'stiff' in f])
    dfs[g]['time_opt'] = list([df for df, f in zip(dfs[g]['all'], files[g]['all']) if 'opt_time' in f])
    dfs[g]['time_real'] = list([df for df, f in zip(dfs[g]['all'], files[g]['all']) if 'real_dog_time' in f])
    vels[g]['fixed'] = np.array([vel for vel, f in zip(vels[g]['all'], files[g]['all']) if 'fixed' in f])
    vels[g]['foot_dep'] = np.array([vel for vel, f in zip(vels[g]['all'], files[g]['all']) if 'foot' in f])
    vels[g]['stiffness'] = np.array([vel for vel, f in zip(vels[g]['all'], files[g]['all']) if 'stiff' in f])
    vels[g]['time_opt'] = np.array([vel for vel, f in zip(vels[g]['all'], files[g]['all']) if 'opt_time' in f])
    vels[g]['time_real'] = np.array([vel for vel, f in zip(vels[g]['all'], files[g]['all']) if 'real_dog_time' in f])


def get_avgs(dfs, column_names, abs_val=False):
    n = (len(column_names), len(dfs))
    avgs = np.zeros(n, dtype=np.float32)
    stddevs = np.zeros(n, dtype=np.float32)
    for i, df in enumerate(dfs):
        if abs_val:
            avgs[:, i] = df[column_names].abs().mean()
        else:
            avgs[:, i] = df[column_names].mean()
        stddevs[:, i] = df[column_names].std()
    return avgs, stddevs


def get_avg_peaks(dfs, column_names, abs_val=False):
    n = (len(column_names), len(dfs))
    peaks = np.zeros(n, np.float32)
    for i, df in enumerate(dfs):
        for k, col in enumerate(column_names):
            if abs_val:
                idxs, _ = find_peaks(df[col].abs())
            else:
                idxs, _ = find_peaks(df[col])
            peaks[k, i] = df[col].iloc[idxs].mean()
    return peaks


def plot_avg_forces(gait, vel_unit='m/s'):
    columns = ['fl_grf', 'fr_grf', 'rl_grf', 'rr_grf']
    fig = plt.figure()
    ax_f = fig.add_subplot(121)
    ax_r = fig.add_subplot(122)
    for strat in strats:
        avgs, _ = get_avgs(dfs[gait][strat], columns)
        vel = vels[gait][strat]
        vel_inds = vel.argsort()
        avgs_sorted = avgs[:, vel_inds]
        vel_sorted = vel[vel_inds]
        front_avgs = (avgs_sorted[0] + avgs_sorted[1] / 2)
        rear_avgs = (avgs_sorted[2] + avgs_sorted[3] / 2)
        ax_f.plot(vel_sorted, front_avgs, linewidth=3., color=colours[strat], label=strat)
        ax_r.plot(vel_sorted, rear_avgs, linewidth=3., color=colours[strat], label=strat)
    ax_f.set_xlabel(f"Velocity ({vel_unit})")
    ax_f.set_ylabel("Mean Foot GRF (N)")
    ax_r.set_xlabel(f"Velocity ({vel_unit})")
    ax_r.set_ylabel("Mean Foot GRF (N)")
    ax_f.set_title(f"Mean Front Foot Force ({gait})")
    ax_r.set_title(f"Mean Rear Foot Force ({gait})")
    ax_f.legend()
    ax_r.legend()


def plot_peak_forces(gait, vel_unit='m/s'):
    columns = ['fl_grf', 'fr_grf', 'rl_grf', 'rr_grf']
    fig = plt.figure()
    ax_f = fig.add_subplot(121)
    ax_r = fig.add_subplot(122)
    for strat in strats:
        peaks = get_avg_peaks(dfs[gait][strat], columns)
        vel = vels[gait][strat]
        vel_inds = vel.argsort()
        peaks_sorted = peaks[:, vel_inds]
        vel_sorted = vel[vel_inds]
        front_peaks = np.maximum(peaks_sorted[0], peaks_sorted[1])
        rear_peaks = np.maximum(peaks_sorted[2], peaks_sorted[3])
        ax_f.plot(vel_sorted, front_peaks, linewidth=3., color=colours[strat], label=strat)
        ax_r.plot(vel_sorted, rear_peaks, linewidth=3., color=colours[strat], label=strat)
    ax_f.set_xlabel(f"Velocity ({vel_unit})")
    ax_f.set_ylabel("Peak Foot GRF (N)")
    ax_r.set_xlabel(f"Velocity ({vel_unit})")
    ax_r.set_ylabel("Peak Foot GRF (N)")
    ax_f.set_title(f"Peak Front Foot Force ({gait})")
    ax_r.set_title(f"Peak Rear Foot Force ({gait})")
    ax_f.legend()
    ax_r.legend()


def plot_feedforward_torques(gait, vel_unit='m/s'):
    columns = ['fft_spine_f_y', 'fft_spine_f_z', 'fft_spine_r_y', 'fft_spine_r_z']
    fig = plt.figure()
    ax_y = fig.add_subplot(121)
    ax_z = fig.add_subplot(122)
    for strat in strats:
        avgs, _ = get_avgs(dfs[gait][strat], columns, abs_val=True)
        vel = vels[gait][strat]
        vel_inds = vel.argsort()
        avgs_sorted = avgs[:, vel_inds]
        vel_sorted = vel[vel_inds]
        pitch_avgs = (avgs_sorted[0] + avgs_sorted[2]) / 2
        yaw_avgs = (avgs_sorted[1] + avgs_sorted[3]) / 2
        ax_y.plot(vel_sorted, pitch_avgs, linewidth=3., color=colours[strat], label=strat)
        ax_z.plot(vel_sorted, yaw_avgs, linewidth=3., color=colours[strat], label=strat)
    ax_y.set_xlabel(f"Velocity ({vel_unit})")
    ax_y.set_ylabel("Mean Absolute Feedforward Torque (Nm)")
    ax_z.set_xlabel(f"Velocity ({vel_unit})")
    ax_z.set_ylabel("Mean Absolute Feedforward Torque (Nm)")
    ax_y.set_title(f"Mean Absolute Feedforward Torque (Pitch) ({gait})")
    ax_z.set_title(f"Mean Absolute Feedforward Torque (Yaw) ({gait})")
    ax_y.legend()
    ax_z.legend()


def plot_spine_err(gait, vel_unit='m/s'):
    columns = ['err_spine_f_y', 'err_spine_f_z', 'err_spine_r_y', 'err_spine_r_z']
    fig = plt.figure()
    ax_fy = fig.add_subplot(221)
    ax_fz = fig.add_subplot(222)
    ax_ry = fig.add_subplot(223)
    ax_rz = fig.add_subplot(224)
    for strat in strats:
        if strat == 'fixed' or strat == 'stiffness':
            continue
        peaks = get_avg_peaks(dfs[gait][strat], columns, abs_val=True)
        vel = vels[gait][strat]
        vel_inds = vel.argsort()
        peaks_sorted = peaks[:, vel_inds]
        vel_sorted = vel[vel_inds]
        ax_fy.plot(vel_sorted, peaks_sorted[0, :], linewidth=3., color=colours[strat], label=strat)
        ax_fz.plot(vel_sorted, peaks_sorted[1, :], linewidth=3., color=colours[strat], label=strat)
        ax_ry.plot(vel_sorted, peaks_sorted[2, :], linewidth=3., color=colours[strat], label=strat)
        ax_rz.plot(vel_sorted, peaks_sorted[3, :], linewidth=3., color=colours[strat], label=strat)
    ax_fy.set_xlabel(f"Velocity ({vel_unit})")
    ax_fy.set_ylabel("Mean Absolute Joint Error (rad)")
    ax_fy.set_title(f"Mean Absolute Front Pitch Error ({gait})")
    ax_fz.set_xlabel(f"Velocity ({vel_unit})")
    ax_fz.set_ylabel("Mean Absolute Joint Error (rad)")
    ax_fz.set_title(f"Mean Absolute Front Yaw Error ({gait})")
    ax_ry.set_xlabel(f"Velocity ({vel_unit})")
    ax_ry.set_ylabel("Mean Absolute Joint Error (rad)")
    ax_ry.set_title(f"Mean Absolute Rear Pitch Error ({gait})")
    ax_rz.set_xlabel(f"Velocity ({vel_unit})")
    ax_rz.set_ylabel("Mean Absolute Joint Error (rad)")
    ax_rz.set_title(f"Mean Absolute Rear Yaw Error ({gait})")
    ax_fy.legend()
    ax_fz.legend()
    ax_ry.legend()
    ax_rz.legend()


def plot_tracking_err(gait, vel_unit='m/s'):
    columns = ['body_err_x', 'body_err_y', 'body_err_z', 'rot_err_x', 'rot_err_y', 'rot_err_z']
    fig_lin = plt.figure()
    ax_lx = fig_lin.add_subplot(131)
    ax_ly = fig_lin.add_subplot(132)
    ax_lz = fig_lin.add_subplot(133)
    fig_rot = plt.figure()
    ax_rx = fig_rot.add_subplot(131)
    ax_ry = fig_rot.add_subplot(132)
    ax_rz = fig_rot.add_subplot(133)
    for strat in strats:
        avgs, _ = get_avgs(dfs[gait][strat], columns, abs_val=True)
        vel = vels[gait][strat]
        vel_inds = vel.argsort()
        avgs_sorted = avgs[:, vel_inds]
        vel_sorted = vel[vel_inds]
        ax_lx.plot(vel_sorted, avgs_sorted[0, :], linewidth=3., color=colours[strat], label=strat)
        ax_ly.plot(vel_sorted, avgs_sorted[1, :], linewidth=3., color=colours[strat], label=strat)
        ax_lz.plot(vel_sorted, avgs_sorted[2, :], linewidth=3., color=colours[strat], label=strat)
        ax_rx.plot(vel_sorted, avgs_sorted[3, :], linewidth=3., color=colours[strat], label=strat)
        ax_ry.plot(vel_sorted, avgs_sorted[4, :], linewidth=3., color=colours[strat], label=strat)
        ax_rz.plot(vel_sorted, avgs_sorted[5, :], linewidth=3., color=colours[strat], label=strat)
    for ax in [ax_lx, ax_ly, ax_lz]:
        ax.set_xlabel(f"Velocity ({vel_unit})")
        ax.set_ylabel("Tracking Error (m)")
    for ax in [ax_rx, ax_ry, ax_rz]:
        ax.set_xlabel(f"Velocity ({vel_unit})")
        ax.set_ylabel("Tracking Error (rad)")
    ax_lx.set_title(f"Mean Absolute X-Axis Tracking Error ({gait})")
    ax_ly.set_title(f"Mean Absolute Y-Axis Tracking Error ({gait})")
    ax_lz.set_title(f"Mean Absolute Z-Axis Tracking Error ({gait})")
    ax_rx.set_title(f"Mean Absolute Roll-Axis Tracking Error ({gait})")
    ax_ry.set_title(f"Mean Absolute Pitch-Axis Tracking Error ({gait})")
    ax_rz.set_title(f"Mean Absolute Yaw-Axis Tracking Error ({gait})")
    for ax in [ax_lx, ax_ly, ax_lz, ax_rx, ax_ry, ax_rz]:
        ax.legend()


plot_avg_forces('Trot')
plot_avg_forces('Walk')
plot_avg_forces('Turn', 'rad/s')
plot_peak_forces('Trot')
plot_peak_forces('Walk')
plot_peak_forces('Turn', 'rad/s')
plot_feedforward_torques('Trot')
plot_feedforward_torques('Walk')
plot_feedforward_torques('Turn', 'rad/s')
plot_spine_err('Trot')
plot_spine_err('Walk')
plot_spine_err('Turn', 'rad/s')
plot_tracking_err('Trot')
plot_tracking_err('Walk')
plot_tracking_err('Turn', 'rad/s')
plt.show()
