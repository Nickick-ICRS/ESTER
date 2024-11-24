import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import os

CONTACT_THRESHOLD = 2.
ZERO_THRESHOLD = 1e-3


def get_foot_contacts(fd: np.array, fu: np.array):
    assert len(fd) == len(fu)
    contacts = np.zeros(len(fd))

    current_stance = False
    for i in range(len(fd)):
        if fd[i] != 0:
            current_stance = True
        if fu[i] != 0:
            current_stance = False
        contacts[i] = 1 if current_stance else 0
    return contacts


def get_cycle_starts(contacts: np.array):
    rising_edges = np.where((contacts[:-1] <= ZERO_THRESHOLD) & (contacts[1:] >= ZERO_THRESHOLD))[0]
    # We want the point that it starts, not the idx before
    return rising_edges + 1


def make_fixed_len_array(contacts: np.array, start: int, end: int):
    N = 100
    L = end - start
    if L == N:
        return contacts[start:end]
    new_pos = np.linspace(0, L-1, N)
    interpolated = np.interp(new_pos, np.arange(L), contacts[start:end])
    interpolated[interpolated >= 0.5] = 1.
    interpolated[interpolated <= 0.5] = 0.
    return interpolated


def plot_hildebrand_df(df: pd.DataFrame, name: str):
    fl_contacts = get_foot_contacts(df['fl_fd_x'].to_numpy(), df['fl_lo_x'].to_numpy())
    fr_contacts = get_foot_contacts(df['fr_fd_x'].to_numpy(), df['fr_lo_x'].to_numpy())
    rl_contacts = get_foot_contacts(df['rl_fd_x'].to_numpy(), df['rl_lo_x'].to_numpy())
    rr_contacts = get_foot_contacts(df['rr_fd_x'].to_numpy(), df['rr_lo_x'].to_numpy())
    idxs = get_cycle_starts(fl_contacts)
    avg_fl_contacts = np.zeros(100)
    avg_fr_contacts = np.zeros(100)
    avg_rl_contacts = np.zeros(100)
    avg_rr_contacts = np.zeros(100)
    N = len(idxs) - 1.

    for i, idx in enumerate(idxs):
        if i == len(idxs)-1:
            break
        next_idx = idxs[i+1]
        avg_fl_contacts += make_fixed_len_array(fl_contacts, idx, next_idx) / N
        avg_fr_contacts += make_fixed_len_array(fr_contacts, idx, next_idx) / N
        avg_rl_contacts += make_fixed_len_array(rl_contacts, idx, next_idx) / N
        avg_rr_contacts += make_fixed_len_array(rr_contacts, idx, next_idx) / N
    plot_hildebrand(avg_fl_contacts, avg_fr_contacts, avg_rl_contacts, avg_rr_contacts, name)


def plot_hildebrand(avg_fl_contacts, avg_fr_contacts, avg_rl_contacts, avg_rr_contacts, name):
    timestamps = np.linspace(0, 1, len(avg_fl_contacts))
    probabilities = list(reversed([avg_fl_contacts, avg_fr_contacts, avg_rl_contacts, avg_rr_contacts]))
    labels = list(reversed(["Front Left", "Front Right", "Hind Left", "Hind Right"]))

    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 2))

    for i, (prob, label) in enumerate(zip(probabilities, labels)):
        y = np.full_like(timestamps, i)  # Assign a row for each leg
        # Map stance probability to grayscale intensity
        colors = plt.cm.gray(1 - prob)  # White for swing, black for stance
        for j in range(len(timestamps) - 1):
            ax.fill_between(
                timestamps[j:j+2],
                y[j:j+2] - 0.4, y[j:j+2] + 0.4,
                color=colors[j],
                edgecolor=None,
                step="post"
            )
        ax.text(1.05, i, label, va='center')  # Add label next to rows

    # Beautify the plot
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.5, len(probabilities) - 0.5)
    ax.set_yticks([])
    ax.set_xticks(np.linspace(0, 1, 5))
    ax.set_xticklabels(["0%", "25%", "50%", "75%", "100%"])
    ax.set_xlabel("Gait Cycle (%)")
    ax.set_title("Hildebrand Plot with Uncertainty ({})".format(name))
    plt.tight_layout()


this_path = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.join(this_path, 'raw_data')
all_files = os.listdir(data_dir)

files = {}
files['Trot'] = {}
files['Walk'] = {}
files['Turn'] = {}
files['Trot']['all'] = list([f for f in all_files if 'trot_0.65' in f])
files['Walk']['all'] = list([f for f in all_files if 'walk_0.3' in f and 'turning' not in f])
files['Turn']['all'] = list([f for f in all_files if 'walk_0.3_turning' in f])

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


def load_df(f: str, expected_vel: float):
    ACCELERATION = 0.5
    df = pd.read_csv(os.path.join(data_dir, f))
    time_to_reach_max_speed = expected_vel / ACCELERATION
    # Drop data from acceleration phase
    return df[df['timestamp'] >= time_to_reach_max_speed * 1.1]


for s in strats:
    df = load_df(files['Walk'][s][0], 0.35)
    plot_hildebrand_df(df, "Walk: {}".format(s))
    df = load_df(files['Trot'][s][0], 0.65)
    plot_hildebrand_df(df, "Trot: {}".format(s))
#    df = load_df(files['Turn'][s][0], 0.35)
#    plot_hildebrand_df(df, "Turn: {}".format(s))


target_fl_walk = np.zeros(100)
target_fl_walk[0:75] = 1
target_fr_walk = np.zeros(100)
target_fr_walk[0:25] = 1
target_fr_walk[50:100] = 1
target_rl_walk = np.zeros(100)
target_rl_walk[25:100] = 1
target_rr_walk = np.zeros(100)
target_rr_walk[0:50] = 1
target_rr_walk[75:100] = 1

target_fl_trot = np.zeros(100)
target_fl_trot[0:48] = 1
target_fr_trot = np.zeros(100)
target_fr_trot[50:98] = 1
target_rl_trot = np.zeros(100)
target_rl_trot[50:98] = 1
target_rr_trot = np.zeros(100)
target_rr_trot[0:48] = 1

plot_hildebrand(target_fl_walk, target_fr_walk, target_rl_walk, target_rr_walk, "Walk: Requested Cycle")
plot_hildebrand(target_fl_trot, target_fr_trot, target_rl_trot, target_rr_trot, "Trot: Requested Cycle")

plt.show()
