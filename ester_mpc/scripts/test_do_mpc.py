import do_mpc
from casadi import *
from liecasadi import SO3, SE3, SO3Tangent, SE3Tangent, DualQuaternion
import time

import rospy
from urdf_parser_py.urdf import URDF
import rospkg
rospack = rospkg.RosPack()

from ester_kinematics.hybrid_optimise_ik import HybridIKSolver
from ester_kinematics.ik_utils import *
from ester_walking_gait.trajectory_planner import TrajectoryPlanner

from ester_mpc.mpc_rviz_vis import MPCRvizVis

rospy.init_node("ester_mpc")

time_horizon = 10
dt = 0.05

ester_description = rospack.get_path("ester_description")
with open(os.path.join(ester_description, 'urdf/ester.urdf'), 'r') as f:
    urdf = URDF.from_xml_string(f.read())
eef_names = [
    "front_left_foot_contact_link",
    "front_right_foot_contact_link",
    "rear_left_foot_contact_link",
    "rear_right_foot_contact_link",
]
urdf_prep = URDFPrep(urdf, 'spine_center_link', eef_names)
initial_angles = np.zeros(16)
hybrid = HybridIKSolver(urdf_prep, 1e-4, 1e-8, 1e3)
zero_pos = hybrid.solve_fk(initial_angles)
update_freq = 1/dt
trajectory_planner = TrajectoryPlanner(
    update_freq, zero_pos[urdf_prep.eef_idxs])
cmd = np.array([0.1, 0, 0, 0.25])
trajs = trajectory_planner.generate_trajectories(cmd[0], [0.25, 0.5, 0.75])
for i in range(4):
    z_min = min(trajs[i, :, 2])
    # Set feet to be zero at the minima (center of stride)
    delta = -cmd[3] - z_min
    trajs[i, :, 2] += delta

cfg = {
    "horizon_dt" : dt,
    "num_steps" : time_horizon,
    "foot_marker_scale" : 0.05,
    "grf_marker_scale" : 0.1,
    "body_x" : 0.4,
    "body_y" : 0.2,
    "body_z" : 0.25,
    "max_its" : 15,
}

mpc = MPCRvizVis(cfg, urdf_prep)
mpc.set_foot_trajectories(trajs)

simulator = do_mpc.simulator.Simulator(mpc._model)
simulator.set_param(t_step = dt)

tvp_temp2 = simulator.get_tvp_template()
def get_next_foot_pos_sim(t_now):
    # For now crash at the end of the trajectories
    t = int(t_now / dt)
    mod = trajs.shape[1]
    data = np.zeros((4, 4))
    for i in range(4):
         data[i, :] = trajs[i][t % mod, :]
    tvp_temp2['foot_ref'] = data
    tvp_temp2['cmd_vel'] = cmd[:3]
    return tvp_temp2


simulator.set_tvp_fun(get_next_foot_pos_sim)
simulator.setup()

x0 = simulator.x0.cat.full()
# Avoid zero as can cause singularities
x0[:] = 1e-3
# Set initial height correct
x0[2] = 0#cmd[3]
x0[9] = 0#cmd[3]
# Set quaternion to unit quaternion
x0[3:6] = 0
x0[6] = 1
# Initial foot positions
x0[13:16] = trajs[0][0, :3].reshape(3, 1)
x0[16:19] = trajs[1][0, :3].reshape(3, 1)
x0[19:22] = trajs[2][0, :3].reshape(3, 1)
x0[22:25] = trajs[3][0, :3].reshape(3, 1)
mpc.set_x0(x0)
simulator.x0 = x0

mpc.set_initial_guess()

import matplotlib.pyplot as plt
mpc_graphics = do_mpc.graphics.Graphics(mpc._mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

fig, ax = plt.subplots(2, sharex=True)
fig.align_ylabels()

for g in [sim_graphics]:#, mpc_graphics]:
    g.add_line(var_type='_u', var_name='grf_xy', axis=ax[0])
    g.add_line(var_type='_u', var_name='grf_z', axis=ax[1])


ax[0].set_ylabel('ground xy reaction force [N]')
ax[1].set_ylabel('ground z reaction force [N]')
ax[1].set_xlabel('time [s]')

msgs, topics = mpc.get_ros_outputs()
pubs = []
for m, t in zip(msgs, topics):
    pubs.append(rospy.Publisher(t, m, queue_size=1))

## FOR PRINT DEBUGGING
from ester_mpc.srbm import *
##

simulator.reset_history()
simulator.x0 = x0
mpc._mpc.reset_history()
rate = rospy.Rate(1/dt)
for i in range(100):
    rate.sleep()
    start = time.time()
    msgs, u0 = mpc.update(x0)
    dur = time.time() - start
    print("MPC Update rate: ", 1/dur, " Hz")
    x0 = simulator.make_step(u0)
    rp = x0[0:3].T
    rq = x0[3:7].T
    rp_1 = x0[7:10].T
    v = rp - rp_1
    w = x0[10:13].T
    rp_t = x0[25:28].T
    grf_xy = u0[0:8].reshape((4,2))
    grf_z = u0[8:12].reshape((4,))
    forces = np.zeros((4, 3))
    forces[:, :2] = grf_xy
    forces[:, 2] = grf_z
    foot_p = x0[13:25].reshape((3, 4)).T
    tvp = get_next_foot_pos_sim(simulator.t0)
    s = tvp['foot_ref'][:, 3]
    print("rp: ", rp, "\nrq: ", rq, "\nv: ", v, "\nw: ", w,
          "\ntgt: ", rp_t,
          "\na: ", sum1(forces).T / mpc.m_rbd + np.array([0, 0, -9.81]),
          "\npos_err: ", m_norm(rp_t - rp) * 1e0,
          "\nvel_err: ", (m_norm(v) + m_norm(w)) * 1e-1,
          "\nrot_err: ", (1 - (Quaternion(np.array([0, 0, 0, 1])).inverse()
                            * Quaternion(rq.T)).coeffs()[3] ** 2) * 1e2,
          "\nz_f_err: ", (sum1(forces[:, 2]) - mpc.m_rbd * 9.81) ** 2 * 1e-2,
          "\nswing_err: ", (m_norm(forces[:, 0] * (1-s)) \
                          + m_norm(forces[:, 1] * (1-s)) \
                          + m_norm(forces[:, 2] * (1-s))) * 1e-1,
          "\nforces_x: ", forces[:, 0],
          "\nforces_y: ", forces[:, 1],
          "\nforces_z: ", forces[:, 2],
          "\nfoot_s: ", s)
    for i, m in enumerate(msgs):
        pubs[i].publish(m)
    # Break on ctrl-c
    if rospy.is_shutdown():
        break

mpc_graphics.plot_predictions(t_ind=0)
sim_graphics.plot_results()
sim_graphics.reset_axes()
plt.show()
