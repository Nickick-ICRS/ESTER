import os
import time
import numpy as np

from urdf_parser_py.urdf import URDF
import rospkg
rospack = rospkg.RosPack()

from ester_kinematics.hybrid_optimise_ik import *
from ester_kinematics.ik_utils import *

ester_description = rospack.get_path("ester_description")
with open(os.path.join(ester_description, 'urdf/ester.urdf'), 'r') as f:
    urdf = URDF.from_xml_string(f.read())
eef_names = [
    "front_left_foot_contact_link",
    "front_right_foot_contact_link",
    "rear_left_foot_contact_link",
    "rear_right_foot_contact_link",
]
urdf_prep = URDFPrep(urdf, 'base_link', eef_names)

initial_joint_angles = np.zeros((16), dtype=float)
# 3 reachable, one not reachable
tgts = np.array([[ 0.15,  0.13, -0.25, 1],
                 [ 0.25, -0.11, -0.25, 1],
                 [-0.12,  0.15, -0.22, 1],
                 [-0.14, -0.11, -0.35, 1]])

hybrid = HybridIKSolver(urdf_prep, 1e-4, 1e-8, 1e3)
    
init = hybrid.solve_fk(initial_joint_angles)
do_visualise(
    init, urdf_prep.children_map, urdf_prep.eef_idxs,
    tgts, color=(0.8, 0.8, 0.8), draw=False)
start = time.time()
ja = hybrid.solve_ik(initial_joint_angles, tgts, 1/100)
elapsed = time.time() - start
pos = hybrid.solve_fk(ja)
p, r = hybrid._get_shoulder_transforms()
for i in range(4):
    o = p[i]
    x = p[i] + r[i].apply(np.array([0.1, 0, 0]))
    y = p[i] + r[i].apply(np.array([0, 0.1, 0]))
    z = p[i] + r[i].apply(np.array([0, 0, 0.1]))
    draw_line(o, x, color=(1, 0, 0))
    draw_line(o, y, color=(0, 1, 0))
    draw_line(o, z, color=(0, 0, 1))
print("Total solve time {} s / {} hz".format(elapsed, 1. / elapsed))
print("Errors: ", np.linalg.norm(pos[urdf_prep.eef_idxs] - tgts[:, :3], axis=1))
print("Distances: ", pos[urdf_prep.eef_idxs] - tgts[:, :3])
do_visualise(
    pos, urdf_prep.children_map, urdf_prep.eef_idxs,
    tgts, color=(0.2, 0.2, 0.2), draw=True)
