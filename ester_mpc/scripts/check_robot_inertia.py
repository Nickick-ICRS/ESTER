from ester_kinematics.ik_utils import *

import rospkg
rospack = rospkg.RosPack()
from urdf_parser_py.urdf import URDF

import os
import numpy as np
from scipy.spatial.transform import Rotation

FRONT_PITCH = np.radians(0)
FRONT_YAW = np.radians(0)
REAR_PITCH = np.radians(0)
REAR_YAW = np.radians(0)

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
I_a = np.array(urdf_prep.inertias[0])
m_a = urdf_prep.masses[0]
com_a = np.array(urdf_prep.coms[0])

I_0 = np.array(urdf_prep.inertias[urdf_prep.spine_idxs[0]])
m_0 = urdf_prep.masses[urdf_prep.spine_idxs[0]]
com_0 = urdf_prep.coms[urdf_prep.spine_idxs[0]]

I_1 = np.array(urdf_prep.inertias[urdf_prep.spine_idxs[1]])
m_1 = urdf_prep.masses[urdf_prep.spine_idxs[1]]
com_1 = urdf_prep.coms[urdf_prep.spine_idxs[1]]

I_2 = np.array(urdf_prep.inertias[urdf_prep.spine_idxs[2]])
m_2 = urdf_prep.masses[urdf_prep.spine_idxs[2]]
com_2 = urdf_prep.coms[urdf_prep.spine_idxs[2]]

I_3 = np.array(urdf_prep.inertias[urdf_prep.spine_idxs[3]])
m_3 = urdf_prep.masses[urdf_prep.spine_idxs[3]]
com_3 = urdf_prep.coms[urdf_prep.spine_idxs[3]]

def T(R, t):
    m = np.zeros((4, 4))
    m[3, 3] = 1
    m[:3, :3] = R.as_matrix()
    m[:3, 3] = t;
    return m

def parallel_axis(v):
    m = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            if j == i:
                # We can index -1, -2
                m[i, j] = v[i-1] ** 2 + v[i-2] ** 2
            else:
                m[i, j] = -v[i] * v[j]
    return m

def transform_I(I, Torig, Tnew):
    I_orig = Torig[:3, :3].T @ parallel_axis(-Torig[:3, 3]) @ I
    I_new = parallel_axis(Tnew[:3, 3]) @ Tnew[:3, :3] @ I
    return I_new

R0 = Rotation.from_rotvec(np.array([0, 1, 0]) * FRONT_PITCH)
T0 = T(R0, urdf_prep.offsets[urdf_prep.spine_idxs[0]])

R1 = Rotation.from_rotvec(np.array([0, 0, 1]) * FRONT_YAW)
T1 = T(R1, urdf_prep.offsets[urdf_prep.spine_idxs[1]]) @ T0

R2 = Rotation.from_rotvec(np.array([0, 1, 0]) * REAR_PITCH)
T2 = T(R2, urdf_prep.offsets[urdf_prep.spine_idxs[2]])

R3 = Rotation.from_rotvec(np.array([0, 0, 1]) * REAR_YAW)
T3 = T(R3, urdf_prep.offsets[urdf_prep.spine_idxs[3]]) @ T2

coms = [com_a, com_0, com_1, com_2, com_3]
cs = np.zeros((4, len(coms)))
for i in range(len(coms)):
    cs[:3, i] = coms[i]
cs[3, :] = 1

Ts = [np.eye(4), T0, T1, T2, T3]
for i in range(len(Ts)):
    cs[:, i] = Ts[i] @ cs[:, i]

ms = np.array([[m_a, m_0, m_1, m_2, m_3]])

m_rbd = np.sum(ms)
com_rbd = (ms @ cs.T).squeeze()[:3] / m_rbd

# Transform all inertia matrices relative to I_a
Is = [I_a, I_0, I_1, I_2, I_3]
for i in range(len(Is)):
    Is[i] = transform_I(Is[i], np.eye(4), Ts[i])
# Sum
I_rbd_a = I_a + I_0 + I_1 + I_2 + I_3

# Transform to com_rbd
T_com = T(Rotation.from_quat([0, 0, 0, 1]), com_rbd)
I_rbd = transform_I(I_rbd_a, np.eye(4), T_com)

# Print results
print("Mass of spine:\n", m_rbd)
print("COM of spine:\n", com_rbd)
print("Inertia of spine:\n", I_rbd)
print("Total mass of robot:\n", np.sum(np.array(urdf_prep.masses)))
