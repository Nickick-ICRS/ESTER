import numpy as np
from scipy.spatial.transform import Rotation

from mayavi import mlab


# ====================== ROBOT REPRESENTATION ======================
class Link:
    def __init__(self, offset, rotation_axis, joint_limits, parent_link):
        self.offset = offset
        self.axis = rotation_axis
        self.limits = joint_limits
        self.parent = parent_link

class URDFPrep:
    def __init__(self, urdf, root_name, eef_names):
        self.base_idx = 0
        self.num_links = 0
        self.num_real_joints = 0
        self.targets = []
        self.target_colours = []
        self.offsets = []
        self.joint_axis = []
        self.joint_limits = []
        self.joint_angles = []
        self.joint_angle_map = []
        self.children_map = []
        self.parent_map = []
        self.eef_idxs = []
        self.eef_names = eef_names
        self.links = []
        self.link_names = [] # for debugging
        self.chains = []
        self.coms = []
        self.inertias = []
        self.masses = []

        self.shoulder_idxs = []
        self.shoulder_names = [
            "front_left_hip_fe_link",
            "front_right_hip_fe_link",
            "rear_left_hip_fe_link",
            "rear_right_hip_fe_link",
        ]

        self.spine_idxs = []
        self.spine_names = [
            "front_spine_pitch_link",
            "front_spine_yaw_link",
            "rear_spine_pitch_link",
            "rear_spine_yaw_link",
        ]


        self._prepare_urdf(urdf, root_name, eef_names)

    def _prepare_urdf(self, urdf, root_name, eef_names):
        self.eef_idxs = np.zeros((len(eef_names)), dtype=int)
        self.shoulder_idxs = np.zeros((len(self.shoulder_names)), dtype=int)
        self.spine_idxs = np.zeros((len(self.spine_names)), dtype=int)

        root = urdf.link_map[root_name]
        link = Link(np.array([0, 0, 0]), np.array([0, 0, 0]), [0, 0], None)
        self.links.append(link)
        self.link_names.append(root_name)
        self.coms.append(np.array(root.inertial.origin.xyz))
        self.inertias.append(root.inertial.inertia.to_matrix())
        self.masses.append(root.inertial.mass)

        self._add_children_to_links(urdf, root_name, link)
        self._construct_chain()
        self._construct_sub_chains()

    def _add_children_to_links(
        self, urdf, parent_name, parent_link, offset=np.array([0, 0, 0]),
        rot_offset=None
    ):
        for (joint_name, link_name) in urdf.child_map[parent_name]:
            joint = urdf.joint_map[joint_name]
            link = urdf.link_map[link_name]
            # TODO Combine link inertial properties
            com = np.array(link.inertial.origin.xyz)
            inertia = link.inertial.inertia.to_matrix()
            mass = link.inertial.mass
            rot = Rotation.from_euler('xyz', joint.origin.rotation)
            if rot_offset is not None:
                rot = rot * rot_offset
            if joint.type == 'revolute':
                l = Link(
                    offset + rot.apply(joint.origin.position), joint.axis,
                    [joint.limit.lower, joint.limit.upper], parent_link)
                self.links.append(l)
                self.link_names.append(link_name)
                self.coms.append(com)
                self.inertias.append(inertia)
                self.masses.append(mass)
                if link_name in self.eef_names:
                    i = self.eef_names.index(link_name)
                    self.eef_idxs[i] = len(self.links) - 1
                elif link_name in self.shoulder_names:
                    i = self.shoulder_names.index(link_name)
                    self.shoulder_idxs[i] = len(self.links) - 1
                elif link_name in self.spine_names:
                    i = self.spine_names.index(link_name)
                    self.spine_idxs[i] = len(self.links) - 1
                try:
                    urdf.child_map[link_name]
                except KeyError:
                    continue
                self._add_children_to_links(urdf, link_name, l)
            elif joint.type == 'fixed':
                end_of_chain = link_name in self.eef_names
                try:
                    urdf.child_map[link_name]
                except KeyError:
                    end_of_chain = True
                if end_of_chain:
                    # We need to add it if it has no children or is an eef
                    l = Link(
                        offset + rot.apply(joint.origin.position),
                        np.array([0, 0, 0]), [0, 0], parent_link)
                    self.links.append(l)
                    self.link_names.append(link_name)
                    self.coms.append(com)
                    self.inertias.append(inertia)
                    self.masses.append(mass)
                    if link_name in self.eef_names:
                        i = self.eef_names.index(link_name)
                        self.eef_idxs[i] = len(self.links) - 1
                    elif link_name in self.shoulder_names:
                        i = self.shoulder_names.index(link_name)
                        self.shoulder_idxs[i] = len(self.links) - 1
                    elif link_name in self.spine_names:
                        i = self.spine_names.index(link_name)
                        self.spine_idxs[i] = len(self.links) - 1
                else:
                    # Combine with next link
                    new_offset = offset + joint.origin.position
                    self._add_children_to_links(
                        urdf, link_name, parent_link, offset=new_offset,
                        rot_offset=rot)

    def _construct_chain(self):
        self.num_links = 0
        for link in self.links:
            idx = self.num_links
            self.num_links += 1
            self.offsets.append(link.offset)
            self.joint_axis.append(link.axis)
            # 0 0 0 means it's a fixed joint
            if np.linalg.norm(link.axis) > 1e-5:
                self.joint_angle_map.append(self.num_real_joints)
                self.num_real_joints += 1
            else:
                self.joint_angle_map.append(-1)
            self.joint_limits.append(link.limits)
            self.children_map.append([])
            if link.parent:
                parent_idx = -1
                for i in range(len(self.links)):
                    if self.links[i] == link.parent:
                        parent_idx = i
                        break
                if parent_idx == -1:
                    raise "Link parent not a member of links"
                self.parent_map.append(parent_idx)
                self.children_map[parent_idx].append((idx, idx))
            else:
                self.parent_map.append(None)

    def _construct_sub_chains(self):
        sub_base_idxs = []
        for end_idx in self.eef_idxs:
            parent_idx = self.parent_map[end_idx]
            while len(self.children_map[parent_idx]) == 1:
                parent_idx = self.parent_map[parent_idx]
            self.chains.append((parent_idx, end_idx))
            if parent_idx not in sub_base_idxs:
                sub_base_idxs.append(parent_idx)
        while len(sub_base_idxs):
            new_vec = []
            for end_idx in sub_base_idxs:
                parent_idx = self.parent_map[end_idx]
                while len(self.children_map[parent_idx]) == 1 and parent_idx != 0:
                    parent_idx = self.parent_map[parent_idx]
                self.chains.append((parent_idx, end_idx))
                if parent_idx != 0 and parent_idx not in new_vec:
                    new_vec.append(parent_idx)
            sub_base_idxs = new_vec


# ===================== RECURSIVE FORWARD KINEMATICS =====================
class FK:
    def __init__(self, urdf_prep):
        self.parent_map = urdf_prep.parent_map
        self.offsets = urdf_prep.offsets
        self.children_map = urdf_prep.children_map
        self.axis = urdf_prep.joint_axis
        self.num_links = urdf_prep.num_links

    def solve_fk(self, joint_angles):
        self.link_positions = np.zeros((self.num_links, 3))
        self.joint_angles = joint_angles
        R = Rotation.from_euler('xyz', [0, 0, 0])
        self._forwards_kinematics(0, 0, R)
        return self.link_positions

    def _forwards_kinematics(self, idx, joint_idx, R):
        # Note: Only works with fixed and rotational joints
        parent_idx = self.parent_map[idx]
        if parent_idx:
            p = self.link_positions[parent_idx]
        else:
            p = np.array([0, 0, 0])
        offset = self.offsets[idx]
        children = self.children_map[idx]
        rot = R
        axis = rot.apply(self.axis[joint_idx])
        self.link_positions[idx] = p + rot.apply(offset)
        rot = Rotation.from_rotvec(self.joint_angles[joint_idx]*axis)*rot

        for next_joint_idx, next_link_idx in children:
            self._forwards_kinematics(next_link_idx, next_joint_idx, rot)


# ====================== VISUALISATIONS ======================
def draw_dot(p, color=(0.5, 0.5, 0.5), sf=0.02):
    mlab.points3d(p[0], p[1], p[2], scale_factor=sf, color=color)


def draw_line(A, B, color=(0.5, 0.5, 0.5)):
    mlab.plot3d(
        [A[0], B[0]], [A[1], B[1]], [A[2], B[2]], tube_radius=0.005,
        color=color)


def show_plot():
    mlab.show(stop=False)


def visualise_chain(
    idx, link_positions, children_map, eef_idxs, break_lines=None,
    visualise=True, color=(0.5, 0.5, 0.5)
):
    # If break lines is not none, don't draw a line between [idx_A, idx_B]
    children = children_map[idx]
    base_position = link_positions[idx]
    if idx not in eef_idxs:
        draw_dot(base_position, color=color)

        draw_line([0, 0, 0], [0.1, 0, 0], color=(1, 0, 0))
        draw_line([0, 0, 0], [0, 0.1, 0], color=(0, 1, 0))
        draw_line([0, 0, 0], [0, 0, 0.1], color=(0, 0, 1))
    else:
        for i, eidx in enumerate(eef_idxs):
            if eidx == idx:
                draw_dot(base_position, color=(1, 0, 0))
    for joint_idx, link_idx in children:
        child_position = link_positions[link_idx]
        if not break_lines or [idx, link_idx] not in break_lines:
            draw_line(base_position, child_position, color=color)
        visualise_chain(
            link_idx, link_positions, children_map, eef_idxs,
            break_lines=break_lines, visualise=False, color=color)

    if visualise:
        for t, c in zip(targets, target_colours):
            draw_dot(t, color=c)
        draw_dot([0, 0, 0], color=(0, 0, 0))
        show_plot()


def do_visualise(
    link_positions, children_map, eef_idx, eef_targets,
    color=(0.3, 0.3, 0.3), draw=True
):
    visualise_chain(
        0, link_positions, children_map, eef_idx, color=color,
        visualise=False)
    for t in eef_targets:
        draw_dot(t, color=(0, 0, 1))
    draw_dot(np.array([0, 0, 0]), color=(0, 0, 0))

    if draw:
        show_plot()


# ======================== MATHS UTILITIES ==========================
def find_angle_between_vectors_about_axis(A, B, axis):
    # Project vectors onto a plane perpendicular to axis
    A_p = A - np.dot(A, axis) / np.dot(axis, axis) * axis
    B_p = B - np.dot(B, axis) / np.dot(axis, axis) * axis
    th = np.arccos(np.dot(A_p, B_p) / (np.linalg.norm(A_p) * np.linalg.norm(B_p)))
    if np.dot(np.cross(A_p, B_p), axis) < 0:
        th = -th
    return th


def clamp(a, upper, lower):
    return max(min(a, upper), lower)
