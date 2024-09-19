import numpy as np
import copy

from .parallel_leg_kinematics import LegKinematicsSolver

class ParallelKinematicsSolver:
    def __init__(self, urdf_prep, limits_override=None):
        if limits_override is not None:
            self._limits = np.array(limits_override)
        else:
            self._limits = np.array(urdf_prep.joint_limits)
        self._eef_idxs = np.array(urdf_prep.eef_idxs)
        self._spine_idxs = np.array(urdf_prep.spine_idxs)
        self._joint_idxs = np.array(urdf_prep.joint_angle_map)
        self._num_links = urdf_prep.num_links
        self._num_joints = urdf_prep.num_real_joints
        self._axis = np.array(urdf_prep.joint_axis)
        self._offsets = np.array(urdf_prep.offsets)

        self._children_map = urdf_prep.children_map
        self._parent_map = urdf_prep.parent_map
        self._link_names = urdf_prep.link_names

        self._leg_idxs = []
        for i in range(4):
            foot = urdf_prep.chains[i][1]
            knee = self._parent_map[foot]
            hip_pitch = self._parent_map[knee]
            hip_roll = self._parent_map[hip_pitch]
            self._leg_idxs.append(
                np.array([hip_roll, hip_pitch, knee, foot], dtype=int))

        self._shoulder_idxs = np.zeros((4), dtype=int)
        self._leg_ks = []
        sides = [1, -1, 1, -1]
        for i in range(4):
            idxs = self._leg_idxs[i]
            self._shoulder_idxs[i] = idxs[0]
            self._leg_ks.append(LegKinematicsSolver(
                self._offsets[idxs[1:]], self._axis[idxs[:3]],
                self._limits[idxs[:3]], i > 1, sides[i]))
            self._leg_idxs[i] = idxs[:3]

    def solve_fk(self, angles):
        """
        @brief Solves full robot fk, returning positions of the feet
        @param angles The angles (n, 16)
        @returns The foot positions (n, 4, 3) (FL, FR, RL, RR)
        """
        n = angles.shape[0]
        shoulder_poses = self.solve_shoulder_pose(angles[:, [0, 1, 8, 9]])
        foot_poses = np.ones((n, 4, 4))
        foot_poses[:, 0, :3] = self._leg_ks[0].solve_fk(angles[:, 2:5])[:, -1]
        foot_poses[:, 1, :3] = self._leg_ks[1].solve_fk(angles[:, 5:8])[:, -1]
        foot_poses[:, 2, :3] = self._leg_ks[2].solve_fk(angles[:, 10:13])[:, -1]
        foot_poses[:, 3, :3] = self._leg_ks[3].solve_fk(angles[:, 13:16])[:, -1]
        # Transform to base link frame
        foot_poses = np.matmul(
            shoulder_poses, foot_poses.reshape((n, 4, 4, 1))).squeeze()
        return foot_poses[:, :, :3]

    def solve_ik(self, initial_angles, eef_targets):
        """
        @brief Solves leg ik, without solving the spine
        @param initial_angles The starting angles for IK solve (n, 16)
        @param eef_targets The end effector target positions in base link
                           frame (n, 4, 3)
        @details target order is FL, FR, RL, RR
                 angle order is F_SPINE, FL, FR, R_SPINE, RL, RR
        @returns joint angles (n, 16)
        """
        n = eef_targets.shape[0]
        if len(initial_angles.shape) != 2:
            initial_angles = initial_angles.reshape((-1, 16)).repeat(n, axis=0)
        # (n, 4) SE3 matrices (n, 4, 4, 4)
        shoulder_pose = self.solve_shoulder_pose(
            initial_angles[:, [0, 1, 8, 9]])
        # We want the inverse of the shoulder pose
        shoulder_pose[:, :, :3, :3] = np.transpose(
            shoulder_pose[:, :, :3, :3], axes=[0, 1, 3, 2])
        # R.T @ t
        shoulder_pose[:, :, :3, 3] = -np.matmul(
            shoulder_pose[:, :, :3, :3],
            shoulder_pose[:, :, :3, 3].reshape((n, 4, 3, 1))).squeeze()
        # Transform targets to shoulder frame
        eef_t_homo = np.c_[eef_targets, np.ones((n, 4, 1))].reshape(
            (n, 4, 4, 1))
        eef_t_sf = np.matmul(shoulder_pose, eef_t_homo).squeeze()[:, :, :3]

        # We don't change spine angles so we can just deepcopy the spine
        # angles
        ja = copy.deepcopy(initial_angles)
        ja[:, 2:5] = self._leg_ks[0].solve_ik(eef_t_sf[:, 0, :])
        ja[:, 5:8] = self._leg_ks[1].solve_ik(eef_t_sf[:, 1, :])
        ja[:, 10:13] = self._leg_ks[2].solve_ik(eef_t_sf[:, 2, :])
        ja[:, 13:16] = self._leg_ks[3].solve_ik(eef_t_sf[:, 3, :])

        return ja

    def solve_shoulder_pose(self, angles):
        """
        @brief Find the pose of the shoulders wrt base link
        @param angles The current spine angles (n, 4)
        @returns Shoulder pose matrices (n, 4, 4, 4) (FL, FR, RL, RR)
        """
        angles = angles.reshape((-1, 4))
        n = angles.shape[0]

        axis = self._axis[self._spine_idxs]
        trans = self._offsets[self._spine_idxs]
        poses = np.zeros((n, 4, 4, 4))
        poses[:, :] = np.eye(4)
        poses[:, 0] = self.transformation_matrices(
            axis[1], angles[:, 1], trans[1].reshape(-1, 3).repeat(n, axis=0)
        ) @ self.transformation_matrices(
            axis[0], angles[:, 0], trans[0].reshape(-1, 3).repeat(n, axis=0)
        )
        poses[:, 1] = poses[:, 0]
        poses[:, 2] = self.transformation_matrices(
            axis[2], angles[:, 2], trans[2].reshape(-1, 3).repeat(n, axis=0)
        ) @ self.transformation_matrices(
            axis[3], angles[:, 3], trans[3].reshape(-1, 3).repeat(n, axis=0)
        )
        poses[:, 3] = poses[:, 2]
        hip_offsets0 = self._offsets[self._leg_idxs[0][0]].reshape((-1, 1))
        hip_offsets1 = self._offsets[self._leg_idxs[1][0]].reshape((-1, 1))
        hip_offsets2 = self._offsets[self._leg_idxs[2][0]].reshape((-1, 1))
        hip_offsets3 = self._offsets[self._leg_idxs[3][0]].reshape((-1, 1))
        poses[:, 0, :3, 3] += (poses[:, 0, :3, :3] @ hip_offsets0).squeeze()
        poses[:, 1, :3, 3] += (poses[:, 1, :3, :3] @ hip_offsets1).squeeze()
        poses[:, 2, :3, 3] += (poses[:, 2, :3, :3] @ hip_offsets2).squeeze()
        poses[:, 3, :3, 3] += (poses[:, 3, :3, :3] @ hip_offsets3).squeeze()
        poses[:, :, 3, 3] = 1

        return poses

    def transformation_matrices(self, axis, thetas, translations):
        """
        @brief Creates a homogenous transformation matrix
        @param axis Axis of rotation (3)
        @param thetas Rotation angle (n)
        @param translations Translation (n, 3)
        @returns homogenous transformation matrices (n, 4, 4)
        """
        assert translations.shape[0] == thetas.shape[0]
        n = thetas.shape[0]
        T = np.zeros((n, 4, 4))
        c = np.cos(thetas)
        c_1 = 1 - c
        s = np.sin(thetas)
        T[:, 0, 0] = c + axis[0] ** 2 * c_1 
        T[:, 0, 1] = axis[0] * axis[1] * c_1 - axis[2] * s 
        T[:, 0, 2] = axis[0] * axis[2] * c_1 + axis[1] * s 
        T[:, 1, 0] = axis[1] * axis[0] * c_1 + axis[2] * s 
        T[:, 1, 1] = c + axis[1] ** 2 * c_1 
        T[:, 1, 2] = axis[1] * axis[2] * c_1 - axis[0] * s 
        T[:, 2, 0] = axis[2] * axis[0] * c_1 - axis[1] * s 
        T[:, 2, 1] = axis[2] * axis[1] * c_1 - axis[0] * s 
        T[:, 2, 2] = c + axis[2] ** 2 * c_1 
        T[:, :3, 3] = translations
        T[:, 3, 3] = 1

        return T
