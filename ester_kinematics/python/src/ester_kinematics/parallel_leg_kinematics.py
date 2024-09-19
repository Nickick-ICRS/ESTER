import numpy as np

class LegKinematicsSolver:
    def __init__(
        self, link_parent_offsets, joint_axis, joint_limits, pref_pos_knee,
        side
    ):
        """
        @param link_parent_offset 3x3 array of offsets of each link from its
                                  parent. I.e. Hip Roll->Hip Pitch
                                               Hip Pitch->Knee
                                               Knee->Foot
        @param joint_axis 3x3 array of default joint axis I.e. [1 0 0]
                                                               [0 1 0]
                                                               [0 1 0]
        @param joint_limits 3x2 array of joint limits [[lower, upper], ...]
        @param pref_pos_knee True if we prefer a positive knee angle
        @param side +1 for left legs, -1 for right legs
        """
        assert len(joint_axis) == 3
        assert len(link_parent_offsets) == 3
        assert len(joint_limits) == 3

        # We make axis assumptions to enable analytical solutions
        assert np.all(joint_axis[0] == np.array([1, 0, 0]))
        assert np.all(joint_axis[1] == np.array([0, 1, 0]))
        assert np.all(joint_axis[2] == np.array([0, 1, 0]))

        self._offsets = link_parent_offsets
        self._axis = joint_axis
        self._limits = joint_limits
        self._pref_pos_knee = pref_pos_knee
        self._side = side

        # Pre-calculate some values for ik
        self._leg_dists = [
            np.sqrt((self._offsets[0][1] + self._offsets[1][1] + self._offsets[2][1]) ** 2 + self._offsets[0][2] ** 2),
            np.sqrt(self._offsets[1][0] ** 2 + self._offsets[1][2] ** 2),
            np.sqrt(self._offsets[2][0] ** 2 + self._offsets[2][2] ** 2)
        ]

    def solve_fk(self, thetas):
        """
        @brief Solved wrt shoulder
        @param thetas roll, pitch, knee (n, 3)
        @returns positions wrt shoulder (n, 3, 3)
        """
        thetas = thetas.reshape((-1, 3))
        n = thetas.shape[0]

        p = np.zeros((n, 3, 3), dtype=float)

        R = self.axis_angle(self._axis[0], thetas[:, 0])
        p[:, 0, :] = np.matmul(R, self._offsets[0].reshape((3, 1))).squeeze()

        R = np.matmul(self.axis_angle(self._axis[1], thetas[:, 1]), R)
        p[:, 1, :] = p[:, 0, :] + np.matmul(R, self._offsets[1].reshape((3, 1))).squeeze()

        R = np.matmul(self.axis_angle(self._axis[2], thetas[:, 2]), R)
        p[:, 2, :] = p[:, 1, :] + np.matmul(R, self._offsets[2].reshape((3, 1))).squeeze()

        return p

    def solve_ik(self, foot_targets):
        """
        @param foot_target foot xyz target _in the  hip frame_ (n, 3)
        @returns thetas (n, 3)
        """
        foot_targets = foot_targets.reshape((-1, 3))
        thts = np.zeros_like(foot_targets)

        tx = foot_targets[:, 0]
        # We mirror about xz plane in the case of right legs
        ty = foot_targets[:, 1] * self._side
        tz = foot_targets[:, 2]

        l0 = self._leg_dists[0]
        l1 = self._leg_dists[1]
        l2 = self._leg_dists[2]

        s = np.maximum(ty ** 2 + tz ** 2 - l0 ** 2, np.zeros_like(ty))
        thts[:, 0] = np.arctan2(tz, ty) + np.arctan2(np.sqrt(s), l0)

        c0 = np.cos(thts[:, 0])
        s0 = np.sin(thts[:, 0])

        k2 = tx ** 2 + ty ** 2 + tz ** 2 + l0 ** 2 - 2 * l0 * (
            ty * c0 + tz * s0)

        c1 = (l1 ** 2 + k2 - l2 ** 2) / (2 * l1 * np.sqrt(k2))
        c2 = (l1 ** 2 + l2 ** 2 - k2) / (2 * l1 * l2)
        np.maximum(c1, -np.ones_like(c1), out=c1)
        np.maximum(c2, -np.ones_like(c2), out=c2)
        np.minimum(c1, np.ones_like(c1), out=c1)
        np.minimum(c2, np.ones_like(c2), out=c2)
        thts[:, 1] = np.arccos(c1)
        thts[:, 2] = np.pi - np.arccos(c2)

        if self._pref_pos_knee:
            thts[:, 1] = -thts[:, 1]
        else:
            thts[:, 2] = -thts[:, 2]

        # Account for heading of foot wrt to shoulder
        thts[:, 1] = thts[:, 1]  - np.arctan2(tx, l0 * s0 - tz)
        # We mirror about xz plane in the case of right legs
        thts[:, 0] *= self._side

        return thts

    def solve_id(self, thetas, grf):
        """
        @brief Calculate joint torques given joint positions and a reaction
               force at the end effector
        @details We model the legs themselves as weightless links, so
                 each link is simply manipulating a point mass
        @param thetas The joint positions (n, 3)
        @param grf The ground reaction force in the shoulder frame (n, 3)
        @returns The estimated joint torques (n, 3)
        """
        # (n, 3, 3)
        joint_positions = self.solve_fk(angles)

        T = inertias * acc + C * vel + grf * pos

    def axis_angle(self, axis, thetas):
        """
        @param create rotation matrices from axis and angle
        @param axis The axis (3)
        @param thetas The thetas (n)
        """
        n = thetas.shape[0]
        R = np.zeros((n, 3, 3))
        c = np.cos(thetas)
        c_1 = 1 - c
        s = np.sin(thetas)
        R[:, 0, 0] = c + axis[0] ** 2 * c_1
        R[:, 0, 1] = axis[0] * axis[1] * c_1 - axis[2] * s
        R[:, 0, 2] = axis[0] * axis[2] * c_1 + axis[1] * s
        R[:, 1, 0] = axis[1] * axis[0] * c_1 + axis[2] * s
        R[:, 1, 1] = c + axis[1] ** 2 * c_1
        R[:, 1, 2] = axis[1] * axis[2] * c_1 - axis[0] * s
        R[:, 2, 0] = axis[2] * axis[0] * c_1 - axis[1] * s
        R[:, 2, 1] = axis[2] * axis[1] * c_1 - axis[0] * s
        R[:, 2, 2] = c + axis[2] ** 2 * c_1
        return R
