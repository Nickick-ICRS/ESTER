import numpy as np

from scipy.spatial.transform import Rotation

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
        self._angles = np.zeros((3), dtype=float)
        self._positions = np.zeros((3, 3), dtype=float)
        self._side = side
        self._has_moved = True

        # Pre-calculate some values for ik
        self._y_target = np.sum(self._offsets, axis=0)[1]
        self._leg_dists = [
            np.sqrt((self._offsets[0][1] + self._offsets[1][1] + self._offsets[2][1]) ** 2 + self._offsets[0][2] ** 2),
            np.sqrt(self._offsets[1][0] ** 2 + self._offsets[1][2] ** 2),
            np.sqrt(self._offsets[2][0] ** 2 + self._offsets[2][2] ** 2)
        ]
        self._offset_th = [
            np.pi - np.arctan2(self._offsets[0][1], self._offsets[0][2]),
            np.pi - np.arctan2(self._offsets[1][0], self._offsets[1][2]),
            np.pi - np.arctan2(self._offsets[2][0], self._offsets[2][2])
        ]
        self._max_dist = self._leg_dists[1] + self._leg_dists[2]
        self._ld_squared = self._leg_dists[1] ** 2 + self._leg_dists[2] ** 2

    def solve_fk(self, thetas):
        """
        @brief Solved wrt shoulder
        """
        R = Rotation.from_rotvec(thetas[0] * self._axis[0])
        p0 = R.apply(self._offsets[0])

        R = Rotation.from_rotvec(thetas[1] * self._axis[1]) * R
        p1 = p0 + R.apply(self._offsets[1])

        R = Rotation.from_rotvec(thetas[2] * self._axis[2]) * R
        p2 = p1 + R.apply(self._offsets[2])

        positions = np.zeros((3, 3), dtype=float)

        positions[0, :] = p0
        positions[1, :] = p1
        positions[2, :] = p2

        return positions

    def _solve_fk(self):
        if not self._has_moved:
            return self._positions

        self._positions = self.solve_fk(self._angles)
        self._has_moved = False
        return self._positions

    def solve_ik(self, foot_target):
        """
        @param foot_target foot xyz target _in the  hip frame_
        """
        tx = foot_target[0]
        # We mirror about xz plane in the case of right legs
        ty = foot_target[1] * self._side
        tz = foot_target[2]

        l0 = self._leg_dists[0]
        l1 = self._leg_dists[1]
        l2 = self._leg_dists[2]

        th0 = np.arctan2(tz, ty) + np.arctan2(
            np.sqrt(ty ** 2 + tz ** 2 - l0 ** 2), l0)

        c0 = np.cos(th0)
        s0 = np.sin(th0)

        k2 = tx ** 2 + ty ** 2 + tz ** 2 + l0 ** 2 - 2 * l0 * (
            ty * c0 + tz * s0)

        c1 = (l1 ** 2 + k2 - l2 ** 2) / (2 * l1 * np.sqrt(k2))
        c2 = (l1 ** 2 + l2 ** 2 - k2) / (2 * l1 * l2)
        th1 = np.arccos(min(max(c1, -1), 1))
        th2 = np.pi - np.arccos(min(max(c2, -1), 1))

        if self._pref_pos_knee:
            th1 = -th1
        else:
            th2 = -th2

        # Account for heading of foot wrt to shoulder
        th1 = th1  - np.arctan2(tx, l0 * s0 - tz)
        # We mirror about xz plane in the case of right legs
        th0 *= self._side

        self._angles[0] = th0
        self._angles[1] = th1
        self._angles[2] = th2

        self._has_moved = True

        return th0, th1, th2

    def solve_ik_old(self, foot_target):
        """
        @param foot_target foot xyz target _in the hip frame_
        """
        # We know that roll is the only joint that affects the y position
        # of the target
        # Find the theta that rotates the target to align with the y
        # position of the foot

        a, b = (foot_target[1], foot_target[2])
        r = np.sqrt(a ** 2 + b ** 2)
        thphi = np.arccos(self._y_target / r)
        phi = np.arccos(a / r)
        phi_check = np.arcsin(b / r)
        phi_check2 = np.pi - phi_check if phi_check >= 0 else -np.pi - phi_check
        if np.isclose(phi, phi_check) or np.isclose(phi, phi_check2):
            pass
        elif np.isclose(-phi, phi_check) or np.isclose(-phi, phi_check2):
            # Take other side of the function
            phi = -phi
        else:
            print("No match between sin and cos!", phi, phi_check, phi_check2)

        # thphi is theta - phi
        # Note the correct angle could also be -thphi
        th_pos = thphi + phi
        th_neg = -thphi + phi
        l = self._limits[0][0]
        u = self._limits[0][1]
        pos_err = np.abs(min(max(-th_pos, l), u) + th_pos)
        neg_err = np.abs(min(max(-th_neg, l), u) + th_neg)
        if neg_err < pos_err:
            th0 = min(max(th_neg, l), u)
        else:
            th0 = min(max(th_pos, l), u)

        # Now we can rotate the target into the relevant plane to solve for
        # x and z
        R = Rotation.from_rotvec(th0 * self._axis[0])

        # Rotate and shift into hip_fe frame
        T = R.inv().apply(foot_target) - self._offsets[0]

        # We can't affect Y anymore, so we only care about X and Z
        dist = np.linalg.norm(T[[0, 2]])

        # First solve for knee
        if dist < self._max_dist:
            cos = (self._ld_squared - dist ** 2) / \
                  (2 * self._leg_dists[1] * self._leg_dists[2])
            th = np.pi - np.arccos(max(min(cos, 1), -1))

            lower = self._limits[2][0]
            upper = self._limits[2][1]
            off = self._offset_th[2]
            pos_in_lim = lower <= th - off and th - off <= upper
            neg_in_lim = lower <= -th - off and -th - off <= upper
            if pos_in_lim and neg_in_lim:
                if self._pref_pos_knee:
                    th2 = th
                else:
                    th2 = -th
            elif pos_in_lim:
                th2 = th
            else:
                th2 = -th
        else:
            th2 = 0 # Maximum distance we can reach

        # Now solve for hip pitch
        x = -self._leg_dists[1] * np.sin(th2)
        z = -self._leg_dists[2] - self._leg_dists[1] * np.cos(th2)
        vec = np.array([x, z])
        tgt = np.array([T[0], T[2]])
        vec /= np.linalg.norm(vec)
        tgt /= np.linalg.norm(tgt)
        th1 = np.arccos(np.dot(vec, tgt))
        sign = (vec[0] * tgt[1]) - (vec[1] * tgt[0])
        if sign > 0:
            th1 = -th1

        th1 = min(max(
            th1-self._offset_th[1], self._limits[1][0]), self._limits[1][1])
        th2 = min(max(
            th2-self._offset_th[2], self._limits[2][0]), self._limits[2][1])

        self._angles[0] = th0
        self._angles[1] = th1
        self._angles[2] = th2

        self._has_moved = True

        pos = self.solve_fk([th0, th1, th2])

        return th0, th1, th2

    def estimate_torques(self, force, R):
        if np.abs(force) <= 1e-6:
            return np.zeros((3), dtype=float)
        # shoulder may be offset, so we need to rotate the joint positions
        # relative to the shoulder rotation
        pos = R.apply(self._solve_fk())

        # Force is straight down, so we only care about y or x components
        M0 = pos[0, 1] * force # Rotates about x-axis so we care about Y
        M1 = pos[1, 0] * force # Rotates about y-axis so we care about X
        M2 = pos[1, 0] * force # Rotates about y-axis so we care about X

        return np.array([M0, M1, M2])
