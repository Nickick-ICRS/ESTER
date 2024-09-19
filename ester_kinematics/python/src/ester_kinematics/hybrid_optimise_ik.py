import numpy as np

from scipy.optimize import minimize, Bounds
from scipy.spatial.transform import Rotation

from .leg_kinematics import LegKinematicsSolver

class HybridIKSolver:
    def __init__(
        self, urdf_prep, done_tol, err_tol, max_its, pos_err_weight=1,
        torque_err_weight=0, power_err_weight=0, limits_override=None
    ):
        self._done_tol = done_tol
        self._err_tol = err_tol
        self._max_its = max_its
        self._pos_err_weight = pos_err_weight
        self._torque_err_weight = torque_err_weight
        self._power_err_weight = power_err_weight

        if limits_override is not None:
            self._limits = np.array(limits_override)
        else:
            self._limits = np.array(urdf_prep.joint_limits)
        self._eef_idxs = urdf_prep.eef_idxs
        self._spine_idxs = np.array(urdf_prep.spine_idxs)
        self._joint_idxs = np.array(urdf_prep.joint_angle_map)
        self._num_links = urdf_prep.num_links
        self._num_joints = urdf_prep.num_real_joints
        self._axis = np.array(urdf_prep.joint_axis)
        self._offsets = np.array(urdf_prep.offsets)

        self._children_map = urdf_prep.children_map
        self._parent_map = urdf_prep.parent_map
        self._link_names = urdf_prep.link_names

        self._angles = np.zeros((self._num_joints), dtype=float)

        self._shoulder_orientations = [[], [], [], []]
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

        lb = []
        ub = []
        kf = []
        for idx in self._spine_idxs:
            lb.append(self._limits[idx][0])
            ub.append(self._limits[idx][1])
            kf.append(True)
        self._bounds = Bounds(lb=lb, ub=ub, keep_feasible=kf)

        self._mass = 4

    def solve_ik(
        self, initial_angles, eef_targets, estimated_dt, solve_spine=True
    ):
        self._initial_angles = initial_angles
        self._angles = initial_angles
        self._dt = estimated_dt

        if not solve_spine:
            self._has_moved = True
            self._solve_leg_iks(eef_targets)
            return self._angles

        done_lim, err_lim, its = self._solve(eef_targets)

        if done_lim:
            print("All end effector targets achieved in {} iterations".format(its))
        elif err_lim:
            print("All end effectors converged in {} iterations".format(its))
        elif its >= self._max_its:
            print("Maximum iterations ({}) reached".format(its))
        else:
            print("Minimize converged in ({}) iterations.".format(its))
        return self._angles

    def _solve_fk(self):
        # Uses internal angles so will only recalculate if they changed
        if self._has_moved:
            self._positions = self.solve_fk(
                self._angles, store_shoulder_rots=True)
            self._has_moved = False
        return self._positions

    def solve_fk(self, angles, store_shoulder_rots=False):
        positions = np.zeros((self._num_links, 3))
        rot = Rotation.from_euler('xyz', [0, 0, 0])
        self._recurse_fk(
            0, angles, positions, rot, ssr=store_shoulder_rots)
        return positions

    def _recurse_fk(self, link_idx, angles, positions, R, ssr=False):
        parent_idx = self._parent_map[link_idx]
        joint_idx = self._joint_idxs[link_idx]
        if parent_idx:
            p = positions[parent_idx]
        else:
            p = np.array([0, 0, 0])

        offset = self._offsets[link_idx]
        children = self._children_map[link_idx]
        rot = R
        axis = rot.apply(self._axis[link_idx])
        positions[link_idx] = p + rot.apply(offset)
        if joint_idx != -1:
            # -1 means it's a fixed joint
            rot = Rotation.from_rotvec(angles[joint_idx]*axis) * rot

        for _, next_link_idx in children:
            if next_link_idx in self._shoulder_idxs:
                i = np.where(self._shoulder_idxs == next_link_idx)[0].item()
                idxs = self._leg_idxs[i]
                angs = angles[self._joint_idxs[idxs]]
                offset = self._offsets[next_link_idx]
                p = positions[link_idx]
                positions[next_link_idx] = p + rot.apply(offset)
                p = positions[next_link_idx]
                # Add one as knee joint -> foot pos
                positions[idxs+1] = p + rot.apply(self._leg_ks[i].solve_fk(angs))
                if ssr:
                    self._shoulder_orientations[i] = rot
            else:
                self._recurse_fk(
                    next_link_idx, angles, positions, rot, ssr=ssr)


    def _solve(self, tgts):
        self._it = 0
        self._done_lim = False
        self._err_lim = False
        self._best_err = np.inf
        self._last_err = np.inf
        self._recent_err = np.inf
        self._best_angs = self._angles.copy()
        self._tgts = tgts

        spine_jnt_idxs = self._joint_idxs[self._spine_idxs]
        X = np.zeros(self._spine_idxs.shape, float)
        X[:] = self._angles[spine_jnt_idxs]
        res = minimize(
            self._calculate_error, X, args=(tgts), bounds=self._bounds,
            callback=self._minimize_cb)

        #self._angles = self._best_angs

        return self._done_lim, self._err_lim, self._it

    def _calculate_error(self, X, tgts, recalculate=True):
        if not recalculate:
            return self._recent_err
        self._has_moved = True

        #self._angles[self._joint_idxs[self._spine_idxs]] = X
        self._angles[self._joint_idxs[self._spine_idxs]] = 0


        # tgts are (4, 4) -> (num_legs, [x, y, z, s]) where s is 1 in stance
        # and 0 in swing
        eef_pos = self._solve_leg_iks(tgts[:, :3])
        torques = self._estimate_joint_torques(tgts[:, 3])

        vels = (self._angles - self._initial_angles) / self._dt
        powers = torques * vels
        pos_err = np.mean(np.linalg.norm(tgts[:, :3] - eef_pos, axis=1)) \
                * self._pos_err_weight
        torque_err = np.mean(np.abs(torques)) * self._torque_err_weight
        power_err = np.mean(np.abs(powers)) * self._power_err_weight

        err = pos_err + torque_err + power_err
        self._recent_err = err
        return err

    def _minimize_cb(self, X):
        self._it += 1

        err = self._calculate_error(X, self._tgts, recalculate=False)
        if err < self._best_err:
            self._best_angs = self._angles.copy()
            if err < self._done_tol:
                self._done_lim = True
        else:
            if np.abs(err - self._last_err) < self._err_tol:
                self._err_lim = True
        self._last_err = err

        return self._done_lim or self._err_lim or self._it >= self._max_its

    def _solve_leg_iks(self, tgts):
        pos, rot = self._get_shoulder_transforms()
        eef_pos = []
        for i, tgt in enumerate(tgts):
            # Transform into shoulder frame
            tgt_transformed = rot[i].inv().apply(tgt - pos[i])
            jnt_idxs = self._joint_idxs[self._leg_idxs[i]]
            self._angles[jnt_idxs] = \
                self._leg_ks[i].solve_ik(tgt_transformed)
            eef_pos.append(
                pos[i] + rot[i].apply(self._leg_ks[i]._solve_fk())[2])
        return eef_pos

    def _get_shoulder_transforms(self):
        # Make sure fk has been solved
        self._solve_fk()
        return self._positions[self._shoulder_idxs], self._shoulder_orientations

    def _estimate_joint_torques(self, stance_array):
        # Force stances to be boolean array
        stances = stance_array > 0.5

        torques = np.zeros((self._num_joints), dtype=float)
        # We estimate legs as weightless for simplicity
        
        # Resolve vertically - all forces equal to mass * gravity
        force_sum = self._mass * 9.81

        shoulder_offsets, orientations = self._get_shoulder_transforms()

        fl = 0
        fr = 1
        rl = 2
        rr = 3
        forces = np.zeros(stances.shape, float)

        if np.sum(stances) == 1:
            leg_id = np.where(stances == True)[0]
            forces[leg_id] = force_sum
        elif np.sum(stances) == 2:
            ids = np.where(stances == True)
            if (stances[fl] and stances[rl]) or (stances[fr] and stances[rr]):
                # We have to solve moments about X
                # All forces in Z direction so ignore Z
                o0 = np.abs(shoulder_offsets[ids[0]][1])
                o1 = np.abs(shoulder_offsets[ids[1]][1])
            else:
                # We can solve moments about Y axis
                # All forces in Z direction so ignore Z
                o0 = np.abs(shoulder_offsets[ids[0]][0])
                o1 = np.abs(shoulder_offsets[ids[1]][0])
            F0 = (force_sum * o0 / o1) / (1 + o0 / o1)
            F1 = force_sum - F0

            if np.isnan(F0):
                F0 = force_sum / 2
                F1 = force_sum / 2

            forces[ids[0]] = F0
            forces[ids[1]] = F1
        elif np.sum(stances) == 3:
            ids = np.where(stances == True)[0]
            # Resolve front and back
            if fl not in ids:
                xf = shoulder_offsets[fr][1]
                xr = (shoulder_offsets[rl][1] + shoulder_offsets[rr][1])/2
                yl = shoulder_offsets[rl][0]
                yr = (shoulder_offsets[fr][0] + shoulder_offsets[rr][0])/2
            elif fr not in ids:
                xf = shoulder_offsets[fl][1]
                xr = (shoulder_offsets[rl][1] + shoulder_offsets[rr][1])/2
                yl = (shoulder_offsets[fl][0] + shoulder_offsets[rl][0])/2
                yr = shoulder_offsets[rr][0]
            elif rl not in ids:
                xf = (shoulder_offsets[fl][1] + shoulder_offsets[fr][1])/2
                xr = shoulder_offsets[rr][1]
                yl = shoulder_offsets[fl][0]
                yr = (shoulder_offsets[fr][0] + shoulder_offsets[rr][0])/2
            elif rr not in ids:
                xf = (shoulder_offsets[fl][1] + shoulder_offsets[fr][1])/2
                xr = shoulder_offsets[rl][1]
                yl = (shoulder_offsets[fl][0] + shoulder_offsets[rl][0])/2
                yr = shoulder_offsets[fr][0]
            xf = np.abs(xf)
            xr = np.abs(xr)
            yl = np.abs(yl)
            yr = np.abs(yr)

            Fx0 = (force_sum * xf / xr) / (1 + xf / xr)
            Fx1 = force_sum - Fx0
            Fy0 = (force_sum * yl / yr) / (1 + yl / yr)
            Fy1 = force_sum - Fy0

            if np.isnan(Fx0):
                Fx0 = force_sum / 2
                Fx1 = force_sum / 2
            if np.isnan(Fy0):
                Fy0 = force_sum / 2
                Fy1 = force_sum / 2

            # Now assign to the relevant joints
            if fl not in ids:
                forces[fr] = Fx0
                forces[rl] = Fy0
                forces[rr] = Fx1 - Fy0
            elif fr not in ids:
                forces[fl] = Fx0
                forces[rl] = Fx1 - Fy1
                forces[rr] = Fy1
            elif rl not in ids:
                forces[fl] = Fy0
                forces[fr] = Fx0 - Fy0
                forces[rr] = Fx1
            else:
                forces[fl] = Fx0 - Fy1
                forces[fr] = Fy1
                forces[rl] = Fx1
        else:
            # All 4 feet touching ground
            xf = np.abs(shoulder_offsets[fl][1] + shoulder_offsets[fr][1])/2
            xr = np.abs(shoulder_offsets[rl][1] + shoulder_offsets[rr][1])/2
            yl = np.abs(shoulder_offsets[fl][0] + shoulder_offsets[rl][0])/2
            yr = np.abs(shoulder_offsets[fr][0] + shoulder_offsets[rr][0])/2

            Fx0 = (force_sum * xf / xr) / (1 + xf / xr)
            Fx1 = force_sum - Fx0
            Fy0 = (force_sum * yl / yr) / (1 + yl / yr)
            Fy1 = force_sum - Fy0

            if np.isnan(Fx0):
                Fx0 = force_sum / 2
                Fx1 = force_sum / 2
            if np.isnan(Fy0):
                Fy0 = force_sum / 2
                Fy1 = force_sum / 2

            # Ffl + Ffr = Fx0
            # Ffl + Frl = Fy0
            # Ffr + Frr = Fy1
            # Frl + Frr = Fx1
            # This is only 3 equations with 4 unknowns so we need one more
            # constraint. Lets assume that the front right and rear left
            # legs have equal moments about the COM about an arbitrary axis
            fr_dist = np.linalg.norm(shoulder_offsets[fr][:2])
            rl_dist = np.linalg.norm(shoulder_offsets[rl][:2])
            # Ffr = Frl * rl_dist / fr_dist
            Frl = (Fx0 - Fy0) / (rl_dist / fr_dist - 1)
            Frr = Fx1 - Frl
            Ffl = Fy0 - Frl
            Ffr = Fx0 - Ffl

            forces[fl] = Ffl
            forces[fr] = Ffr
            forces[rl] = Frl
            forces[rr] = Frr
            
        for i in range(4):
            idxs = self._joint_idxs[self._leg_idxs[i]]
            torques[idxs] = self._leg_ks[i].estimate_torques(
                forces[i], orientations[i])
        return torques
