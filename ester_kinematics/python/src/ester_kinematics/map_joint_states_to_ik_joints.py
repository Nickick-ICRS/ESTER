import numpy as np
from sensor_msgs.msg import JointState

joint_name_to_link_name_map = {
    "front_left_hip_fe_joint" : "front_left_hip_fe_link",
    "front_left_upper_leg_joint" : "front_left_upper_leg_link",
    "front_left_lower_leg_joint" : "front_left_lower_leg_link",
    "front_right_hip_fe_joint" : "front_right_hip_fe_link",
    "front_right_upper_leg_joint" : "front_right_upper_leg_link",
    "front_right_lower_leg_joint" : "front_right_lower_leg_link",
    "front_spine_pitch_joint" : "front_spine_pitch_link",
    "front_spine_yaw_joint" : "front_spine_yaw_link",
    "rear_left_hip_fe_joint" : "rear_left_hip_fe_link",
    "rear_left_upper_leg_joint" : "rear_left_upper_leg_link",
    "rear_left_lower_leg_joint" : "rear_left_lower_leg_link",
    "rear_right_hip_fe_joint" : "rear_right_hip_fe_link",
    "rear_right_upper_leg_joint" : "rear_right_upper_leg_link",
    "rear_right_lower_leg_joint" : "rear_right_lower_leg_link",
    "rear_spine_pitch_joint" : "rear_spine_pitch_link",
    "rear_spine_yaw_joint" : "rear_spine_yaw_link",
}

class MapJointStatesToIKJointStates:
    def __init__(self, ik, joint_states):
        """
        @param ik An intialised IK solver
        @param joint_states A received joint states message
        """
        self._js_to_ik_map = {}
        self._ik_to_js_map = {}
        for idx, name in enumerate(joint_states.name):
            for link_idx, ik_name in enumerate(ik._link_names):
                if ik_name == joint_name_to_link_name_map[name]:
                    ik_idx = ik._joint_idxs[link_idx]
                    self._js_to_ik_map[idx] = ik_idx
                    self._ik_to_js_map[ik_idx] = idx

    def js_to_ik_idx(self, idx):
        return self._js_to_ik_map[idx]

    def ik_to_js_idx(self, idx):
        return self._ik_to_js_map[idx]

    def js_to_ik(self, js):
        ik_js = np.zeros(16)
        if type(js) == JointState:
            js = js.position
        for i, p in enumerate(js):
            ik_js[self.js_to_ik_idx(i)] = p
        return ik_js

    def ik_to_js(self, ik_js):
        js = np.zeros(16)
        for i, p in enumerate(ik_js):
            js[self.ik_to_js_idx(i)] = p
        return js
