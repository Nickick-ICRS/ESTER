#ifndef __ESTER_ENUMS_HPP__
#define __ESTER_ENUMS_HPP__

#include <sstream>

#include <unordered_map>
#include <array>

namespace ester_common {

enum class LegId {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
};

inline const std::array<LegId, 4> ALL_LEG_IDS {
    LegId::FL,
    LegId::FR,
    LegId::RL,
    LegId::RR,
};

template<typename T>
using LegMap = std::unordered_map<LegId, T>;

enum class LegJointId {
    HIP_ROLL  = 0,
    HIP_PITCH = 1,
    KNEE      = 2,
};

inline const std::array<LegJointId, 3> ALL_LEG_JOINTS {
    LegJointId::HIP_ROLL,
    LegJointId::HIP_PITCH,
    LegJointId::KNEE,
};

template<typename T>
using LegJointMap = std::unordered_map<LegJointId, T>;

enum class SpineJointId {
    FRONT_PITCH = 0,
    FRONT_YAW   = 1,
    REAR_PITCH  = 2,
    REAR_YAW    = 3,
};

inline const std::array<SpineJointId, 4> ALL_SPINE_JOINTS {
    SpineJointId::FRONT_PITCH,
    SpineJointId::FRONT_YAW,
    SpineJointId::REAR_PITCH,
    SpineJointId::REAR_YAW,
};

template<typename T>
using SpineJointMap = std::unordered_map<SpineJointId, T>;

enum class AllJointId {
    F_SPINE_PITCH     = 0,
    F_SPINE_YAW       = 1,
    FL_HIP_ROLL       = 2,
    FL_HIP_PITCH      = 3,
    FL_KNEE           = 4,
    FR_HIP_ROLL       = 5,
    FR_HIP_PITCH      = 6,
    FR_KNEE           = 7,
    R_SPINE_PITCH     = 8,
    R_SPINE_YAW       = 9,
    RL_HIP_ROLL       = 10,
    RL_HIP_PITCH      = 11,
    RL_KNEE           = 12,
    RR_HIP_ROLL       = 13,
    RR_HIP_PITCH      = 14,
    RR_KNEE           = 15,
};

inline const std::array<AllJointId, 16> ALL_JOINT_IDS {
    AllJointId::F_SPINE_PITCH,
    AllJointId::F_SPINE_YAW,
    AllJointId::FL_HIP_ROLL,
    AllJointId::FL_HIP_PITCH,
    AllJointId::FL_KNEE,
    AllJointId::FR_HIP_ROLL,
    AllJointId::FR_HIP_PITCH,
    AllJointId::FR_KNEE,
    AllJointId::R_SPINE_PITCH,
    AllJointId::R_SPINE_YAW,
    AllJointId::RL_HIP_ROLL,
    AllJointId::RL_HIP_PITCH,
    AllJointId::RL_KNEE,
    AllJointId::RR_HIP_ROLL,
    AllJointId::RR_HIP_PITCH,
    AllJointId::RR_KNEE,
};

template<typename T>
using AllJointMap = std::unordered_map<AllJointId, T>;

inline AllJointId full_joint_id(const SpineJointId &spine) {
    switch(spine) {
    case SpineJointId::FRONT_PITCH:
        return AllJointId::F_SPINE_PITCH;
    case SpineJointId::FRONT_YAW:
        return AllJointId::F_SPINE_YAW;
    case SpineJointId::REAR_PITCH:
        return AllJointId::R_SPINE_PITCH;
    case SpineJointId::REAR_YAW:
        return AllJointId::R_SPINE_YAW;
    }
    // Shouldn't be able to get here
    throw;
    return AllJointId::F_SPINE_PITCH;
}

inline AllJointId full_joint_id(const LegId &leg, const LegJointId &jnt) {
    switch(leg) {
    case LegId::FL:
        switch(jnt) {
        case LegJointId::HIP_PITCH:
            return AllJointId::FL_HIP_PITCH;
        case LegJointId::HIP_ROLL:
            return AllJointId::FL_HIP_ROLL;
        case LegJointId::KNEE:
            return AllJointId::FL_KNEE;
        }
    case LegId::FR:
        switch(jnt) {
        case LegJointId::HIP_PITCH:
            return AllJointId::FR_HIP_PITCH;
        case LegJointId::HIP_ROLL:
            return AllJointId::FR_HIP_ROLL;
        case LegJointId::KNEE:
            return AllJointId::FR_KNEE;
        }
    case LegId::RL:
        switch(jnt) {
        case LegJointId::HIP_PITCH:
            return AllJointId::RL_HIP_PITCH;
        case LegJointId::HIP_ROLL:
            return AllJointId::RL_HIP_ROLL;
        case LegJointId::KNEE:
            return AllJointId::RL_KNEE;
        }
    case LegId::RR:
        switch(jnt) {
        case LegJointId::HIP_PITCH:
            return AllJointId::RR_HIP_PITCH;
        case LegJointId::HIP_ROLL:
            return AllJointId::RR_HIP_ROLL;
        case LegJointId::KNEE:
            return AllJointId::RR_KNEE;
        }
    }
    // Shouldn't be able to get here
    throw;
    return AllJointId::F_SPINE_PITCH;
}

inline AllJointId operator+(const LegId &leg, const LegJointId &jnt) {
    return full_joint_id(leg, jnt);
}

inline std::string str(const LegId &id) {
    switch(id) {
    case LegId::FL:
        return "FL";
    case LegId::FR:
        return "FR";
    case LegId::RL:
        return "RL";
    case LegId::RR:
        return "RR";
    default:
        return "UNKNOWN LEG ID";
    }
}

inline std::string str(const LegJointId &id) {
    switch(id) {
    case LegJointId::HIP_ROLL:
        return "HIP_ROLL";
    case LegJointId::HIP_PITCH:
        return "HIP_PITCH";
    case LegJointId::KNEE:
        return "KNEE";
    default:
        return "UNKNOWN LEG JOINT ID";
    }
}

inline std::string str(const SpineJointId &id) {
    switch(id) {
    case SpineJointId::FRONT_PITCH:
        return "FRONT_PITCH";
    case SpineJointId::FRONT_YAW:
        return "FRONT_YAW";
    case SpineJointId::REAR_PITCH:
        return "REAR_PITCH";
    case SpineJointId::REAR_YAW:
        return "REAR_YAW";
    default:
        return "UNKNOWN SPINE JOINT ID";
    }
}

inline std::ostream &operator<<(std::ostream &os, const LegId &rhs) {
    os << str(rhs);
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const LegJointId &rhs) {
    os << str(rhs);
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const SpineJointId &rhs) {
    os << str(rhs);
    return os;
}

inline std::string get_link_name(const LegId &leg, const LegJointId &joint)
{
    std::string front;
    std::string back;
    switch(leg) {
    case LegId::FL:
        front = "front_left_";
        break;
    case LegId::FR:
        front = "front_right_";
        break;
    case LegId::RL:
        front = "rear_left_";
        break;
    case LegId::RR:
        front = "rear_right_";
        break;
    default:
        front = "UNKNOWN LEG ID";
        break;
    }
    switch(joint) {
    case LegJointId::HIP_ROLL:
        back = "hip_fe_link";
        break;
    case LegJointId::HIP_PITCH:
        back = "upper_leg_link";
        break;
    case LegJointId::KNEE:
        back = "lower_leg_link";
        break;
    default:
        back = "UNKNOWN LEG JOINT";
        break;
    }
    return front + back;
}

inline std::string str(const AllJointId &jnt) {
    switch(jnt) {
    case AllJointId::F_SPINE_PITCH:
        return "front_spine_pitch_joint";
    case AllJointId::F_SPINE_YAW:
        return "front_spine_yaw_joint";
    case AllJointId::FL_HIP_ROLL:
        return "front_left_hip_fe_joint";
    case AllJointId::FL_HIP_PITCH:
        return "front_left_upper_leg_joint";
    case AllJointId::FL_KNEE:
        return "front_left_lower_leg_joint";
    case AllJointId::FR_HIP_ROLL:
        return "front_right_hip_fe_joint";
    case AllJointId::FR_HIP_PITCH:
        return "front_right_upper_leg_joint";
    case AllJointId::FR_KNEE:
        return "front_right_lower_leg_joint";
    case AllJointId::R_SPINE_PITCH:
        return "rear_spine_pitch_joint";
    case AllJointId::R_SPINE_YAW:
        return "rear_spine_yaw_joint";
    case AllJointId::RL_HIP_ROLL:
        return "rear_left_hip_fe_joint";
    case AllJointId::RL_HIP_PITCH:
        return "rear_left_upper_leg_joint";
    case AllJointId::RL_KNEE:
        return "rear_left_lower_leg_joint";
    case AllJointId::RR_HIP_ROLL:
        return "rear_right_hip_fe_joint";
    case AllJointId::RR_HIP_PITCH:
        return "rear_right_upper_leg_joint";
    case AllJointId::RR_KNEE:
        return "rear_right_lower_leg_joint";
    default:
        return "UNKNOWN ALLJOINTID!";
    }
}

inline std::ostream &operator<<(std::ostream &os, const AllJointId &rhs) {
    os << str(rhs);
    return os;
}

}; // ns ester_common

#endif // __ESTER_ENUMS_HPP__
