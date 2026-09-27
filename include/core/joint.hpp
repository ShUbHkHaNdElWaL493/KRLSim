#pragma once

#include <string>

#include "types.hpp"

namespace krlsim
{
    enum class JointType
    {
        FIXED,
        REVOLUTE,
        PRISMATIC,
        CONTINUOUS,
        FLOATING,
        PLANAR,
        UNKNOWN
    };

    struct JointLimits
    {
        Scalar lower = 0.0;
        Scalar upper = 0.0;
        Scalar velocity = 0.0;
        Scalar effort = 0.0;
        bool has_position_limits = false;
        bool has_velocity_limits = false;
    };

    struct JointDescriptor
    {
        int joint_idx;
        std::string name;

        Isometry3 origin;
        Vector3 axis;

        JointType type;

        int parent_link_idx;
        int child_link_idx;

        int q_idx;
        int v_idx;

        JointLimits limits;
    };
}