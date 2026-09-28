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
    };

    struct JointDescriptor
    {
        int joint_index;
        std::string name;
        JointType type;
        Isometry3 origin;
        int parent_link_index;
        int child_link_index;
        Vector3 axis;
        JointLimits limits;
    };
}