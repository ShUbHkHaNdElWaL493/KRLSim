#pragma once

#include <string>
#include <variant>
#include <vector>

#include "types.hpp"

namespace krlsim
{
    struct BoxGeometry { Vector3 size; };
    struct SphereGeometry { Scalar radius; };
    struct CylinderGeometry { Scalar length; Scalar radius; };
    struct MeshGeometry { std::string filename; Vector3 scale; };

    struct CollisionShape
    {
        std::string name;
        std::variant<BoxGeometry, SphereGeometry, CylinderGeometry, MeshGeometry> geometry;
        Isometry3 origin;
    };

    struct LinkDescriptor
    {
        int link_idx;
        std::string name;
        int parent_joint_idx;
        int parent_link_idx;
        std::vector<int> child_joint_indices;
        std::vector<int> child_link_indices;
        std::vector<CollisionShape> collisions;
    };
}