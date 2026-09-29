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

    struct CollisionGeometry
    {
        Isometry3 origin;
        std::variant<BoxGeometry, SphereGeometry, CylinderGeometry, MeshGeometry> geometry;
    };

    struct LinkDescriptor
    {
        int link_index;
        std::string name;
        Isometry3 origin;
        std::vector<CollisionGeometry> collisions;
    };
}