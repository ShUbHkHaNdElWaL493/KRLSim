#pragma once

#include "joint.hpp"
#include "link.hpp"

namespace krlsim
{
    struct KinematicsModel
    {
        std::string name;

        int n_dof = 0;
        int n_q = 0;   // Size of the position vector q
        int n_v = 0;   // Size of the velocity vector v

        int root_link_idx = -1;

        std::vector<LinkDescriptor> links;
        std::vector<JointDescriptor> joints;
        std::vector<int> active_joint_indices; 
    };
}