#pragma once

#include "joint.hpp"
#include "link.hpp"

namespace krlsim
{
    class Model
    {

        private:
            std::string name;
            size_t n_q, n_v;
            std::vector<JointDescriptor> joints;
            std::vector<LinkDescriptor> links;

        public:
            Model(std::string name);
            ReturnType parseRobotDescription(const std::string& robot_description);
            ReturnType parseURDF(const std::string& urdf_file_path);

    };
}