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
            std::pair<bool, std::string> parseRobotDescription(const std::string& robot_description);
            std::pair<bool, std::string> parseURDF(const std::string& urdf_file_path);

    };
}