#pragma once

#include "joint.hpp"
#include "link.hpp"

namespace krlsim
{
    class KinematicsModel
    {

        private:
            std::string name;
            std::vector<JointDescriptor> joints;
            std::vector<LinkDescriptor> links;

        public:
            KinematicsModel(std::string name);
            void parseJSON();
            void parseURDF();

    };
}