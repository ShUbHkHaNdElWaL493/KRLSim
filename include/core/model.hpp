#pragma once

#include "joint.hpp"
#include "link.hpp"

namespace krlsim
{
    class Model
    {

        private:
            std::string name;
            std::vector<JointDescriptor> joints;
            std::vector<LinkDescriptor> links;

        public:
            Model(std::string name);
            void parseJSON();
            void parseURDF();

    };
}