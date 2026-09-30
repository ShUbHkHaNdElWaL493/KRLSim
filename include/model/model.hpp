#pragma once

#include <memory>

#include "core/logger.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace krlsim
{
    class Model
    {

        private:
            std::shared_ptr<Logger> logger;
            std::string name;
            std::vector<JointDescriptor> joints;
            std::vector<LinkDescriptor> links;

        public:
            Model(const std::string& name = "robot", std::shared_ptr<Logger> logger = nullptr);
            void parseRobotDescription(const std::string& robot_description);
            void parseURDF(const std::string& urdf_file_path);
            std::string toJSON();

    };
}