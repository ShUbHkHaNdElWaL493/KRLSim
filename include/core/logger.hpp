#pragma once

#include <string>

#include "types.hpp"

namespace krlsim
{
    class Logger
    {
        private:
            size_t log_priority;
            bool checkPriority(const LogType& log_type);
            std::string logTypeToString(const LogType& log_type);

        public:
            Logger(const LogType& log_type = LogType::LOG);
            void log(const LogType& log_type, const std::string& message);
    };
}