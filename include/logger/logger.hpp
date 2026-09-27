#pragma once

#include <string>

namespace krlsim
{

    enum class LogType
    {
        ERROR,
        WARNING,
        LOG,
        DEBUG
    };

    class Logger
    {

        private:
            size_t log_priority;
            bool checkPriority(LogType log_type);
            std::string logTypeToString(LogType log_type);

        public:
            Logger(LogType log_type = LogType::LOG);
            void log(LogType log_type, std::string message);

    };

}