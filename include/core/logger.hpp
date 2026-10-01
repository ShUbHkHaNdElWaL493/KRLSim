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
            LogType log_priority;
            std::string logTypeToString(const LogType& log_type) const;
            bool checkPriority(const LogType& log_type) const;

        public:
            Logger(const LogType& log_priority = LogType::LOG);
            LogType getPriority() const;
            void log(const LogType& log_type, const std::string& message) const;
    };
}