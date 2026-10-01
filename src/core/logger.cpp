#include <iostream>

#include "core/logger.hpp"

namespace krlsim
{

    bool Logger::checkPriority(const LogType& log_type) const
    {
        switch (log_type)
        {
            case LogType::ERROR:
                return (this->log_priority == LogType::ERROR);
            case LogType::WARNING:
                return (
                    (this->log_priority == LogType::ERROR) ||
                    (this->log_priority == LogType::WARNING)
                );
            case LogType::LOG:
                return (
                    (this->log_priority == LogType::ERROR) ||
                    (this->log_priority == LogType::WARNING) ||
                    (this->log_priority == LogType::LOG)
                );
            case LogType::DEBUG:
                return (
                    (this->log_priority == LogType::ERROR) ||
                    (this->log_priority == LogType::WARNING) ||
                    (this->log_priority == LogType::LOG) ||
                    (this->log_priority == LogType::DEBUG)
                );
            default:
                return false;
        }
    }

    std::string Logger::logTypeToString(const LogType& log_type) const
    {
        switch (log_type)
        {
            case LogType::ERROR:
                return "ERROR";
                break;
            case LogType::WARNING:
                return "WARNING";
                break;
            case LogType::LOG:
                return "LOG";
                break;
            case LogType::DEBUG:
                return "DEBUG";
                break;
            default:
                return "UNKNOWN";
        }
    }

    Logger::Logger(const LogType& log_priority) : log_priority(log_priority) {}

    LogType Logger::getPriority() const { return log_priority; }

    void Logger::log(const LogType& log_type, const std::string& message) const
    {
        if (this->checkPriority(log_type))
        {
            std::cout << "[" << this->logTypeToString(log_type) << "] " << message << std::endl;
        }
    }

}