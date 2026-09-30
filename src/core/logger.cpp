#include <iostream>

#include "core/logger.hpp"

namespace krlsim
{

    bool Logger::checkPriority(const LogType& log_type)
    {
        switch (log_type)
        {
            case LogType::ERROR:
                return this->log_priority >= 1;
                break;
            case LogType::WARNING:
                return this->log_priority >= 2;
                break;
            case LogType::LOG:
                return this->log_priority >= 3;
                break;
            case LogType::DEBUG:
                return this->log_priority >= 4;
                break;
            default:
                return this->log_priority >= 0;
        }
    }

    std::string Logger::logTypeToString(const LogType& log_type)
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

    Logger::Logger(const LogType& log_type)
    {
        switch (log_type)
        {
            case LogType::ERROR:
                this->log_priority = 1;
                break;
            case LogType::WARNING:
                this->log_priority = 2;
                break;
            case LogType::LOG:
                this->log_priority = 3;
                break;
            case LogType::DEBUG:
                this->log_priority = 4;
                break;
            default:
                this->log_priority = 0;
        }
    }

    void Logger::log(const LogType& log_type, const std::string& message)
    {
        if (this->checkPriority(log_type))
        {
            std::cout << "[" << this->logTypeToString(log_type) << "] " << message << std::endl;
        }
    }

}