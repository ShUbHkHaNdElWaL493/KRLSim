#include <iostream>

#include "model/model.hpp"
#include "logger/logger.hpp"

using namespace krlsim;

int main(int argc, char **argv)
{
    Logger logger;
    if (argc != 2)
    {
        logger.log(LogType::ERROR, "Usage: ./check_urdf ${URDF_FILE_PATH}");
        return 1;
    } else
    {
        Model model("robot");
        ReturnType result = model.parseURDF(argv[1]);
        if (result.first)
        {
            logger.log(LogType::LOG, result.second);
            return 0;
        } else
        {
            logger.log(LogType::ERROR, result.second);
            return 1;
        }
    }

}