#include <iostream>

#include "model/model.hpp"

using namespace krlsim;

int main(int argc, char **argv)
{
    std::shared_ptr<Logger> logger = std::make_shared<Logger>(LogType::LOG);
    if (argc != 2)
    {
        logger->log(LogType::ERROR, "Usage: ./check_urdf ${URDF_FILE_PATH}");
        return 1;
    } else
    {
        Model model("robot", logger);
        model.parseURDF(argv[1]);
        model.visualize();
        return 0;
    }

}