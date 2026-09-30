#include <iostream>

#include "model/model.hpp"

using namespace krlsim;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        Logger logger;
        logger.log(LogType::ERROR, "Usage: ./check_urdf ${URDF_FILE_PATH}");
        return 1;
    } else
    {
        Model model("robot", std::make_shared<Logger>());
        model.parseURDF(argv[1]);
        std::string model_json = model.toJSON();
        return 0;
    }

}