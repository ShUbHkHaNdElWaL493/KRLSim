cmake_minimum_required(VERSION 3.24)
project(krlsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(coal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

add_executable(
    ${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/logger/logger.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)