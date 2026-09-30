cmake_minimum_required(VERSION 3.24)
project(krlsim LANGUAGES CXX)

option(BUILD_TESTING "Build the tests for KRLSim" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(coal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

add_library(
    krlsim_core
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_link_libraries(
    krlsim_core
    PUBLIC
    Eigen3::Eigen
)
target_include_directories(
    krlsim_core
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

add_library(
    krlsim_model
    ${CMAKE_SOURCE_DIR}/src/model/model.cpp
)
target_link_libraries(
    krlsim_model
    PUBLIC
    krlsim_core
    ${urdfdom_LIBRARIES}
)
target_include_directories(
    krlsim_model
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${urdfdom_INCLUDE_DIRS}
)

# add_executable(
#     ${PROJECT_NAME}
#     ${CMAKE_SOURCE_DIR}/src/main.cpp
# )
# target_link_libraries(
#     ${PROJECT_NAME}
#     PUBLIC
#     krlsim_logger
#     krlsim_model
# )

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()