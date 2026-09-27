cmake_minimum_required(VERSION 3.24)
project(krlsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(coal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(EnTT REQUIRED)
find_package(nanobind CONFIG REQUIRED)
find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(urdfdom REQUIRED)

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/include)

# add_library(krlsim_core SHARED
#     src/core/ecs.cpp
#     src/core/forward_kin.cpp
#     src/parser/urdf_loader.cpp
#     src/collision/collision_engine.cpp
#     src/main.cpp
# )

# target_include_directories(krlsim_core PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# target_link_libraries(krlsim_core PUBLIC
#     Eigen3::Eigen
#     urdfdom::urdfdom_model
#     urdfdom::urdfdom_world
#     urdfdom::urdfdom_sensor
#     coal::coal
#     EnTT::EnTT
# )

# # --- 2. Python Bindings Layer ---
# nanobind_add_module(kinsim_ext python/bindings/bindings.cpp)
# target_link_libraries(kinsim_ext PRIVATE krlsim_core)

# # Installation definitions used by scikit-build-core
# install(TARGETS krlsim_core kinsim_ext DESTINATION kinsim)