set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(check_urdf check_urdf.cpp)
target_link_libraries(
    check_urdf
    PUBLIC
    krlsim_logger
    krlsim_model
)