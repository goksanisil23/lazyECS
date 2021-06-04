cmake_minimum_required(VERSION 3.12)

project(minimal_3d)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(minimal_3d main.cpp)

target_link_libraries(minimal_3d
                        lazyECS)

target_include_directories(minimal_3d PUBLIC ${LAZY_ECS_PARENT_DIR})