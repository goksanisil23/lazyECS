cmake_minimum_required(VERSION 3.12)

project(potential_field)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(potential_field main.cpp)

target_link_libraries(potential_field
                        lazyECS)

target_include_directories(lazyECS PUBLIC ../..)