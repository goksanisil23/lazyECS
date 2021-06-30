cmake_minimum_required(VERSION 3.12)

project(wavefront)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(wavefront main.cpp WaveFront.cpp GoalAndObstacle.hpp Ego.cpp)

target_include_directories(wavefront PUBLIC
                                            ${LAZY_ECS_PARENT_DIR}
                                            ${LAZY_ECS_INCLUDE_DIRS}
                                            ${CMAKE_CURRENT_SOURCE_DIR})

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(wavefront PUBLIC ${p})
endforeach()

target_compile_definitions(wavefront PUBLIC ${NANOGUI_EXTRA_DEFS})

target_link_libraries(wavefront lazyECS)