cmake_minimum_required(VERSION 3.12)

project(potential_field)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(potential_field main.cpp PotentialField.cpp GoalAndObstacle.hpp Ego.cpp)

target_include_directories(potential_field PUBLIC
                                            ${LAZY_ECS_PARENT_DIR}
                                            ${LAZY_ECS_INCLUDE_DIRS}
                                            ${CMAKE_CURRENT_SOURCE_DIR})

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(potential_field PUBLIC ${p})
endforeach()

target_compile_definitions(potential_field PUBLIC ${NANOGUI_EXTRA_DEFS})

target_link_libraries(potential_field lazyECS)