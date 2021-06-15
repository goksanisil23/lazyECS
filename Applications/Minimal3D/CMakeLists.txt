cmake_minimum_required(VERSION 3.12)

project(minimal_3d)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(minimal_3d main.cpp Minimal3D.cpp)

target_include_directories(minimal_3d PUBLIC
                                            ${LAZY_ECS_PARENT_DIR}
                                            ${LAZY_ECS_INCLUDE_DIRS}
                                            ${CMAKE_CURRENT_SOURCE_DIR})

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(minimal_3d PUBLIC ${p})
endforeach()

target_compile_definitions(minimal_3d PUBLIC ${NANOGUI_EXTRA_DEFS})

target_link_libraries(minimal_3d lazyECS)