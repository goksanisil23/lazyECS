cmake_minimum_required(VERSION 3.12)

project(lazyECS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(LAZY_ECS_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # we use parent dir since we include lazyECS headers as "lazyECS/..." 

###################### NANOGUI ##############################
set(NANOGUI_BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_BACKEND       OpenGL CACHE BOOL " " FORCE)
# Ask Nanogui not to build shared libraries
SET(NANOGUI_BUILD_SHARED OFF CACHE BOOL "Build Nanogui with static libraries" FORCE)
add_subdirectory(lazyECS/External/nanogui)

###################### LAZYECS ##############################

set(LAZY_ECS_INCLUDE_DIRS
        ${LAZY_ECS_PARENT_DIR}/lazyECS
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Systems
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Systems/Rendering
        ${LAZY_ECS_PARENT_DIR}/lazyECS/ECSCore
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Components
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Components/utils
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Utilities
        ${LAZY_ECS_PARENT_DIR}/lazyECS/Systems/Rendering/opengl-framework/src
        ${LAZY_ECS_PARENT_DIR}/lazyECS/External/nanogui/include
        ${LAZY_ECS_PARENT_DIR}/lazyECS/External/json
        ${ReactPhysics3D_INCLUDE_DIR}    
    )
#################

add_subdirectory(lazyECS)

add_subdirectory(Applications/Minimal3D)
add_subdirectory(Applications/PotentialField)
add_subdirectory(Applications/WaveFront)
add_subdirectory(Applications/Raycast)
