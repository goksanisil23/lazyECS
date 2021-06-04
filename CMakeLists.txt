cmake_minimum_required(VERSION 3.12)

project(lazyECS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LAZY_ECS_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # we use parent dir since we include lazyECS headers as "lazyECS/..." 

add_subdirectory(lazyECS)
# add_subdirectory(Applications/PotentialField)
add_subdirectory(Applications/Minimal3D)