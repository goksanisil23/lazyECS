cmake_minimum_required(VERSION 3.12)

project(raycast)

# add_compile_options(-Wall -Wextra -Werror) #-pedantic -> causes error during c code compilation

add_executable(raycast main.cpp 
                       Raycast.cpp 
                       ${CMAKE_CURRENT_SOURCE_DIR}/../BaseApp/BaseApp.cpp)

target_include_directories(raycast PUBLIC
                                            ${LAZY_ECS_PARENT_DIR}
                                            ${LAZY_ECS_INCLUDE_DIRS}
                                            ${CMAKE_CURRENT_SOURCE_DIR}
                                            ${CMAKE_CURRENT_SOURCE_DIR}/../BaseApp)

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(raycast PUBLIC ${p})
endforeach()

target_compile_definitions(raycast PUBLIC ${NANOGUI_EXTRA_DEFS})

target_link_libraries(raycast lazyECS)