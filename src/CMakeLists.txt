# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/$ENV{CURR_ENV}/*.cpp)

idf_component_register(SRCS ${app_sources})
