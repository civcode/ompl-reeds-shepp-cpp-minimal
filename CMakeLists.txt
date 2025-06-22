cmake_minimum_required(VERSION 3.10)
project(OMPL_ReedsShepp_Demo)

find_package(ompl REQUIRED)
find_Package(RenderModule REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(distance src/main_distance.cpp)
target_link_libraries(distance PUBLIC ompl::ompl RenderModule::RenderModule)

add_executable(interpolate src/main_interpolate.cpp)
target_link_libraries(interpolate PUBLIC ompl::ompl RenderModule::RenderModule)

add_executable(direction src/main_direction.cpp)
target_link_libraries(direction PUBLIC ompl::ompl RenderModule::RenderModule)

