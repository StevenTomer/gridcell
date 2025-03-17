cmake_minimum_required(VERSION 3.30)
project(gridcell_neighborhood)

file(GLOB_RECURSE sources src/*.cpp)

add_executable(gridcell ${sources})

target_compile_options(gridcell PUBLIC -std=c++20 -Wall)
target_include_directories(gridcell PUBLIC src)

find_package(Boost COMPONENTS program_options REQUIRED)

target_link_libraries(gridcell PUBLIC
  ${Boost_LIBRARIES}
)
