cmake_minimum_required(VERSION 3.20)
project(crowd_simulation_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(crowd_simulation
    src/graph.cpp
    src/simulation.cpp
)
target_include_directories(crowd_simulation PUBLIC include)
target_compile_options(crowd_simulation PRIVATE -Wall -Wextra -Wpedantic)

add_executable(crowd-sim src/main.cpp)
target_link_libraries(crowd-sim PRIVATE crowd_simulation)

enable_testing()
add_executable(crowd-tests tests/test_main.cpp)
target_link_libraries(crowd-tests PRIVATE crowd_simulation)
add_test(NAME crowd-tests COMMAND crowd-tests)
