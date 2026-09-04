cmake_minimum_required(VERSION 3.10)
project(HeuristIQ CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

include_directories(include)

add_executable(heuristiq
    src/main.cpp
    src/Grid.cpp
    src/AStarGrid.cpp
)

# --- Tests -------------------------------------------------------
# Each test file is its own tiny executable (they each define main()).
# Run all of them with: ctest   (from inside the build directory)
enable_testing()

add_executable(test_astar_grid
    tests/test_astar_grid.cpp
    src/Grid.cpp
    src/AStarGrid.cpp
)
target_include_directories(test_astar_grid PRIVATE tests)
add_test(NAME AStarGridTests COMMAND test_astar_grid)

add_executable(test_graph_astar
    tests/test_graph_astar.cpp
)
target_include_directories(test_graph_astar PRIVATE tests)
add_test(NAME GraphAStarTests COMMAND test_graph_astar)
