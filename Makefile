CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude
SRC := src/main.cpp src/Grid.cpp src/AStarGrid.cpp
TARGET := heuristiq

.PHONY: all clean run test

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

run: $(TARGET)
	./$(TARGET) maps/sample_map.txt

# Builds and runs both test executables. Each prints its own PASS/FAIL
# summary; `make test` fails (non-zero exit) if either suite fails.
test:
	$(CXX) $(CXXFLAGS) -Itests -o test_astar_grid tests/test_astar_grid.cpp src/Grid.cpp src/AStarGrid.cpp
	$(CXX) $(CXXFLAGS) -Itests -o test_graph_astar tests/test_graph_astar.cpp
	./test_astar_grid
	./test_graph_astar

clean:
	rm -f $(TARGET) test_astar_grid test_graph_astar test_astar_grid.exe test_graph_astar.exe $(TARGET).exe
