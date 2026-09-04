#ifndef GRID_H
#define GRID_H

#include <vector>
#include <string>
#include <utility>
#include <functional>

// Simple (row, col) coordinate on the grid.
struct Point {
    int row;
    int col;

    bool operator==(const Point& other) const {
        return row == other.row && col == other.col;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

// Hash so Point can be used in unordered_map / unordered_set.
// Uses a portable hash-combine (works whether size_t is 32-bit or 64-bit,
// unlike a raw "<< 32" shift which overflows on 32-bit size_t).
struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        std::size_t h1 = std::hash<int>{}(p.row);
        std::size_t h2 = std::hash<int>{}(p.col);
        return h1 ^ (h2 + 0x9e3779b9u + (h1 << 6) + (h1 >> 2));
    }
};

// Represents a 2D grid where each cell is either walkable or an obstacle.
// Cells can also carry a movement cost (>= 1) for weighted-grid experiments.
class Grid {
public:
    Grid(int rows, int cols);

    // Load a grid from a text file. Format:
    //   '.' = walkable cell (cost 1)
    //   '#' = obstacle (not walkable)
    //   digit 2-9 = walkable cell with that movement cost
    // Returns true on success.
    static Grid fromFile(const std::string& path);

    int rows() const { return rows_; }
    int cols() const { return cols_; }

    bool inBounds(const Point& p) const;
    bool isWalkable(const Point& p) const;
    int cost(const Point& p) const;

    void setObstacle(const Point& p);
    void setCost(const Point& p, int cost);

    // Returns valid neighbours of a cell. If allowDiagonal is true,
    // 8-directional movement is used, otherwise 4-directional.
    std::vector<Point> neighbours(const Point& p, bool allowDiagonal) const;

    // Pretty-print the grid to stdout. Optionally overlay a path and
    // the visited/explored set for visualization.
    void print(const std::vector<Point>& path = {},
               const std::vector<Point>& visited = {},
               const Point* start = nullptr,
               const Point* goal = nullptr) const;

private:
    int rows_, cols_;
    std::vector<std::vector<int>> cells_; // -1 = obstacle, else movement cost
};

#endif // GRID_H
