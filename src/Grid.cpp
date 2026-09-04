#include "Grid.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cctype>

Grid::Grid(int rows, int cols)
    : rows_(rows), cols_(cols),
      cells_(rows, std::vector<int>(cols, 1)) {}

Grid Grid::fromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open map file: " + path);
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back(); // handle CRLF
        if (line.empty()) continue;
        lines.push_back(line);
    }
    if (lines.empty()) {
        throw std::runtime_error("Map file is empty: " + path);
    }

    int rows = static_cast<int>(lines.size());
    int cols = static_cast<int>(lines[0].size());
    for (const auto& l : lines) {
        cols = std::max(cols, static_cast<int>(l.size()));
    }

    Grid grid(rows, cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < static_cast<int>(lines[r].size()); ++c) {
            char ch = lines[r][c];
            Point p{r, c};
            if (ch == '#') {
                grid.setObstacle(p);
            } else if (std::isdigit(static_cast<unsigned char>(ch)) && ch != '0' && ch != '1') {
                grid.setCost(p, ch - '0');
            }
            // '.' or anything else -> default walkable, cost 1
        }
    }
    return grid;
}

bool Grid::inBounds(const Point& p) const {
    return p.row >= 0 && p.row < rows_ && p.col >= 0 && p.col < cols_;
}

bool Grid::isWalkable(const Point& p) const {
    if (!inBounds(p)) return false;
    return cells_[p.row][p.col] != -1;
}

int Grid::cost(const Point& p) const {
    if (!inBounds(p)) return -1;
    return cells_[p.row][p.col];
}

void Grid::setObstacle(const Point& p) {
    if (inBounds(p)) cells_[p.row][p.col] = -1;
}

void Grid::setCost(const Point& p, int cost) {
    if (inBounds(p) && cost > 0) cells_[p.row][p.col] = cost;
}

std::vector<Point> Grid::neighbours(const Point& p, bool allowDiagonal) const {
    static const std::vector<Point> fourDir = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    static const std::vector<Point> eightDir = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    const auto& dirs = allowDiagonal ? eightDir : fourDir;
    std::vector<Point> result;
    result.reserve(dirs.size());

    for (const auto& d : dirs) {
        Point np{p.row + d.row, p.col + d.col};
        if (!isWalkable(np)) continue;

        // Prevent cutting through diagonal walls (corner clipping).
        if (allowDiagonal && d.row != 0 && d.col != 0) {
            Point side1{p.row + d.row, p.col};
            Point side2{p.row, p.col + d.col};
            if (!isWalkable(side1) || !isWalkable(side2)) continue;
        }
        result.push_back(np);
    }
    return result;
}

void Grid::print(const std::vector<Point>& path,
                  const std::vector<Point>& visited,
                  const Point* start,
                  const Point* goal) const {
    std::vector<std::vector<char>> canvas(rows_, std::vector<char>(cols_));
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            canvas[r][c] = (cells_[r][c] == -1) ? '#' : '.';
        }
    }

    for (const auto& v : visited) {
        if (inBounds(v)) canvas[v.row][v.col] = ':';
    }
    for (const auto& p : path) {
        if (inBounds(p)) canvas[p.row][p.col] = '*';
    }
    if (start) canvas[start->row][start->col] = 'S';
    if (goal) canvas[goal->row][goal->col] = 'G';

    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            std::cout << canvas[r][c] << ' ';
        }
        std::cout << '\n';
    }
}
