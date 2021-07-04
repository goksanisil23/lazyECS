#pragma once

#include "ECSCore/Types.hpp"
#include "Components/Tag.h"
#include "WaveFrontTypes.h"
#include "GridCell.hpp"

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

class Ego {

public:
    explicit Ego();

    std::vector<uint16_t> x_idx_; // position of the ego in the 2D grid
    std::vector<uint16_t> z_idx_;
    bool pathCalculated_; // if one of the goals have been reached
    std::deque<uint16_t> path_; // indices of the grid cells that make up the shortest path to goal

    void MoveToCell(const GridCell& grid_cell);

    void FindShortestPath(const std::vector<GridCell>& grid_cells, const uint16_t& num_cells_x, const uint16_t& num_cells_z);

private:

};