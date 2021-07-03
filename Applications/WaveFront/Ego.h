#pragma once

#include "ECSCore/Types.hpp"
#include "Components/Tag.h"
#include "WaveFrontTypes.h"
#include "GridCell.hpp"

#include <cstdint>
#include <utility>
#include <vector>
#include <unordered_map>

class Ego {

public:
    explicit Ego();

    std::vector<uint16_t> x_idx_; // position of the ego in the 2D grid
    std::vector<uint16_t> z_idx_;
    bool goalReached_; // if one of the goals have been reached

    void MoveToCell(const GridCell& grid_cell);

    uint16_t FindBestMove(const std::vector<GridCell>& grid_cells, const uint16_t& num_cells_x, const uint16_t& num_cells_z);

private:

};