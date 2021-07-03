#include "Ego.h"
#include "WaveFrontTypes.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <utility>
#include <vector>

Ego::Ego() = default;

void Ego::MoveToCell(const GridCell& grid_cell){
    this->x_idx_.at(0) = grid_cell.x_idx_;
    this->z_idx_.at(0) = grid_cell.z_idx_;
    // return grid_cell.aabb_.getCenter();
}


uint16_t Ego::FindBestMove(const std::vector<GridCell>& grid_cells, const uint16_t& num_cells_x, const uint16_t& num_cells_z) {
    // Look at 4 directions from the current cell and find the smallest distance value
    uint16_t target_x;
    uint16_t target_z;
    uint16_t best_move_x;
    uint16_t best_move_z;
    uint16_t cell_target;
    int min_distance = std::numeric_limits<int>::max();
    // NORTH
    target_x = x_idx_.at(0);
    target_z = z_idx_.at(0) - 1;
    cell_target = target_z * num_cells_x + target_x;
    if( (z_idx_.at(0) != 0 ) && ( grid_cells.at(cell_target).distance_ != -1) 
                             && ( grid_cells.at(cell_target).distance_ < min_distance)  ) { // not on the edge
        min_distance = grid_cells.at(cell_target).distance_;
        best_move_x = target_x;
        best_move_z = target_z;
    } 
    // SOUTH
    target_x = x_idx_.at(0);
    target_z = z_idx_.at(0) + 1;
    cell_target = target_z * num_cells_x + target_x;
    if( (z_idx_.at(0) != (num_cells_z-1) ) && ( grid_cells.at(cell_target).distance_ != -1) 
                                           && ( grid_cells.at(cell_target).distance_ < min_distance)  ) { // not on the edge
        min_distance = grid_cells.at(cell_target).distance_;
        best_move_x = target_x;
        best_move_z = target_z;
    } 
    // WEST
    target_x = x_idx_.at(0) - 1;
    target_z = z_idx_.at(0);
    cell_target = target_z * num_cells_x + target_x;
    if( (x_idx_.at(0) != 0 ) && ( grid_cells.at(cell_target).distance_ != -1)  
                             && ( grid_cells.at(cell_target).distance_ < min_distance)  ) { // not on the edge
        min_distance = grid_cells.at(cell_target).distance_;
        best_move_x = target_x;
        best_move_z = target_z;
    } 
    // EAST
    target_x = x_idx_.at(0) + 1;
    target_z = z_idx_.at(0);
    cell_target = target_z * num_cells_x + target_x;
    if( (x_idx_.at(0) != (num_cells_x-1) ) && ( grid_cells.at(cell_target).distance_ != -1)  
                                           && ( grid_cells.at(cell_target).distance_ < min_distance)  ) { // not on the edge
        min_distance = grid_cells.at(cell_target).distance_;
        best_move_x = target_x;
        best_move_z = target_z;
    }

    // Update the 2d info with best move
    this->x_idx_.at(0) = best_move_x;
    this->z_idx_.at(0) = best_move_z;

    // return std::make_pair(best_move_x, best_move_z);
    return cell_target = best_move_z * num_cells_x + best_move_x;
}