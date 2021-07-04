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

// Finds the best grid cell to move by following down the gradient.
// Returns serialized coordinates of the cell that is the best decision
void Ego::FindShortestPath(const std::vector<GridCell>& grid_cells, const uint16_t& num_cells_x, const uint16_t& num_cells_z) {

    pathCalculated_ = false;

    // Look at 4 directions from the current cell and find the smallest distance value
    uint16_t target_x;
    uint16_t target_z;
    uint16_t best_move_x;
    uint16_t best_move_z;
    uint16_t target_cell;
    int min_distance = std::numeric_limits<int>::max();

    while(!pathCalculated_) {
        // NORTH
        target_x = x_idx_.at(0);
        target_z = z_idx_.at(0) - 1;
        target_cell = target_z * num_cells_x + target_x;
        if( (z_idx_.at(0) != 0 ) && ( grid_cells.at(target_cell).distance_ != -1) 
                                && ( grid_cells.at(target_cell).distance_ < min_distance)  ) { // not on the edge
            min_distance = grid_cells.at(target_cell).distance_;
            best_move_x = target_x;
            best_move_z = target_z;
        } 
        // SOUTH
        target_x = x_idx_.at(0);
        target_z = z_idx_.at(0) + 1;
        target_cell = target_z * num_cells_x + target_x;
        if( (z_idx_.at(0) != (num_cells_z-1) ) && ( grid_cells.at(target_cell).distance_ != -1) 
                                            && ( grid_cells.at(target_cell).distance_ < min_distance)  ) { // not on the edge
            min_distance = grid_cells.at(target_cell).distance_;
            best_move_x = target_x;
            best_move_z = target_z;
        } 
        // WEST
        target_x = x_idx_.at(0) - 1;
        target_z = z_idx_.at(0);
        target_cell = target_z * num_cells_x + target_x;
        if( (x_idx_.at(0) != 0 ) && ( grid_cells.at(target_cell).distance_ != -1)  
                                && ( grid_cells.at(target_cell).distance_ < min_distance)  ) { // not on the edge
            min_distance = grid_cells.at(target_cell).distance_;
            best_move_x = target_x;
            best_move_z = target_z;
        } 
        // EAST
        target_x = x_idx_.at(0) + 1;
        target_z = z_idx_.at(0);
        target_cell = target_z * num_cells_x + target_x;
        if( (x_idx_.at(0) != (num_cells_x-1) ) && ( grid_cells.at(target_cell).distance_ != -1)  
                                            && ( grid_cells.at(target_cell).distance_ < min_distance)  ) { // not on the edge
            min_distance = grid_cells.at(target_cell).distance_;
            best_move_x = target_x;
            best_move_z = target_z;
        }

        // Update the 2D coordinates of ego with best move
        this->x_idx_.at(0) = best_move_x;
        this->z_idx_.at(0) = best_move_z;
        path_.push_back(best_move_z * num_cells_x + best_move_x); // update the path

        // check if destination has been reached
        if(grid_cells.at(path_.back()).distance_ == 1) {
            pathCalculated_ = true;
        }

    }

    std::cout << " **** Path has been calculated! ****" << std::endl;

}