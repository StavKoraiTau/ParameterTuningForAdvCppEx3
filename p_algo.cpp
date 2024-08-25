
#include "p_algo.h" 

#include "algo_common/AlgorithmRegistration.h"
REGISTER_ALGORITHM(algo_7);

double algo_7::getValidScore(const AlgoInfo& state,
                                         const AlgoTileInfo& tile_info) const{
    double score = 0;
    double batt_precent = static_cast<double>(state.getBattery()) / state.getMaxBattery();
    if (tile_info.getStatus() == TileStatus::ValidUnvisited) {
        score += tile_info.getDistance() * UNVISITED_DIST +
                 tile_info.getDockDistance() * UNVISITED_DOCK_DIST *
                     (1 / (batt_precent + UNVISITED_BATT_BIAS)) * UNVISITED_BATT_MULT;
    } else {
        score += (tile_info.getDistance() * DIRTY_DIST +
                  tile_info.getDockDistance() * DIRTY_DOCK_DIST *
                      (1 / (batt_precent + DIRTY_BATT_BIAS)) * DIRTY_BATT_MULT -
                  (tile_info.getDirtLevel()) * DIRT_MULT);
    }
    return score;
}
std::optional<double> algo_7::getCloseValidScore(const AlgoInfo& state,
                                                             const AlgoTileInfo& tile_info) const{
    double score = 0;
    double batt_precent = static_cast<double>(state.getBattery()) / state.getMaxBattery();
    if (tile_info.getStatus() == TileStatus::ValidUnvisited) {
        score += tile_info.getDistance() * C_UNVISITED_DIST +
                 tile_info.getDockDistance() * C_UNVISITED_DOCK_DIST *
                     (1 / (batt_precent + C_UNVISITED_BATT_BIAS)) * C_UNVISITED_BATT_MULT;
    } else {
        score += (tile_info.getDistance() * C_DIRTY_DIST +
                  tile_info.getDockDistance() * C_DIRTY_DOCK_DIST *
                      (1 / (batt_precent + C_DIRTY_BATT_BIAS)) * C_DIRTY_BATT_MULT -
                  (tile_info.getDirtLevel()) * C_DIRT_MULT);
    }
    return score;
}
