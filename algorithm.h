#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__
#include "SmartAlgorithm.h"
class ParameterAlgorithm : public SmartAlgorithm {
    static constexpr double UNVISITED_DIST = 0;
    static constexpr double UNVISITED_DOCK_DIST = 0;
    static constexpr double UNVISITED_BATT_BIAS = 0.2;
    static constexpr double UNVISITED_BATT_MULT = 0.2;
    static constexpr double DIRTY_DIST = 0;
    static constexpr double DIRTY_DOCK_DIST = 0;
    static constexpr double DIRTY_BATT_BIAS = 0.2;
    static constexpr double DIRTY_BATT_MULT = 0.2;
    static constexpr double DIRT_MULT = 0;
    static constexpr double C_UNVISITED_DIST = 0;
    static constexpr double C_UNVISITED_DOCK_DIST = 0;
    static constexpr double C_UNVISITED_BATT_BIAS = 0.2;
    static constexpr double C_UNVISITED_BATT_MULT = 0.2;
    static constexpr double C_DIRTY_DIST = 0;
    static constexpr double C_DIRTY_DOCK_DIST = 0;
    static constexpr double C_DIRTY_BATT_BIAS = 0.2;
    static constexpr double C_DIRTY_BATT_MULT = 0.2;
    static constexpr double C_DIRT_MULT = 0;

protected:
    double getValidScore(const AlgoInfo& state, const AlgoTileInfo& tile_info) const override;
    std::optional<double> getCloseValidScore(const AlgoInfo& state,
                                             const AlgoTileInfo& tile_info) const override;
};
#endif  //__ALGORITHM_H__