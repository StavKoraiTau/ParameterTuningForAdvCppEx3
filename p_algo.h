
#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__
#include "algo_common/SmartAlgorithm.h"
class algo_7 : public SmartAlgorithm {
    static constexpr double UNVISITED_DIST = 0.5462097578163492;
    static constexpr double UNVISITED_DOCK_DIST = 0.44100805703890267;
    static constexpr double UNVISITED_BATT_BIAS = 0.6224245469066215;
    static constexpr double UNVISITED_BATT_MULT = -0.2138235509510409;
    static constexpr double DIRTY_DIST = 0.14152948588080067;
    static constexpr double DIRTY_DOCK_DIST = -1.3521790558779676;
    static constexpr double DIRTY_BATT_BIAS = 0.9043342025488852;
    static constexpr double DIRTY_BATT_MULT = -0.3606794654419676;
    static constexpr double DIRT_MULT = 1.0028269573857551;
    static constexpr double C_UNVISITED_DIST = 1.6970835275065956;
    static constexpr double C_UNVISITED_DOCK_DIST = -0.39456208898318657;
    static constexpr double C_UNVISITED_BATT_BIAS = 0.31986528516598134;
    static constexpr double C_UNVISITED_BATT_MULT = -0.3484275407239051;
    static constexpr double C_DIRTY_DIST = -1.0883987375406126;
    static constexpr double C_DIRTY_DOCK_DIST = 0.17950150794935907;
    static constexpr double C_DIRTY_BATT_BIAS = 2.159988682990796;
    static constexpr double C_DIRTY_BATT_MULT = -0.17673248801061728;
    static constexpr double C_DIRT_MULT = -1.0408871893183558;

protected:
    double getValidScore(const AlgoInfo& state, const AlgoTileInfo& tile_info) const override;
    std::optional<double> getCloseValidScore(const AlgoInfo& state,
                                             const AlgoTileInfo& tile_info) const override;
};
#endif  //__ALGORITHM_H__
