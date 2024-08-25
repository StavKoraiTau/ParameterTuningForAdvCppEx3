
#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__
#include "algo_common/SmartAlgorithm.h"
class BestAlgo : public SmartAlgorithm {
    static constexpr double UNVISITED_DIST = 1.1370962599778203;
    static constexpr double UNVISITED_DOCK_DIST = 1.2418428945341162;
    static constexpr double UNVISITED_BATT_BIAS = 0.28918873002175505;
    static constexpr double UNVISITED_BATT_MULT = -0.3179860138418087;
    static constexpr double DIRTY_DIST = 0.4932758474461607;
    static constexpr double DIRTY_DOCK_DIST = 0.29256496738200827;
    static constexpr double DIRTY_BATT_BIAS = -0.20873227537103978;
    static constexpr double DIRTY_BATT_MULT = 0.0032537666993173175;
    static constexpr double DIRT_MULT = 0;
    static constexpr double C_UNVISITED_DIST = -2.465438996633364;
    static constexpr double C_UNVISITED_DOCK_DIST = -2.013422242963672;
    static constexpr double C_UNVISITED_BATT_BIAS = 0.03749164691633776;
    static constexpr double C_UNVISITED_BATT_MULT = -0.16933004184816008;
    static constexpr double C_DIRTY_DIST = 1.321767929051818;
    static constexpr double C_DIRTY_DOCK_DIST = -0.7920311370771206;
    static constexpr double C_DIRTY_BATT_BIAS = 0.9602668163024034;
    static constexpr double C_DIRTY_BATT_MULT = 1.316353054728793;
    static constexpr double C_DIRT_MULT = 0.11830377797144165;

protected:
    double getValidScore(const AlgoInfo& state, const AlgoTileInfo& tile_info) const override;
    std::optional<double> getCloseValidScore(const AlgoInfo& state,
                                             const AlgoTileInfo& tile_info) const override;
};
#endif  //__ALGORITHM_H__
