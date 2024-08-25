#ifndef __SMART_ALGORITHM_H__
#define __SMART_ALGORITHM_H__
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "algo_common/TileManager.h"
#include "common/AbstractAlgorithm.h"
#include "common/BatteryMeter.h"
#include "common/DirtSensor.h"
#include "common/WallSensor.h"
#include "common/enums.h"

class SmartAlgorithm : public AbstractAlgorithm {
protected:
    //====== Wrapper classes for passing info to scoring functions =======
    class AlgoInfo {
        const SmartAlgorithm* m_algo;

    public:
        AlgoInfo(const SmartAlgorithm* algo) : m_algo(algo) {}
        size_t getBattery() const;
        size_t getMaxBattery() const;
        size_t getStepsLeft() const;
        size_t numberDoneTiles() const;
        size_t numberUnvisitedTiles() const;
        size_t numberCleanableTiles() const;
    };
    class AlgoTileInfo {
        const std::pair<int, int> m_coord;
        const TileInfo& m_tile_info;
        const TileManager& m_tile_manager;
        bool m_close_tile;
        size_t m_distance;

    public:
        AlgoTileInfo(const std::pair<int, int>& coord, const TileInfo& tile_info,
                     const TileManager& manager, size_t distance)
            : m_coord(coord),
              m_tile_info(tile_info),
              m_tile_manager(manager),
              m_distance(distance) {}
        size_t getDockDistance() const;
        size_t getDistance() const;
        size_t getDirtLevel() const;
        TileStatus getStatus() const;
        size_t numUnvisitedNeighbours() const;
        size_t numDirtyNeighbours() const;
    };
    virtual double getValidScore(const AlgoInfo& state, const AlgoTileInfo& tile_info) const = 0;
    virtual std::optional<double> getCloseValidScore(const AlgoInfo& state,
                                                     const AlgoTileInfo& tile_info) const = 0;
    //===================================================================
private:
    enum class AlgorithmState { OnPath, Charging, Cleaning, Idle };
    const std::pair<int, int> DockingCoords{0, 0};

    const DirtSensor* m_dirt_sensor;
    const WallsSensor* m_wall_sensor;
    const BatteryMeter* m_battery_meter;

    size_t m_max_steps;
    size_t m_curr_steps;
    size_t m_max_charge;

    std::vector<std::pair<int, int>> m_curr_path;
    std::pair<int, int> m_curr_coord;
    int& m_curr_x;
    int& m_curr_y;
    AlgorithmState m_algo_state;
    TileManager m_tile_manager;
    /*
    Update the current tile and surounding ones based on sensor data.
    */
    void updateSurroundings();

    const TileInfo& getNeighbourTile(Direction dir) const;
    std::pair<int, int> getNeighbourCoord(Direction dir) const;
    /*
    Convert a neighbour coordinate (to curr_coord) and convert it to the direction it is at.
    */
    Direction coordinateToDirection(const std::pair<int, int>& coord) const;
    /*
    Check if its possible to clean more tiles. (based on max steps remaining and reachability from
    dock with max_charge)
    */
    bool isDone();
    /*
    Get the max cleanable distance. (based on max steps remaining and reachability from dock from
    dock with max_charge)
    */
    size_t cleanableDistance() const;
    /*
    Check whether a tile is cleanable based on cleanableDistance()
    */
    bool isCleanable(const TileInfo& tile) const;
    /*
    Using to chosen step update our coordinate
    */
    void updateCoordinates(Step step);
    /*
    Find a tile which is not done  and we can travel to and get from it to docking station within
    max steps and battery constraints, while cleaning or exploring.
    */
    std::optional<std::pair<int, int>> findViableTrip() const;
    /*
    Find a neighbour tile which is not done  and we can travel to and get from it to docking station
    within max steps and battery constraints, while cleaning or exploring.
    */
    std::optional<std::pair<int, int>> findNeighbourTrip() const;
    /*
    Check wether it is possible to stay on the current tile based on battery and max steps
    constraints
    */
    bool canStay() const;
    void getNextPath();
    /*
    Get and remove the next step on the current path
    */
    Step popNextStep();
    bool atCoordinate(const std::pair<int, int>& coord) const;
    static Step directionToStep(Direction dir);

    AlgoInfo m_algo_state_info;
    const AlgoInfo& getInfo() const { return m_algo_state_info; }
    std::optional<Step> tryContinueState();
    size_t getMaxTripLength() const;

public:
    /*
    Set algorithm parameters
    */
    void setMaxSteps(std::size_t max_steps) override;
    void setWallsSensor(const WallsSensor& wall_sense) override;
    void setDirtSensor(const DirtSensor& dirt_sense) override;
    void setBatteryMeter(const BatteryMeter& batt_meter) override;
    /*
    Get the next step of the algorithm, the algorithm assumes the returned step was taken.
    */
    Step nextStep() override;
    inline bool atDockingStation() const { return atCoordinate(DockingCoords); }

    SmartAlgorithm()
        : m_dirt_sensor(nullptr),
          m_wall_sensor(nullptr),
          m_battery_meter(nullptr),
          m_max_steps(0),
          m_curr_steps(0),
          m_max_charge(0),
          m_curr_coord(0, 0),
          m_curr_x(m_curr_coord.first),
          m_curr_y(m_curr_coord.second),
          m_algo_state(AlgorithmState::Idle),
          m_tile_manager(DockingCoords),
          m_algo_state_info(this) {}
};
#endif