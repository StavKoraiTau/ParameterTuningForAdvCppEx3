#include "algo_common/SmartAlgorithm.h"

#include <iostream>
#include <utility>

Step SmartAlgorithm::nextStep() {
    if (getInfo().getStepsLeft() == 0 && atCoordinate(DockingCoords)) {
        return Step::Finish;
    }
    updateSurroundings();
    if (m_algo_state == AlgorithmState::Idle) {  // try to change state from idle
        if (atCoordinate(DockingCoords)) {
            if (isDone()) {
                return Step::Finish;
            }
            m_algo_state = AlgorithmState::Charging;
        }
        if (m_dirt_sensor->dirtLevel() > 0) {
            m_algo_state = AlgorithmState::Cleaning;
        }
    }
    std::optional<Step> next_step = tryContinueState();
    if (next_step.has_value()) {
        return next_step.value();
    }
    // Idle : not cleaning charging or on a path
    getNextPath();
    if (m_curr_path.empty()) {
        m_algo_state = AlgorithmState::Idle;
        return Step::Finish;  // no viable tile to go to
    }
    next_step = popNextStep();
    updateCoordinates(next_step.value());
    m_curr_steps++;
    return next_step.value();
}

std::optional<Step> SmartAlgorithm::tryContinueState() {
    const AlgoInfo& info = getInfo();
    switch (m_algo_state) {
    case AlgorithmState::Charging:
        if (info.getBattery() >= info.getStepsLeft()) {
            m_algo_state = AlgorithmState::Idle;  // no reason to charge
            break;
        }
        if (info.getBattery() < info.getMaxBattery()) {
            m_curr_steps++;
            return Step::Stay;
        }
        m_algo_state = AlgorithmState::Idle;
        break;

    case AlgorithmState::OnPath: {
        Step next_step = popNextStep();
        updateCoordinates(next_step);
        m_curr_steps++;
        return next_step;
    }

    case AlgorithmState::Cleaning:
        if ((m_dirt_sensor->dirtLevel() > 0) && canStay()) {
            m_curr_steps++;
            return Step::Stay;
        }
        m_algo_state = AlgorithmState::Idle;
        break;
    default:
        break;
    }
    return {};
}

void SmartAlgorithm::getNextPath() {
    auto dst_coord = findNeighbourTrip();  // pseudo dfs
    if (dst_coord.has_value()) {
        m_curr_path.push_back(dst_coord.value());
    } else {
        // if there isn't a viable neighbour we do a more through search
        m_tile_manager.updateDockDistance();  // update all distances from docking
        // update graph with current tile as source and range of battery
        m_tile_manager.setSource(m_curr_coord, m_battery_meter->getBatteryState());
        dst_coord = findViableTrip();
        if (atCoordinate(DockingCoords) && !dst_coord.has_value()) {
            return;  // no viable tile to go to
        }
        if (dst_coord.has_value()) {
            m_tile_manager.getPathFromSrc(dst_coord.value(), m_curr_path);
        } else {  // no viable tile :( we return to docking station
            m_tile_manager.getPathFromSrc(DockingCoords, m_curr_path);
        }
    }
    m_algo_state = AlgorithmState::OnPath;
}

std::optional<std::pair<int, int>> SmartAlgorithm::findViableTrip() const {
    std::optional<double> best_score = {};
    std::optional<std::pair<int, int>> best_viable = {};
    const auto& state = getInfo();
    size_t max_trip_length = getMaxTripLength();
    if (max_trip_length <= 1) {
        return {};
    }

    std::vector<AlgoTileInfo> neighbours_info;
    for (auto& [k, tile] : m_tile_manager.valids()) {
        AlgoTileInfo tile_info(k, tile, m_tile_manager, m_tile_manager.getSrcDistance(k));
        if (tile_info.getDistance() == SIZE_MAX || tile_info.getDockDistance() == SIZE_MAX) {
            continue;
        }
        size_t trip_length = tile_info.getDistance() + tile_info.getDockDistance() +
                             ((tile_info.getStatus() == TileStatus::ValidUnvisited) ? 0 : 1);
        if (atCoordinate(k) || trip_length > max_trip_length) {
            continue;
        }
        double score = getValidScore(state, tile_info);
        if (!best_score.has_value() || best_score.value() > score) {
            best_score = score;
            best_viable = k;
        }
    }
    return best_viable;
}
std::optional<std::pair<int, int>> SmartAlgorithm::findNeighbourTrip() const {
    std::optional<double> best_score = {};
    std::optional<std::pair<int, int>> best_viable = {};
    const auto& state = getInfo();
    size_t max_trip_length = getMaxTripLength();
    if (max_trip_length <= 1) {
        return {};
    }
    for (const auto& [k, tile] : m_tile_manager.getNeighbours(m_curr_coord)) {
        AlgoTileInfo tile_info(k, tile, m_tile_manager, 1);
        size_t trip_length = tile_info.getDistance() + tile_info.getDockDistance() +
                             ((tile_info.getStatus() == TileStatus::ValidUnvisited) ? 0 : 1);
        if (tile_info.getStatus() == TileStatus::Done || trip_length > max_trip_length) {
            continue;
        }
        std::optional<double> score = getCloseValidScore(state, tile_info);
        if (!score.has_value()) {
            continue;
        }
        if (!best_score.has_value() || best_score.value() > score.value()) {
            best_score = score;
            best_viable = k;
        }
    }
    return best_viable;
}

void SmartAlgorithm::updateSurroundings() {
    TileStatus prev_status = m_tile_manager.visit(m_curr_coord, m_dirt_sensor->dirtLevel());
    if (prev_status != TileStatus::ValidUnvisited) {
        return;
    }
    for (auto dir : TileManager::Directions) {
        if (!m_wall_sensor->isWall(dir)) {
            m_tile_manager.addNeighbour(m_curr_coord, dir);
        }
    }
}

size_t SmartAlgorithm::getMaxTripLength() const {
    const auto& info = getInfo();
    size_t max_distance =
        (info.getBattery() < info.getStepsLeft()) ? info.getBattery() : info.getStepsLeft();
    return max_distance;
}

const TileInfo& SmartAlgorithm::getNeighbourTile(Direction dir) const {
    return m_tile_manager.tiles().at(getNeighbourCoord(dir));
}
std::pair<int, int> SmartAlgorithm::getNeighbourCoord(Direction dir) const {
    return TileManager::getNeighbourCoord(m_curr_coord, dir);
}

bool SmartAlgorithm::isDone() {
    for (auto& [k, tile] : m_tile_manager.valids()) {
        if (isCleanable(tile)) {
            return false;
        }
    }
    return true;
}

Step SmartAlgorithm::popNextStep() {
    if (m_curr_path.size() == 0) {
        throw std::runtime_error("Algorithm Error: Trying to get step with empty path.");
    }
    Step step = directionToStep(coordinateToDirection(m_curr_path.back()));
    m_curr_path.pop_back();
    if (m_curr_path.size() == 0) {
        m_algo_state = AlgorithmState::Idle;
    }
    return step;
}
size_t SmartAlgorithm::cleanableDistance() const {
    size_t max_steps_d =
        ((m_max_steps - m_curr_steps) > m_max_charge) ? m_max_charge : (m_max_steps - m_curr_steps);
    if (max_steps_d > 1) {
        return (max_steps_d % 2 == 0) ? ((max_steps_d / 2) - 1) : (max_steps_d / 2);
    }
    return 0;
}
bool SmartAlgorithm::isCleanable(const TileInfo& tile) const {
    return cleanableDistance() >= tile.getDockDistance();
}
bool SmartAlgorithm::canStay() const {
    size_t max_steps_d = ((m_max_steps - m_curr_steps) > m_battery_meter->getBatteryState())
                             ? m_battery_meter->getBatteryState()
                             : (m_max_steps - m_curr_steps);
    return max_steps_d > m_tile_manager.getDockDistance(m_curr_coord);
}

void SmartAlgorithm::setWallsSensor(const WallsSensor& wall_sense) { m_wall_sensor = &wall_sense; }
void SmartAlgorithm::setDirtSensor(const DirtSensor& dirt_sense) { m_dirt_sensor = &dirt_sense; }
void SmartAlgorithm::setBatteryMeter(const BatteryMeter& batt_meter) {
    m_battery_meter = &batt_meter;
    m_max_charge = m_battery_meter->getBatteryState();
}
void SmartAlgorithm::setMaxSteps(std::size_t max_steps) { m_max_steps = max_steps; }
bool SmartAlgorithm::atCoordinate(const std::pair<int, int>& coord) const {
    return (m_curr_coord.first == coord.first) && (m_curr_coord.second == coord.second);
}

size_t SmartAlgorithm::AlgoInfo::getBattery() const {
    return m_algo->m_battery_meter->getBatteryState();
}
size_t SmartAlgorithm::AlgoInfo::getMaxBattery() const { return m_algo->m_max_charge; }
size_t SmartAlgorithm::AlgoInfo::getStepsLeft() const {
    return (m_algo->m_curr_steps > m_algo->m_max_steps)
               ? 0
               : (m_algo->m_curr_steps - m_algo->m_max_steps);
}
size_t SmartAlgorithm::AlgoInfo::numberDoneTiles() const {
    return m_algo->m_tile_manager.getNumberOfDone();
}
size_t SmartAlgorithm::AlgoInfo::numberUnvisitedTiles() const {
    return m_algo->m_tile_manager.getNumberOfUnvisited();
}
size_t SmartAlgorithm::AlgoInfo::numberCleanableTiles() const {
    return m_algo->m_tile_manager.getNumberOfDirty();
}
size_t SmartAlgorithm::AlgoTileInfo::getDockDistance() const {
    return m_tile_info.getDockDistance();
}
size_t SmartAlgorithm::AlgoTileInfo::getDistance() const { return m_distance; }
size_t SmartAlgorithm::AlgoTileInfo::getDirtLevel() const {
    if (getStatus() == TileStatus::ValidUnvisited) {
        return 10;
    }
    return m_tile_info.getDirtLevel();
}
TileStatus SmartAlgorithm::AlgoTileInfo::getStatus() const { return m_tile_info.getStatus(); }
size_t SmartAlgorithm::AlgoTileInfo::numUnvisitedNeighbours() const {
    size_t num_unvisited = 0;
    for (const auto& [k, tile] : m_tile_manager.getNeighbours(m_coord)) {
        if (tile.getStatus() == TileStatus::ValidUnvisited) {
            num_unvisited++;
        }
    }
    return num_unvisited;
}
size_t SmartAlgorithm::AlgoTileInfo::numDirtyNeighbours() const {
    size_t num_dirty = 0;
    for (const auto& [k, tile] : m_tile_manager.getNeighbours(m_coord)) {
        if (tile.getStatus() == TileStatus::Dirty) {
            num_dirty++;
        }
    }
    return num_dirty;
}
Direction SmartAlgorithm::coordinateToDirection(const std::pair<int, int>& coord) const {
    if (coord.first == m_curr_x && coord.second == m_curr_y + 1) {
        return Direction::South;
    }
    if (coord.first == m_curr_x && coord.second == m_curr_y - 1) {
        return Direction::North;
    }
    if (coord.first == m_curr_x + 1 && coord.second == m_curr_y) {
        return Direction::East;
    }
    if (coord.first == m_curr_x - 1 && coord.second == m_curr_y) {
        return Direction::West;
    }
    throw std::runtime_error("Trying to get direction from a none neighbour coordinate.\n");
}
void SmartAlgorithm::updateCoordinates(Step step) {
    switch (step) {
    case Step::North:
        m_curr_y--;
        return;
    case Step::South:
        m_curr_y++;
        return;
    case Step::West:
        m_curr_x--;
        return;
    case Step::East:
        m_curr_x++;
        return;
    default:
        return;
    }
}
Step SmartAlgorithm::directionToStep(Direction dir) {
    switch (dir) {
    case Direction::North:
        return Step::North;
    case Direction::South:
        return Step::South;
    case Direction::West:
        return Step::West;
    case Direction::East:
        return Step::East;
    default:
        return Step::Finish;
    }
}