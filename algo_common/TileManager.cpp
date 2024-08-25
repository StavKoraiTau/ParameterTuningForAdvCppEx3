#include "algo_common/TileManager.h"

const std::vector<Direction> TileManager::Directions = {Direction::North, Direction::South,
                                                        Direction::West, Direction::East};

TileStatus TileManager::visit(const std::pair<int, int>& coord, int tile_dirt_level) {
    auto tile_itr = m_tiles.find(coord);
    TileStatus prev_status;
    if (tile_itr == m_tiles.end()) {  // add tile if it dosen't exist
        tile_itr = m_tiles.emplace(coord, TileInfo(0, SIZE_MAX, TileStatus::Done))
                       .first;  // the new tile iterator
        m_tile_graph.addNode(coord);
        addEdgesToExistingNeighbours(coord);
        prev_status = TileStatus::ValidUnvisited;
    } else {
        prev_status = (*tile_itr).second.getStatus();
    }
    TileInfo& tile = (*tile_itr).second;
    tile.setDirtLevel(tile_dirt_level);
    tile.setStatus(tile_dirt_level > 0 ? TileStatus::Dirty : TileStatus::Done);

    auto tile_v_itr = m_valid_tiles.find(coord);  // update valid map if needed
    if (tile_v_itr == m_valid_tiles.end() && tile_dirt_level > 0) {
        m_valid_tiles.insert({coord, tile});
    }
    if (tile_v_itr != m_valid_tiles.end() && tile_dirt_level == 0) {
        m_valid_tiles.erase(coord);
    }
    if (prev_status == TileStatus::ValidUnvisited) {
        m_num_visited++;
    }
    return prev_status;
}
void TileManager::addNeighbour(const std::pair<int, int>& curr_coord, Direction dir) {
    std::pair<int, int> neigh_coord = getNeighbourCoord(curr_coord, dir);
    auto tile_itr = m_tiles.find(neigh_coord);
    if (tile_itr == m_tiles.end()) {  // this neighbour tile dosen't exist
        tile_itr = m_tiles
                       .emplace(neigh_coord, TileInfo(0, m_tiles[curr_coord].getDockDistance() + 1,
                                                      TileStatus::ValidUnvisited))
                       .first;  // the new tile iterator
        m_valid_tiles.insert({neigh_coord, (*tile_itr).second});
        m_tile_graph.addNode(neigh_coord);
        addEdgesToExistingNeighbours(neigh_coord);
        return;
    }
    TileInfo& tile = (*tile_itr).second;
    if (tile.getStatus() == TileStatus::Done) {
        return;  // we already visited this neighbour tile so we don't have any new info
    }
    // add edge to graph if it dosen't exist already
    tryAddingGraphEdge(curr_coord, neigh_coord);
    tile.relaxDockDistance(m_tiles[curr_coord].getDockDistance() + 1);
}

std::pair<int, int> TileManager::getNeighbourCoord(const std::pair<int, int>& curr_coord,
                                                   Direction dir) {
    int neigh_x = curr_coord.first;
    int neigh_y = curr_coord.second;
    switch (dir) {
    case Direction::North:
        neigh_y -= 1;
        break;
    case Direction::South:
        neigh_y += 1;
        break;
    case Direction::West:
        neigh_x -= 1;
        break;
    case Direction::East:
        neigh_x += 1;
        break;
    default:
        break;
    }
    return std::make_pair(neigh_x, neigh_y);
}
bool TileManager::addEdgesToExistingNeighbours(const std::pair<int, int>& tile_coord) {
    size_t tile_dist = m_tiles[tile_coord].getDockDistance();
    bool added_edge = false;
    for (auto dir : Directions) {
        auto next_neigh_coord = getNeighbourCoord(tile_coord, dir);
        auto next_neigh_itr = m_tiles.find(next_neigh_coord);
        if (next_neigh_itr != m_tiles.end()) {
            added_edge = tryAddingGraphEdge(tile_coord, next_neigh_coord) || added_edge;
            (*next_neigh_itr).second.relaxDockDistance(tile_dist + 2);
        }
    }
    return added_edge;
}
void TileManager::updateDockDistance() {
    if (m_valid_dock_dist) {
        return;  // no edge was added since last update
    }
    m_tile_graph.runBFS(m_dock_coord);
    for (auto& [k, tile] : m_tiles) {
        tile.relaxDockDistance(m_tile_graph.getDistance(k));
    }
    m_curr_src = m_dock_coord;
    m_curr_max_distance = SIZE_MAX;
    m_valid_dock_dist = true;
    m_valid_src = true;
}
size_t TileManager::getDockDistance(const std::pair<int, int>& coord, bool lazy) {
    auto tile_itr = m_tiles.find(coord);
    if (tile_itr == m_tiles.end()) {
        return SIZE_MAX;
    }
    if (lazy) {
        return (*tile_itr).second.getDockDistance();
    }
    updateDockDistance();  // iterator is still valid after
    return (*tile_itr).second.getDockDistance();
}
size_t TileManager::getDockDistance(const std::pair<int, int>& coord) const {
    auto tile_itr = m_tiles.find(coord);
    if (tile_itr == m_tiles.end()) {
        return SIZE_MAX;
    }
    return (*tile_itr).second.getDockDistance();
}
void TileManager::setSource(const std::pair<int, int>& src_coord, size_t max_dist) {
    if (m_valid_src && m_curr_src.first == src_coord.first &&
        m_curr_src.second == src_coord.second && m_curr_max_distance >= max_dist) {
        return;
    }
    m_tile_graph.runBFS(src_coord, max_dist);
    m_curr_src = src_coord;
    m_curr_max_distance = max_dist;
    m_valid_src = true;
}
size_t TileManager::getSrcDistance(const std::pair<int, int>& dst_coord) const {
    return m_tile_graph.getDistance(dst_coord);
}
void TileManager::getPathFromSrc(const std::pair<int, int>& dst_coord,
                                 std::vector<std::pair<int, int>>& out_path) {
    m_tile_graph.getPath(dst_coord, out_path);
}
std::optional<std::vector<std::pair<int, int>>> TileManager::getPathFromSrc(
    const std::pair<int, int>& dst_coord) {
    return m_tile_graph.getPath(dst_coord);
}
bool TileManager::tryAddingGraphEdge(const std::pair<int, int>& node_key_1,
                                     const std::pair<int, int>& node_key_2) {
    bool was_added = m_tile_graph.addIfNoEdge(node_key_1, node_key_2);
    if (was_added) {
        m_valid_dock_dist = false;
        m_valid_src = false;
    }
    return was_added;
}
std::vector<std::pair<std::pair<int, int>, const TileInfo&>> TileManager::getNeighbours(
    const std::pair<int, int>& coord) const {
    std::vector<std::pair<std::pair<int, int>, const TileInfo&>> neighbours;
    for (const auto& dir : Directions) {
        auto tile_itr = m_tiles.find(getNeighbourCoord(coord, dir));
        if (tile_itr == m_tiles.end()) {
            continue;
        }
        neighbours.emplace_back(*tile_itr);
    }
    return neighbours;
}