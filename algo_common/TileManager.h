#ifndef __TILE_MANAGER_H__
#define __TILE_MANAGER_H__
#include <cstdint>
#include <map>
#include <vector>

#include "algo_common/PathingGraph.h"
#include "common/enums.h"

enum class TileStatus { ValidUnvisited, Dirty, Done };
class TileInfo {
    int m_dirt_level;
    size_t m_dock_distance;
    TileStatus m_status;

public:
    TileInfo() : TileInfo(0, SIZE_MAX, TileStatus::ValidUnvisited) {}
    TileInfo(int init_dirt, size_t dist_init, TileStatus init_status)
        : m_dirt_level(init_dirt), m_dock_distance(dist_init), m_status(init_status) {}
    /*
    Get the dirt level of a visited tile (will return 0 for unvisited ones).
    (currently not in use but)
    */
    inline int getDirtLevel() const { return m_dirt_level; }
    inline size_t getDockDistance() const { return m_dock_distance; }
    inline TileStatus getStatus() const { return m_status; }
    inline void setDirtLevel(int dirt_level) { m_dirt_level = dirt_level; }
    /*
    Relax the current distance of the tile from the docking station, meaning we only change it if
    the distance is lesser then the current dock distance
    */
    inline void relaxDockDistance(size_t distance) {
        m_dock_distance = (m_dock_distance > distance) ? distance : m_dock_distance;
    }
    inline void setStatus(TileStatus status) { m_status = status; }
};

class TileManager {
    std::map<std::pair<int, int>, TileInfo> m_tiles;
    // we can keep refrence since its a map and not a unordered map
    std::map<std::pair<int, int>, TileInfo&> m_valid_tiles;
    size_t m_num_visited;
    PathingGraph m_tile_graph;

    const std::pair<int, int> m_dock_coord;
    /*
    Dock distance is valid if we haven't added an edge since last update
    */
    bool m_valid_dock_dist;

    bool m_valid_src;
    std::pair<int, int> m_curr_src;
    size_t m_curr_max_distance;
    /*
    Add edges to already existing neighbour tiles surounding the passed tile coord
    */
    bool addEdgesToExistingNeighbours(const std::pair<int, int>& tile_coord);
    bool tryAddingGraphEdge(const std::pair<int, int>& node_key_1,
                            const std::pair<int, int>& node_key_2);

public:
    static const std::vector<Direction> Directions;
    TileManager(const std::pair<int, int>& dock_coord)
        : m_tiles(),
          m_valid_tiles(),
          m_num_visited(0),
          m_tile_graph(),
          m_dock_coord(dock_coord),
          m_valid_dock_dist(false),
          m_valid_src(false),
          m_curr_src(dock_coord),
          m_curr_max_distance(0) {
        // add docking station to tiles and to graph with distance and dirt 0 (and mark as
        // unvisited)
        m_tiles.emplace(dock_coord, TileInfo(0, 0, TileStatus::ValidUnvisited));
        m_tile_graph.addNode(dock_coord);
    }
    /*
    Visit a tile updating its parameters and status, does not add neighbours.
    */
    TileStatus visit(const std::pair<int, int>& coord, int tile_dirt_level);
    /*
    Add a neighbour in the passed direction to curr coord, will not add if already exists, does also
    update tile connections for paths and distances.
    */
    void addNeighbour(const std::pair<int, int>& curr_coord, Direction dir);
    /*
    Set the source tile from which subsequent getSrcDistance and getSrcPath will correspond to.
    */
    void setSource(const std::pair<int, int>& coord1, size_t max_dist = SIZE_MAX);
    size_t getSrcDistance(const std::pair<int, int>& dst_coord) const;
    /*
    Get the current saved dock distance (not necessarily the actual shortest distance, but it is a
    valid distance of some path), if lazy=false will update all dock distances to the actual
    shortest distance and then return.
    */
    size_t getDockDistance(const std::pair<int, int>& coord, bool lazy = true);
    /*
    Get the current saved dock distance (not necessarily the actual shortest distance, but it is a
    valid distance of some path)
    */
    size_t getDockDistance(const std::pair<int, int>& coord) const;
    /*
    Load into out_path vector the path from the current source to the passed dst_coord.
    The order of the coords to pass through is from the last to the first meaning the last element
    (vec.back()) is the next step from source. We can just use as stack of coordiantes.
    out_path is of size 0 if there is no path or dst=src
    */
    void getPathFromSrc(const std::pair<int, int>& dst_coord,
                        std::vector<std::pair<int, int>>& out_path);
    /*
    Return a vector of coords corresponding to the path from the current source to the passed
    dst_coord. The order of the coords to pass through is from the last to the first meaning the
    last element (vec.back()) is the next step from source. We can just use as stack of coordiantes.
    no value if dst_coord is not reachable from source.
    size = 0 if dst=src
    */
    std::optional<std::vector<std::pair<int, int>>> getPathFromSrc(
        const std::pair<int, int>& dst_coord);
    /*
    Update the dock distance of all currently saved tiles
    */
    void updateDockDistance();
    /*
    Map of tiles whose status is not done.
    */
    inline const std::map<std::pair<int, int>, TileInfo&>& valids() const { return m_valid_tiles; }
    /*
    Map of all saved tiles.
    */
    inline const std::map<std::pair<int, int>, TileInfo>& tiles() const { return m_tiles; }
    // Get neighbours of given coord, they are given in [k,tile_info] format
    std::vector<std::pair<std::pair<int, int>, const TileInfo&>> getNeighbours(
        const std::pair<int, int>& coord) const;
    /*
    Access tiles currently saved.
    */
    inline const TileInfo& operator[](const std::pair<int, int>& coord) const {
        return m_tiles.at(coord);
    }
    inline size_t getNumberOfVisited() const { return m_num_visited; }
    inline size_t getNumberOfDone() const { return m_tiles.size() - m_valid_tiles.size(); }
    inline size_t getNumberOfUnvisited() const { return m_tiles.size() - m_num_visited; }
    inline size_t getNumberOfDirty() const { return m_valid_tiles.size() - getNumberOfUnvisited(); }
    /*
    Get the coordinate of a neighbour tile of curr_coord in the direction dir.
    */
    static std::pair<int, int> getNeighbourCoord(const std::pair<int, int>& curr_coord,
                                                 Direction dir);
};

#endif  //__TILE_MANAGER_H__