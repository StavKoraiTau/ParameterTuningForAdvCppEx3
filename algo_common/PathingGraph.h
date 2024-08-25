#ifndef __PATHING_GRAPH_H__
#define __PATHING_GRAPH_H__

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <queue>
#include <vector>

#include "common/enums.h"

/*
Data structure to store a graph with pair<int,int> as keys and ability to run BFS and get distances.
(Could have used templates but rather not implement in the header file or lose generality by
specifing types in header)
*/
class PathingGraph {
    /*
    The interface to the graph is by the key but the internal handling is by assigning an unique id
    to each key and then using a vector instead of map (nicer and faster)
    */
    enum class BFSStatus { Init, InQueue, Done };

    /*Internal data structure to represent a node in the graph*/
    class Node {
        const std::pair<int, int> m_key;   // ??
        std::vector<size_t> m_neighbours;  // Node neighbours
        size_t m_parent;                   // Pi of the node
        size_t m_distance;                 // Delta of the node from s
        BFSStatus m_status;                // Node color: white\grey\black

    public:
        Node(const std::pair<int, int>& key)
            : m_key(key), m_parent(INVALID_ID), m_distance(INVALID_ID) {}
        inline const std::pair<int, int>& getKey() const { return m_key; }
        /*const refrence to the vector of neighbour ids (nice for iterating over them)*/
        inline const std::vector<size_t>& neighbours() const { return m_neighbours; }
        /*refrence to the vector of neighbour ids (nice for iterating over them and adding)*/
        inline std::vector<size_t>& neighbours() { return m_neighbours; }
        void addNeighbour(size_t neighbour_id);
        inline size_t getParent() const { return m_parent; }
        inline void setParent(size_t parent_id) { m_parent = parent_id; }
        inline size_t getDistance() const { return m_distance; }
        inline void setDistance(size_t distance) { m_distance = distance; }
        inline BFSStatus getStatus() const { return m_status; }
        inline void setStatus(BFSStatus status) { m_status = status; }
    };
    std::map<std::pair<int, int>, size_t> m_translation_map;
    std::vector<Node> m_nodes;
    static constexpr size_t INVALID_ID = SIZE_MAX;
    size_t m_next_id;
    size_t m_cur_src;
    std::queue<size_t> m_bfs_queue;
    size_t translateKey(const std::pair<int, int>& key) const;
    void bfsInit();
    void bfsVisit(size_t node_id);
    bool isEdge(size_t node1_id, size_t node2_id) const;
    void getPath(size_t node_id, std::vector<std::pair<int, int>>& out_path) const;

public:
    PathingGraph() : m_next_id(0), m_cur_src(INVALID_ID) {}
    void addEdge(const std::pair<int, int>& node_key_1, const std::pair<int, int>& node_key_2);
    /*
    Add an edge between two nodes if it dosen't exist already
    */
    bool addIfNoEdge(const std::pair<int, int>& node_key_1, const std::pair<int, int>& node_key_2);
    bool isEdge(const std::pair<int, int>& node_key_1, const std::pair<int, int>& node_key_2) const;
    void addNode(const std::pair<int, int>& node_key);
    size_t getDistance(const std::pair<int, int>& node_key) const;
    std::optional<std::pair<int, int>> getParent(const std::pair<int, int>& node_key) const;
    /*
    Runs BFS from given source node key and up to max distance from it.
    After running a BFS graph distances and paths are updated to fit this BFS.
    */
    void runBFS(const std::pair<int, int>& source_node_key, size_t max_distance = SIZE_MAX);
    /*
    Return a vector with the order of the nodes to pass through, from the last to
    the first meaning the last element (vec.back()) is the next step from source.
    We can just use as stack of coordiantes.
    Return has value if there is a path otherwise no value (and vector of size 0 if dst=src)
    */
    std::optional<std::vector<std::pair<int, int>>> getPath(
        const std::pair<int, int>& dst_key) const;
    /*
    Load path into passed vector refrence the order of the nodes to pass through is from the last to
    the first meaning the last element (vec.back()) is the next step from source.
    We can just use as stack of coordiantes.
    out_path is of size 0 if there is no path or dst=src
     */
    void getPath(const std::pair<int, int>& dst_key,
                 std::vector<std::pair<int, int>>& out_path) const;
    bool isReachable(const std::pair<int, int>& dst_key) const;
};
#endif  //__PATHING_GRAPH_H__