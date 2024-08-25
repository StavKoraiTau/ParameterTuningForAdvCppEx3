#include "algo_common/PathingGraph.h"

#include <algorithm>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>
void PathingGraph::Node::addNeighbour(size_t neighbour_id) {
    if (std::find(m_neighbours.begin(), m_neighbours.end(), neighbour_id) != m_neighbours.end()) {
        return;  // already a neighbour
    }
    m_neighbours.push_back(neighbour_id);
}
size_t PathingGraph::translateKey(const std::pair<int, int>& key) const {
    auto node_itr = m_translation_map.find(key);
    if (node_itr == m_translation_map.end()) {
        return INVALID_ID;  // no key found
    }
    return (*node_itr).second;
}
void PathingGraph::addEdge(const std::pair<int, int>& node_key_1,
                           const std::pair<int, int>& node_key_2) {
    size_t node1_id = translateKey(node_key_1);
    size_t node2_id = translateKey(node_key_2);
    if (node1_id == INVALID_ID || node2_id == INVALID_ID) {
        throw std::runtime_error("Trying to add a edge to node that dosen't exist.\n");
    }
    m_nodes[node1_id].addNeighbour(node2_id);
    m_nodes[node2_id].addNeighbour(node1_id);
}
bool PathingGraph::isEdge(const std::pair<int, int>& node_key_1,
                          const std::pair<int, int>& node_key_2) const {
    size_t node1_id = translateKey(node_key_1);
    size_t node2_id = translateKey(node_key_2);
    if (node1_id == INVALID_ID || node2_id == INVALID_ID) {
        return false;
    }
    return isEdge(node1_id, node2_id);
}
bool PathingGraph::isEdge(size_t node1_id, size_t node2_id) const {
    for (auto neigh_node_id : m_nodes[node1_id].neighbours()) {
        if (neigh_node_id == node2_id) {
            return true;
        }
    }
    return false;
}
bool PathingGraph::addIfNoEdge(const std::pair<int, int>& node_key_1,
                               const std::pair<int, int>& node_key_2) {
    size_t node1_id = translateKey(node_key_1);
    size_t node2_id = translateKey(node_key_2);
    if (node1_id == INVALID_ID || node2_id == INVALID_ID) {
        throw std::runtime_error("Trying to add a edge to node that dosen't exist.\n");
    }
    if (isEdge(node1_id, node2_id)) {
        return false;
    }
    m_nodes[node1_id].addNeighbour(node2_id);
    m_nodes[node2_id].addNeighbour(node1_id);
    return true;
}
void PathingGraph::addNode(const std::pair<int, int>& node_key) {
    if (translateKey(node_key) != INVALID_ID) {
        throw std::runtime_error("Trying to add a node that already exists.\n");
    }
    m_translation_map[node_key] = m_next_id;
    m_next_id++;
    m_nodes.emplace_back(node_key);
}

size_t PathingGraph::getDistance(const std::pair<int, int>& node_key) const {
    size_t node_id = translateKey(node_key);
    if (node_id == INVALID_ID) {
        throw std::runtime_error("Trying to get distance from a node that doesn't exists.\n");
    }
    if (m_cur_src == INVALID_ID) {
        throw std::runtime_error("Trying to get distance before running BFS.\n");
    }
    return m_nodes[node_id].getDistance();
}
std::optional<std::pair<int, int>> PathingGraph::getParent(
    const std::pair<int, int>& node_key) const {
    size_t node_id = translateKey(node_key);
    if (m_cur_src == INVALID_ID) {
        throw std::runtime_error("Trying to get parent before running BFS.\n");
    }
    if (node_id == INVALID_ID) {
        throw std::runtime_error("Trying to get parent from a node that doesn't exists.\n");
    }
    size_t parent_id = m_nodes[node_id].getParent();
    if (parent_id != INVALID_ID) {
        return m_nodes[parent_id].getKey();
    }
    return {};  // no parents :(
}

/* For clearity Initialize BFS algorithm by assignment of: delta:=inf, pi:=not exist, color := White
 */
void PathingGraph::bfsInit() {
    for (auto& node : m_nodes) {
        node.setDistance(SIZE_MAX);
        node.setParent(INVALID_ID);
        node.setStatus(BFSStatus::Init);
    }
    m_nodes[m_cur_src].setDistance(0);
    m_nodes[m_cur_src].setParent(m_cur_src);  // it's own parent
    m_bfs_queue.push(m_cur_src);
    m_nodes[m_cur_src].setStatus(BFSStatus::InQueue);
}
/* Checking the neighbours */
void PathingGraph::bfsVisit(size_t node_id) {
    for (auto neighbour_id : m_nodes[node_id].neighbours()) {
        /* If white put in the queue */
        if (m_nodes[neighbour_id].getStatus() == BFSStatus::Init) {
            m_nodes[neighbour_id].setDistance(m_nodes[node_id].getDistance() + 1);
            m_nodes[neighbour_id].setParent(node_id);
            m_nodes[neighbour_id].setStatus(BFSStatus::InQueue);
            m_bfs_queue.push(neighbour_id);
        }
    }
    m_nodes[node_id].setStatus(BFSStatus::Done);
}

void PathingGraph::runBFS(const std::pair<int, int>& source_node_key, size_t max_distance) {
    m_cur_src = translateKey(source_node_key);
    if (m_cur_src == INVALID_ID) {
        throw std::runtime_error("Trying to run BFS from a node that doesn't exists.\n");
    }

    while (!m_bfs_queue.empty()) {
        m_bfs_queue.pop();
    }
    bfsInit();
    while (!m_bfs_queue.empty()) {
        size_t curr_id = m_bfs_queue.front();
        if (m_nodes[curr_id].getDistance() > max_distance) {
            return;
        }
        bfsVisit(curr_id);
        m_bfs_queue.pop();
    }
}
bool PathingGraph::isReachable(const std::pair<int, int>& dst_key) const {
    return getParent(dst_key).has_value();
}
std::optional<std::vector<std::pair<int, int>>> PathingGraph::getPath(
    const std::pair<int, int>& dst_key) const {
    size_t dst_id = translateKey(dst_key);
    if (dst_id == INVALID_ID) {
        throw std::runtime_error("Trying to get path to a node that doesn't exists.\n");
    }
    if (m_cur_src == INVALID_ID) {
        throw std::runtime_error("Trying to get path before running BFS.\n");
    }
    if (m_nodes[dst_id].getParent() == INVALID_ID) {
        // if we can't reach a node
        return {};
    }
    std::vector<std::pair<int, int>> path;
    getPath(dst_id, path);
    // we love move/RVO
    return path;
}
void PathingGraph::getPath(const std::pair<int, int>& dst_key,
                           std::vector<std::pair<int, int>>& out_path) const {
    size_t dst_id = translateKey(dst_key);
    if (dst_id == INVALID_ID) {
        throw std::runtime_error("Trying to get path to a node that doesn't exists.\n");
    }
    if (m_cur_src == INVALID_ID) {
        throw std::runtime_error("Trying to get path before running BFS.\n");
    }
    getPath(dst_id, out_path);
}
void PathingGraph::getPath(size_t dst_id, std::vector<std::pair<int, int>>& out_path) const {
    out_path.clear();
    if (m_nodes[dst_id].getParent() == INVALID_ID) {
        // if we can't reach a node
        return;
    }
    size_t curr_id = dst_id;
    while (curr_id != m_cur_src) {
        out_path.push_back(m_nodes[curr_id].getKey());
        curr_id = m_nodes[curr_id].getParent();
    }
}