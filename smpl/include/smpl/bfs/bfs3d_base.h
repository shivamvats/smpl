#ifndef SMPL_BFS3D_BASE_H
#define SMPL_BFS3D_BASE_H

#include <stdio.h>
#include <queue>
#include <thread>
#include <tuple>
#include <iostream>

namespace smpl {

class BFS_3D_Base
{
public:

    static const int WALL = 0x7FFFFFFF;
    static const int UNDISCOVERED = 0xFFFFFFFF;

    BFS_3D_Base(int length, int width, int height);
    ~BFS_3D_Base();

    void getDimensions(int* length, int* width, int* height);

    void setWall(int x, int y, int z);

    void run(int x, int y, int z);

    using BaseState = std::array<int, 3>;
    std::vector<BaseState> getPath(int x, int y, int z);

    /// \brief Run the BFS starting from a variable number of cells
    template <typename InputIt>
    void run(InputIt cells_begin, InputIt cells_end);

    bool inBounds(int x, int y, int z) const;

    /// \brief Return the distance, in cells, to the nearest occupied cell.
    ///
    /// This function is blocking if the BFS is running in parallel and a value
    /// has not yet been computed for this cell.
    inline int getDistance(int x, int y, int z) const{
        int node = getNode(x, y, z);
        while (m_running && m_distance_grid[node] < 0);
        return m_distance_grid[node];
    }

    /// \brief Return whether this cell has been discovered.
    ///
    /// This function is blocking if the BFS is running. Once the BFS has
    /// finished, this function will return true for cells that are isolated
    /// from the region of the grid containing the start cell.
    bool isUndiscovered(int x, int y, int z) const;

    int getNearestFreeNodeDist(int x, int y, int z);
    bool isWall(int x, int y, int z) const;

    bool isRunning() const { return m_running; }

    int countWalls() const;
    int countUndiscovered() const;
    int countDiscovered() const;

private:

    std::thread m_search_thread;

    int m_dim_x, m_dim_y, m_dim_z;
    int m_dim_xy, m_dim_xyz;

    int volatile* m_distance_grid;

    int* m_queue;
    int m_queue_head, m_queue_tail;

    volatile bool m_running;

    int m_neighbor_offsets[26];
    std::vector<bool> m_closed;
    std::vector<int> m_distances;

    int getNode(int x, int y, int z) const;
    bool getCoord(int node, int& x, int& y, int& z) const;
    void setWall(int node);
    void unsetWall(int node);
    bool isWall(int node) const;
    int isUndiscovered(int node) const;
    int neighbor(int node, int neighbor) const;

    void search(
        int width,
        int planeSize,
        int volatile* distance_grid,
        int* queue,
        int& queue_head,
        int& queue_tail);

    template <typename Visitor>
    void visit_free_cells(int node, const Visitor& visitor);
};

inline bool BFS_3D_Base::inBounds(int x, int y, int z) const
{
    return !(x < 0 || y < 0 || z < 0 ||
            x >= m_dim_x - 2 || y >= m_dim_y - 2 || z >= m_dim_z - 2);
}

template <typename InputIt>
void BFS_3D_Base::run(InputIt cells_begin, InputIt cells_end)
{
    if (m_running) {
        return;
    }

    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    m_queue_head = 0;

    // seed the search with all start cells
    int xyz[3];
    int ind = 0;
    int start_count = 0;
    for (auto it = cells_begin; it != cells_end;) {
        if (ind == 3) {
            auto origin = getNode(xyz[0], xyz[1], xyz[2]);
            m_queue[start_count++] = origin;
            m_distance_grid[origin] = 0;
            ind = 0;
        } else {
            xyz[ind++] = *it++;
        }
    }

    m_queue_tail = start_count;

    // fire off background thread to compute bfs
    m_search_thread = std::thread([&]()
    {
        this->search(m_dim_x, m_dim_xy, m_distance_grid, m_queue, m_queue_head, m_queue_tail);
    });

    m_running = true;
}

inline int BFS_3D_Base::getNode(int x, int y, int z) const
{
    if (!inBounds(x, y, z) && !inBounds(x, y, z-1) && !inBounds(x, y, z+1)) {
        return -1;
    }

    // 2PI = 0.
    // Normalize negative angle
    return (( (z + m_dim_z - 2) % (m_dim_z - 2) ) + 1) * m_dim_xy + (y + 1) * m_dim_x + (x + 1);
}

inline bool BFS_3D_Base::getCoord(int node, int& x, int& y, int& z) const
{
    if (node < 0 || node >= m_dim_xyz) {
        return false;
    }

    int zz = node / m_dim_xy;
    int yy = (node - zz * m_dim_xy) / m_dim_x;
    int xx = node - zz * m_dim_xy - yy * m_dim_x;
    z = zz - 1;
    y = yy - 1;
    x = xx - 1;
    return true;
}

inline void BFS_3D_Base::setWall(int node)
{
    m_distance_grid[node] = WALL;
}

inline void BFS_3D_Base::unsetWall(int node)
{
    m_distance_grid[node] = UNDISCOVERED;
}

inline bool BFS_3D_Base::isWall(int node) const
{
    return m_distance_grid[node] == WALL;
}

inline int BFS_3D_Base::isUndiscovered(int node) const
{
    return m_distance_grid[node] < 0;
}

inline int BFS_3D_Base::neighbor(int node, int neighbor) const
{
    return node + m_neighbor_offsets[neighbor];
}

} // namespace smpl

#endif
