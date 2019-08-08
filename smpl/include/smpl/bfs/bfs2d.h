
#ifndef SMPL_BFS2D_H
#define SMPL_BFS2D_H

#include <stdio.h>
#include <queue>
#include <thread>
#include <tuple>
#include <iostream>

namespace smpl {

class BFS_2D
{
public:

    static const int WALL = 0x7FFFFFFF;
    static const int UNDISCOVERED = 0xFFFFFFFF;

    // X, Y axes
    BFS_2D(int length, int width);
    ~BFS_2D();

    void getDimensions(int& length, int& width);

    void setWall(int x, int y);

    // \brief Clear cells around a given cell until freespace is encountered.
    //
    // Clear cells around a given cell, in bfs-fashion, until a path has been
    // made to an existing free cell.
    //
    // \return false if out of bounds or if there are no free cells, true
    //         otherwise
    bool escapeCell(int x, int y);

    void run(int x, int y);

    /// \brief Run the BFS starting from a variable number of cells
    //template <typename InputIt>
    //void run(InputIt cells_begin, InputIt cells_end);

    //void run_components(int gx, int gy);

    bool inBounds(int x, int y) const;

    /// \brief Return the distance, in cells, to the nearest occupied cell.
    ///
    int getDistance(int x, int y) const;

    /// \brief Return whether this cell has been discovered.
    ///
    bool isUndiscovered(int x, int y) const;

    int getNearestFreeNodeDist(int x, int y);
    bool isWall(int x, int y) const;


    public:
    int m_dim_x, m_dim_y;
    int m_dim_xy;

    int* m_distance_grid;

    int* m_queue;
    int m_queue_head, m_queue_tail;

    int m_neighbor_offsets[8];
    std::vector<bool> m_closed;
    std::vector<int> m_distances;

    int getNode(int x, int y) const;
    bool getCoord(int node, int& x, int& y) const;
    void setWall(int node);
    void unsetWall(int node);
    bool isWall(int node) const;
    int isUndiscovered(int node) const;
    int neighbor(int node, int neighbor) const;
};

inline bool BFS_2D::inBounds(int x, int y) const
{
    return !(x < 0 || y < 0 ||
            x >= m_dim_x - 2 || y >= m_dim_y - 2);
}

inline int BFS_2D::getNode(int x, int y) const
{
    if (!inBounds(x, y)) {
        return -1;
    }

    return (y + 1)*m_dim_x + (x + 1);
}

inline bool BFS_2D::getCoord(int node, int& x, int& y) const
{
    if (node < 0 || node >= m_dim_xy) {
        return false;
    }

    x = (node % m_dim_x) - 1;
    y = (node / m_dim_x) - 1;
    return true;
}

inline void BFS_2D::setWall(int node)
{
    m_distance_grid[node] = WALL;
}

inline void BFS_2D::unsetWall(int node)
{
    m_distance_grid[node] = UNDISCOVERED;
}

inline bool BFS_2D::isWall(int node) const
{
    return m_distance_grid[node] == WALL;
}

inline int BFS_2D::isUndiscovered(int node) const
{
    return m_distance_grid[node] < 0;
}

inline int BFS_2D::neighbor(int node, int neighbor) const
{
    return node + m_neighbor_offsets[neighbor];
}

} // namespace smpl

#endif
