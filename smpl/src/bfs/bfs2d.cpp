#include <smpl/bfs/bfs2d.h>

#include <smpl/console/console.h>

namespace smpl {

BFS_2D::BFS_2D(int length, int width) :
    m_dim_x(),
    m_dim_y(),
    m_distance_grid(nullptr),
    m_queue(nullptr),
    m_queue_head(),
    m_queue_tail(),
    m_neighbor_offsets(),
    m_closed(),
    m_distances()
{
    if (width <= 0 || length <= 0) {
        return;
    }

    m_dim_x = length + 2;
    m_dim_y = width + 2;

    m_dim_xy = m_dim_x * m_dim_y;

    m_neighbor_offsets[0] = -m_dim_x;
    m_neighbor_offsets[1] = 1;
    m_neighbor_offsets[2] = m_dim_x;
    m_neighbor_offsets[3] = -1;
    m_neighbor_offsets[4] = -m_dim_x-1;
    m_neighbor_offsets[5] = -m_dim_x+1;
    m_neighbor_offsets[6] = m_dim_x+1;
    m_neighbor_offsets[7] = m_dim_x-1;

    m_distance_grid = new int[m_dim_xy];
    m_queue = new int[width*length];

    for (int node = 0; node < m_dim_xy; node++) {
        int x = node % m_dim_x;
        int y = node / m_dim_x;
        if (x == 0 || x == m_dim_x - 1 ||
            y == 0 || y == m_dim_y - 1)
        {
            m_distance_grid[node] = WALL;
        }
        else {
            m_distance_grid[node] = UNDISCOVERED;
        }
    }
}

BFS_2D::~BFS_2D()
{
    if (m_distance_grid) {
        delete[] m_distance_grid;
    }
    if (m_queue) {
        delete[] m_queue;
    }
}

void BFS_2D::getDimensions(int& length, int& width)
{
    length = m_dim_x - 2;
    width = m_dim_y - 2;
}

void BFS_2D::setWall(int x, int y)
{
    int node = getNode(x, y);
    m_distance_grid[node] = WALL;
}

bool BFS_2D::isWall(int x, int y) const
{
    int node = getNode(x, y);
    return m_distance_grid[node] == WALL;
}

bool BFS_2D::isUndiscovered(int x, int y) const
{
    int node = getNode(x, y);
    return m_distance_grid[node] == UNDISCOVERED;
}

void BFS_2D::run(int x, int y)
{
    for (int i = 0; i < m_dim_xy; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // get index of start coordinate
    int origin = getNode(x, y);

    // initialize the queue
    m_queue_head = 0;
    m_queue_tail = 1;
    m_queue[0] = origin;

    // initialize starting distance
    m_distance_grid[origin] = 0;

    auto sqrd_dist = [&](int a, int b){return (a-x)*(a-x) + (b-y)*(b-y);};

    while (m_queue_head < m_queue_tail) {
        int currentNode = m_queue[m_queue_head++];
        int currentCost = m_distance_grid[currentNode] + 1;

        for( int i=0; i<8; i++ ){
            int childNode = currentNode + m_neighbor_offsets[i];
            if (m_distance_grid[childNode] < 0) {
                m_queue[m_queue_tail++] = childNode;
                // Goal regions
                // Circle of radius 0.6 m is considered part of goal region.
                if(sqrd_dist(childNode%m_dim_x, childNode/m_dim_x) < 144)
                    m_distance_grid[childNode] = 0;
                else
                    m_distance_grid[childNode] = currentCost;
            }
        }
    }
}

int BFS_2D::getDistance(int x, int y) const
{
    int node = getNode(x, y);
    return m_distance_grid[node];
}

int BFS_2D::countWalls() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xy; ++i) {
        if (m_distance_grid[i] == WALL) {
            ++count;
        }
    }
    return count;
}

int BFS_2D::countUndiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xy; ++i) {
        if (m_distance_grid[i] == UNDISCOVERED) {
            ++count;
        }
    }
    return count;
}

int BFS_2D::countDiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xy; ++i) {
        if (m_distance_grid[i] != WALL && m_distance_grid[i] >= 0) {
            ++count;
        }
    }
    return count;
}

void BFS_2D::printGrid(){
    for(int i=0; i<m_dim_xy; i++){
        if( i % m_dim_x == 0 )
            std::cout<<"\n";
        if(m_distance_grid[i] == UNDISCOVERED)
            std::cout<<"o"<<"  ";
        else if(m_distance_grid[i] == WALL)
            std::cout<<"x"<<"  ";
        else
            std::cout<<m_distance_grid[i]<<"  ";
    }
    std::cout<<"\n";
}

} // namespace smpl
