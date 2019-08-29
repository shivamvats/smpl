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

void BFS_2D::setGoalCells(std::vector<int>& xs, std::vector<int>& ys){
    assert(xs.size() == ys.size());
    for(int i=0; i<xs.size(); i++){
        m_distance_grid[getNode(xs[i], ys[i])] = 0;
    }
}

#define EXPAND_NEIGHBOR(offset)                                 \
        if (m_distance_grid[currentNode + offset] < 0) {        \
            m_queue[m_queue_tail++] = currentNode + offset;     \
            m_distance_grid[currentNode + offset] = currentCost;\
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

    while (m_queue_head < m_queue_tail) {
        int currentNode = m_queue[m_queue_head++];
        int currentCost = m_distance_grid[currentNode] + 1;

        for( int i=0; i<8; i++ ){
            EXPAND_NEIGHBOR(m_neighbor_offsets[i]);
        }
    }
}

#undef EXPAND_NEIGHBOR

int BFS_2D::getDistance(int x, int y) const
{
    int node = getNode(x, y);
    return m_distance_grid[node];
}

/*
int BFS_2D::getNearestFreeNodeDist(int x, int y, int z)
{
    // initialize closed set and distances
    m_closed.assign(m_dim_xyz, false);
    m_distances.assign(m_dim_xyz, -1);

    std::queue<std::tuple<int, int, int>> q;
    q.push(std::make_tuple(x, y, z));

    int n = getNode(x, y, z);
    m_distances[n] = 0;

    while (!q.empty()) {
        std::tuple<int, int, int> ncoords = q.front();
        q.pop();

        // extract the coordinates of this cell
        int nx = std::get<0>(ncoords);
        int ny = std::get<1>(ncoords);
        int nz = std::get<2>(ncoords);

        // extract the index of this cell
        n = getNode(nx, ny, nz);

        // mark as visited
        m_closed[n] = true;

        int dist = m_distances[n];

        // goal == found a free cell
        if (!isWall(n)) {
            int cell_dist = getDistance(nx, ny, nz);
            if (cell_dist < 0) {
                // TODO: mark as a wall, and move on
                setWall(nx, ny, nz);
                SMPL_INFO("Encountered isolated cell, m_running: %s", m_running ? "true" : "false");
            }
            else {
                return dist + cell_dist;
            }
        }


#define ADD_NEIGHBOR(xn, yn, zn) \
{\
if (inBounds(xn, yn, zn)) {\
    int nn = getNode(xn, yn, zn);\
    if (!m_closed[nn] && (m_distances[nn] == -1 || dist + 1 < m_distances[nn])) {\
        m_distances[nn] = dist + 1;\
        q.push(std::make_tuple(xn, yn, zn));\
    }\
}\
}

        ADD_NEIGHBOR(nx - 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz    );
        ADD_NEIGHBOR(nx - 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny + 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz    );
        ADD_NEIGHBOR(nx    , ny - 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny,     nz - 1);
//            ADD_NEIGHBOR(nx    , ny,     nz    );
        ADD_NEIGHBOR(nx    , ny,     nz + 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz    );
        ADD_NEIGHBOR(nx    , ny + 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz    );
        ADD_NEIGHBOR(nx + 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny + 1, nz + 1);
#undef ADD_NEIGHBOR
    }

    fprintf(stderr, "Found no free neighbor\n");
    return -1;
}
*/

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
