#include <smpl/bfs/bfs3d_base.h>

#include <smpl/console/console.h>

namespace smpl {

BFS_3D_Base::BFS_3D_Base(int width, int height, int length) :
    m_search_thread(),
    m_dim_x(),
    m_dim_y(),
    m_dim_z(),
    m_distance_grid(nullptr),
    m_queue(nullptr),
    m_queue_head(),
    m_queue_tail(),
    m_running(false),
    m_neighbor_offsets(),
    m_closed(),
    m_distances()
{
    if (width <= 0 || height <= 0 || length <= 0) {
        return;
    }

    m_dim_x = width + 2;
    m_dim_y = height + 2;
    m_dim_z = length + 2;

    m_dim_xy = m_dim_x * m_dim_y;
    m_dim_xyz = m_dim_xy * m_dim_z;

    //m_neighbor_offsets[1] = 1;
    //m_neighbor_offsets[3] = -1;
    //m_neighbor_offsets[8] = m_dim_xy;
    //m_neighbor_offsets[17] = -m_dim_xy;

    m_distance_grid = new int[m_dim_xyz];
    m_queue = new int[width * height * length];

    for (int node = 0; node < m_dim_xyz; node++) {
        int x = node % m_dim_x;
        int y = node / m_dim_x % m_dim_y;
        int z = node / m_dim_xy;
        if (x == 0 || x == m_dim_x - 1 ||
            y == 0 || y == m_dim_y - 1 ||
            z == 0 || z == m_dim_z - 1)
        {
            m_distance_grid[node] = WALL;
        }
        else {
            m_distance_grid[node] = UNDISCOVERED;
        }
    }

    m_running = false;
}

BFS_3D_Base::~BFS_3D_Base()
{
    if (m_search_thread.joinable()) {
        m_search_thread.join();
    }

    if (m_distance_grid) {
        delete[] m_distance_grid;
    }
    if (m_queue) {
        delete[] m_queue;
    }
}

void BFS_3D_Base::getDimensions(int* width, int* height, int* length)
{
    *width = m_dim_x - 2;
    *height = m_dim_y - 2;
    *length = m_dim_z - 2;
}

void BFS_3D_Base::setWall(int x, int y, int z)
{
    if (m_running) {
        //error "Cannot modify grid while search is running"
        return;
    }

    int node = getNode(x, y, z);
    m_distance_grid[node] = WALL;
}

bool BFS_3D_Base::isWall(int x, int y, int z) const
{
    int node = getNode(x, y, z);
    return m_distance_grid[node] == WALL;
}

bool BFS_3D_Base::isUndiscovered(int x, int y, int z) const
{
    int node = getNode(x, y, z);
    while (m_running && m_distance_grid[node] < 0);
    return m_distance_grid[node] == UNDISCOVERED;
}

void BFS_3D_Base::run(int x, int y, int z)
{
    if (m_running) {
        return;
    }

    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // get index of start coordinate
    int origin = getNode(x, y, z);

    // initialize the queue
    m_queue_head = 0;
    m_queue_tail = 1;
    m_queue[0] = origin;

    // initialize starting distance
    m_distance_grid[origin] = 0;

    // fire off background thread to compute bfs
    m_search_thread = std::thread([&]()
    {
        this->search(m_dim_x, m_dim_xy, m_distance_grid, m_queue, m_queue_head, m_queue_tail);
    });

    m_running = true;
}

std::vector<BFS_3D_Base::BaseState> BFS_3D_Base::getPath(int x, int y, int z){
    std::vector<BaseState> path;
    int currentNode = getNode(x, y, z);
    int bestPred = currentNode;
    while(m_distance_grid[currentNode] != 0){
        int x = currentNode % m_dim_x - 1;
        int y = currentNode / m_dim_x % m_dim_y - 1;
        int z = currentNode / m_dim_xy - 1;
        // rotate
        auto updatedPred = [&](int node, int bestPred){
            if( (m_distance_grid[node] >= 0) && (m_distance_grid[node] < m_distance_grid[bestPred]) ){
                return node;
            }
            else
                return bestPred;
        };

        bestPred = updatedPred(getNode(x, y, z+1), bestPred);
        bestPred = updatedPred(getNode(x, y, z-1), bestPred);

        int quarter_z = (m_dim_z - 2)/4;
        if( z == 0 || z == (2*quarter_z) ){
            bestPred = updatedPred(getNode(x+1, y, z), bestPred);
            bestPred = updatedPred(getNode(x-1, y, z), bestPred);
        } else if( z == quarter_z || z == (3*quarter_z)){
            bestPred = updatedPred(getNode(x, y+1, z), bestPred);
            bestPred = updatedPred(getNode(x, y-1, z), bestPred);
        }

        currentNode = bestPred;

        x = currentNode % m_dim_x - 1;
        y = currentNode / m_dim_x % m_dim_y - 1;
        z = currentNode / m_dim_xy - 1;
        BaseState s = {x, y, z};
        path.push_back(s);
    }
    return path;
}

template <typename Visitor>
void BFS_3D_Base::visit_free_cells(int node, const Visitor& visitor)
{
    if (isWall(node)) {
        return;
    }

    int nx, ny, nz;
    getCoord(node, nx, ny, nz);

    std::vector<bool> visited(m_dim_xyz, false);
    std::queue<int> nodes;
    nodes.push(node);

    while (!nodes.empty()) {
        int n = nodes.front();
        nodes.pop();

        visitor(n);

        for (int i = 0; i < 26; ++i) {
            int nn = neighbor(n, i);
            if (!visited[nn] && !isWall(nn)) {
                nodes.push(nn);
                // mark visited here to avoid adding nodes to the queue multiple
                // times
                visited[nn] = true;
                if (nodes.size() >= m_dim_xyz) {
                    SMPL_ERROR("Wow queue is too damn big");
                    return;
                }
            }
        }
    }
}

int BFS_3D_Base::getNearestFreeNodeDist(int x, int y, int z)
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

int BFS_3D_Base::countWalls() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == WALL) {
            ++count;
        }
    }
    return count;
}

int BFS_3D_Base::countUndiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == UNDISCOVERED) {
            ++count;
        }
    }
    return count;
}

int BFS_3D_Base::countDiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] != WALL && m_distance_grid[i] >= 0) {
            ++count;
        }
    }
    return count;
}

void BFS_3D_Base::search(
    int width,
    int planeSize,
    int volatile* distance_grid,
    int* queue,
    int& queue_head,
    int& queue_tail)
{
    while (queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        int x = currentNode % m_dim_x - 1;
        int y = currentNode / m_dim_x % m_dim_y - 1;
        int z = currentNode / m_dim_xy - 1;
        // Rotate
        if (distance_grid[getNode(x, y, z+1)] < 0) {
            queue[queue_tail++] = getNode(x, y, z+1);
            distance_grid[getNode(x, y, z+1)] = currentCost;
        }
        if (distance_grid[getNode(x, y, z-1)] < 0) {
            queue[queue_tail++] = getNode(x, y, z-1);
            distance_grid[getNode(x, y, z-1)] = currentCost;
        }

        int quarter_z = (m_dim_z - 2)/4;
        //Along x
        if( z == 0 || z == (2*quarter_z) ){
            if (distance_grid[getNode(x + 1, y, z)] < 0) {
                queue[queue_tail++] = getNode(x + 1, y, z);
                distance_grid[getNode(x + 1, y, z)] = currentCost;
            }
            if (distance_grid[getNode(x - 1, y, z)] < 0) {
                queue[queue_tail++] = getNode(x - 1, y, z);
                distance_grid[getNode(x - 1, y, z)] = currentCost;
            }
        }
        if( z == quarter_z || z == (3*quarter_z)){
            if (distance_grid[getNode(x, y+1, z)] < 0) {
                queue[queue_tail++] = getNode(x, y+1, z);
                distance_grid[getNode(x, y+1, z)] = currentCost;
            }
            if (distance_grid[getNode(x, y-1, z)] < 0) {
                queue[queue_tail++] = getNode(x, y-1, z);
                distance_grid[getNode(x, y-1, z)] = currentCost;
            }
        }
    }
    m_running = false;
}

} // namespace smpl
