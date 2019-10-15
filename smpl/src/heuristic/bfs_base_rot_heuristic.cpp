#include <cmath>
#include <limits>
#include <algorithm>
#include <smpl/heuristic/bfs_base_rot_heuristic.h>
#include <smpl/angles.h>

// project includes
#include <smpl/bfs/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/graph/manip_lattice.h>

#define PI 3.14

namespace smpl {

static const char* LOG = "heuristic.bfs_base_rot";

BfsBaseRotHeuristic::~BfsBaseRotHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BfsBaseRotHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid, double angle)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    if (grid == NULL) {
        return false;
    }

    m_grid = grid;
    m_yaw = angle;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp != NULL) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    syncGridAndBfs();

    return true;
}

void BfsBaseRotHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void BfsBaseRotHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void BfsBaseRotHeuristic::updateGoal(const GoalConstraint& goal)
{
    syncGridAndBfs();
    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
    case GoalType::JOINT_STATE_GOAL:
    {
        // TODO: This assumes goal.pose is initialized, regardless of what kind
        // of goal this is. For joint state goals, we should project the start
        // state to a goal position, since we can't reliably expect goal.pose
        // to be valid.
        int gx, gy, gz;
        grid()->worldToGrid(
                goal.pose.translation()[0],
                goal.pose.translation()[1],
                goal.pose.translation()[2],
                gx, gy, gz);

        SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

        if (!m_bfs_3d->inBounds(gx, gy, gz)) {
            SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
            break;
        }
        m_goal_cells.emplace_back(gx, gy, gz);

        m_bfs_3d->run(gx, gy, gz);

        auto goal_pose = goal.pose;
        double goal_x = goal_pose.translation()[0];
        double goal_y = goal_pose.translation()[1];
        auto rot = goal_pose.rotation();

        double r, p, ya;
        smpl::angles::get_euler_zyx(rot, ya, p, r);
        ROS_ERROR("Yaw: %f", ya);

        double base_x=0, base_y=0;
        double arm_length = 0.55;

        double delta = 0;
        double increment = 0.005;

        bool found_base = false;

        double theta_opt = PI/2 + ya;
        double theta = theta_opt;
        for (int i=0; i<1500; i++) {
            delta += increment;
            // Explore symmetrically about the optimal theta.
            if(i%2)
                theta = theta_opt + delta;
            else
                theta = theta_opt - delta;
            double possible_x = goal_x + arm_length*cos(theta);
            double possible_y = goal_y + arm_length*sin(theta);
            double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x);
            RobotState possible_state(planningSpace()->robot()->jointCount(), 0);
            //SV_SHOW_INFO_NAMED("heur", dynamic_cast<smpl::collision::CollisionSpace*>(planningSpace()->collisionChecker())->getCollisionRobotVisualization(possible_base));
            possible_state[0] = possible_x;
            possible_state[1] = possible_y;
            possible_state[2] = possible_yaw;

            if (planningSpace()->collisionChecker()->isStateValid(possible_state)) {
                m_heuristic_base_poses.push_back(possible_state);
                found_base = true;
                ROS_INFO("Success on Iteration: %d", i);
                ROS_INFO("Optimal yaw: %f, Found yaw: %f", theta_opt, theta);
                break;
            }
        }
        if(!found_base){
            ROS_ERROR("No base pose found.");
            throw "Could not find valid base pose at the goal.";
        }
        grid()->worldToGrid(
                m_heuristic_base_poses[0][0],
                m_heuristic_base_poses[0][1],
                0,
                gx, gy, gz);
        if (!m_bfs_2d->inBounds(gx, gy)) {
            SMPL_ERROR_NAMED(LOG, "Base goal is out of BFS bounds");
            break;
        }

        m_bfs_2d->run(gx, gy);
        break;
    }
    case GoalType::MULTIPLE_POSE_GOAL:
    {
        std::vector<int> cell_coords;
        for (auto& goal_pose : goal.poses) {
            int gx, gy, gz;
            grid()->worldToGrid(
                    goal.pose.translation()[0],
                    goal.pose.translation()[1],
                    goal.pose.translation()[2],
                    gx, gy, gz);

            SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

            if (!m_bfs_3d->inBounds(gx, gy, gz)) {
                SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
                continue;
            }

            cell_coords.push_back(gx);
            cell_coords.push_back(gy);
            cell_coords.push_back(gz);

            m_goal_cells.emplace_back(gx, gy, gz);
        }
        m_bfs_3d->run(begin(cell_coords), end(cell_coords));
        break;
    }
    case GoalType::USER_GOAL_CONSTRAINT_FN:
    default:
        SMPL_ERROR("Unsupported goal type in BFS Heuristic");
        break;
    }
}

double BfsBaseRotHeuristic::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Vector3 p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double BfsBaseRotHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs_3d->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs_3d->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

Extension* BfsBaseRotHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

// 1. Find a reasonable base position near the goal
// 2. Compute 2D distance between robot state's base and target base.
// 3. Add 3D BFS distance between end-effector and goal xyz.
// 4. (Optional) Add Euclidean distance for rpy.
int BfsBaseRotHeuristic::GetGoalHeuristic(int state_id)
{
    if (m_pp == NULL) {
        return 0;
    }

    Vector3 p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    int base_dist = 0;
    int yaw_dist = 0;
    if( state_id != 0 ) {
        auto robot_state = (dynamic_cast<ManipLattice*>(planningSpace()))->extractState(state_id);
        auto& goal_tr = planningSpace()->goal().pose.translation();

        int robot_grid[3];
        grid()->worldToGrid(robot_state[0], robot_state[1], 0, robot_grid[0], robot_grid[1], robot_grid[2]);
        auto dist_xyz_target = m_cost_per_cell*getBfsCostToGoal(robot_grid[0], robot_grid[1]);

        double dist = sqrt( (goal_tr[0] - p.x())*(goal_tr[0] - p.x()) + (goal_tr[1] - p.y())*(goal_tr[1] - p.y()) );
        if(dist < 0.5)
            return 0;
        else
            base_dist = dist_xyz_target;

        //yaw_dist = m_cost_per_cell*fabs(robot_state[2] - m_heuristic_base_poses[0][2]);
        yaw_dist = m_cost_per_cell*fabs(shortest_angle_dist(robot_state[2], m_yaw));
    } else {
        base_dist = 0;
    }

    //int arm_heur = 0.1*m_cost_per_cell*getBfsCostToGoal(*m_bfs_3d, dp.x(), dp.y(), dp.z());
    int heuristic = base_dist + 2*yaw_dist;// + arm_heur;
    SMPL_DEBUG_NAMED(LOG, "base_dist: %d, yaw_dist: %d", base_dist, yaw_dist);
    //int heuristic = m_cost_per_cell*base_dist;
    //SMPL_DEBUG_NAMED(LOG, "BfsBaseRotHeuristic value: %d ", heuristic );
    return heuristic;
}

int BfsBaseRotHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("BfsBaseRotHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int BfsBaseRotHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("BfsBaseRotHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto BfsBaseRotHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Vector3> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_bfs_3d->isWall(x, y, z)) {
            Vector3 p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    return visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");
}

auto BfsBaseRotHeuristic::getValuesVisualization() -> visual::Marker
{
    bool all_invalid = true;
    for (auto& cell : m_goal_cells) {
        if (!m_bfs_3d->isWall(cell.x, cell.y, cell.z)) {
            all_invalid = false;
            break;
        }
    }

    // no goal cells or all invalid => all invalid
    if (all_invalid) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    // arbitrary limit on size of visualization...64Mb worth of points+colors
    const size_t max_points =
            (64 * 1024 * 1024) /
            (sizeof(visual::Color) + sizeof(Vector3));

    std::vector<Vector3> points;
    std::vector<visual::Color> colors;

    struct CostCell
    {
        int x, y, z, g;
    };
    std::queue<CostCell> cells;
    Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
    for (auto& cell : m_goal_cells) {
        if (!m_bfs_3d->isWall(cell.x, cell.y, cell.z)) {
            visited(cell.x, cell.y, cell.z) = true;
            cells.push({ cell.x, cell.y, cell.z, 0 });
        }
    }
    while (!cells.empty()) {
        CostCell c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            double cost_pct = (double)c.g / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);
            color.a = 1.0f;

            Vector3 p;
            grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            points.push_back(p);

            colors.push_back(color);
        }

//        visited(c.x, c.y, c.z) = true;

        const int d = m_cost_per_cell * m_bfs_3d->getDistance(c.x, c.y, c.z);

        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            int sx = c.x + dx;
            int sy = c.y + dy;
            int sz = c.z + dz;

            // check if neighbor is valid
            if (!m_bfs_3d->inBounds(sx, sy, sz) || m_bfs_3d->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_bfs_3d->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }

    return visual::MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs_values");
}

visual::Marker BfsBaseRotHeuristic::get2DValuesVisualization(){
    std::vector<Vector3> voxels;

    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();

    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);
    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    std::vector<visual::Color> colors;

    for(int i=0 ; i<xc; i++)
        for(int j=0; j<yc; j++){
            Vector3 p;
            grid()->gridToWorld(i, j, 0, p.x(), p.y(), p.z());
            voxels.push_back(p);

            double cost_pct = (double)(m_cost_per_cell*getBfsCostToGoal(i, j)) / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };
            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);
            color.a = 1.0f;
            colors.push_back(color);
        }

    return visual::MakeCubesMarker(
            std::move(voxels),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs2d_values");
}

void BfsBaseRotHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    SMPL_DEBUG_NAMED(LOG, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs_3d.reset(new BFS_3D(xc, yc, zc));
    m_bfs_2d.reset(new BFS_2D(xc, yc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    int projection_cell_thresh = m_2d_projection_thresh/grid()->resolution();

    for (int x = 0; x < xc; ++x) {
    for (int y = 0; y < yc; ++y) {
        double min_dist = std::numeric_limits<double>::max();
        for (int z = 0; z < zc; ++z) {
            const double radius = m_inflation_radius;
            if (grid()->getDistance(x, y, z) <= radius) {
                m_bfs_3d->setWall(x, y, z);
                ++wall_count;
            }
            if( z < projection_cell_thresh ){
                //Project occupancy grid up till a height of projection_thresh.
                min_dist = std::min(min_dist,  grid()->getDistance(x, y, z) );
            }
        }
        if(min_dist <= m_inflation_radius)
            m_bfs_2d->setWall(x, y);
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

visual::Marker BfsBaseRotHeuristic::get2DMapVisualization(){
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    std::vector<Vector3> voxels;
    for(int i=0 ; i<xc; i++){
        for(int j=0; j<yc; j++)
            if(m_bfs_2d->isWall(i, j)){
                    Vector3 p;
                    grid()->gridToWorld(i, j, 0, p.x(), p.y(), p.z());
                    voxels.push_back(p);
            }
    }
    return visual::MakeCubesMarker(
        std::move(voxels),
        m_grid->resolution(),
        visual::Color{ 0.8f, 0.3f, 0.5f, 1.0f },
        grid()->getReferenceFrame(),
        "projected_2d_voxels");
}


int BfsBaseRotHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else {
        return bfs.getDistance(x, y, z);
    }
}

int BfsBaseRotHeuristic::getBfsCostToGoal(int x, int y) const {
    if (!m_bfs_2d->inBounds(x, y)) {
        return Infinity;
    }
    else if (m_bfs_2d->getDistance(x, y) == BFS_2D::WALL) {
        return Infinity;
    }
    else {
        return m_bfs_2d->getDistance(x, y);
    }
}

} // namespace smpl
