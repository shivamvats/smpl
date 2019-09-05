#include <cmath>
#include <limits>
#include <algorithm>

// project includes
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/bfs_2d_heuristic.h>
#include <smpl/angles.h>

#define PI 3.14

namespace smpl {

static const char* LOG = "heuristic.bfs_2d";

Bfs2DHeuristic::~Bfs2DHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool Bfs2DHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    if (grid == NULL) {
        return false;
    }

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp != NULL) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    syncGridAndBfs();

    return true;
}

void Bfs2DHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void Bfs2DHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void Bfs2DHeuristic::updateGoal(const GoalConstraint& goal)
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

        // When planning for the right hand, walker's base yaw must be at 90
        // degrees at the goal. Hence, sample base positions around that region.
        auto goal_pose = goal.pose;
        double goal_x = goal_pose.translation()[0];
        double goal_y = goal_pose.translation()[1];
        auto rot = goal_pose.rotation();

        double r, p, ya;
        smpl::angles::get_euler_zyx(rot, ya, p, r);

        double base_x=0, base_y=0;
        double arm_length = 0.55;

        double delta = 0;
        double increment = 0.005;

        bool found_base = false;

        double theta_opt = PI/2 + ya;
        double theta = theta_opt;
        for (int i=0; i<1500; i++) {
            delta += increment;
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
            // Explore symmetrically about the optimal theta.
            if(i%2)
                theta = theta_opt + delta;
            else
                theta = theta_opt - delta;
        }
        if(!m_heuristic_base_poses.size())
            throw "No base position found.";
        int gx, gy, gz;
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
        SMPL_ERROR("Unsupported goal type in BFS Heuristic");
        break;
    }
    case GoalType::USER_GOAL_CONSTRAINT_FN:
    default:
        SMPL_ERROR("Unsupported goal type in BFS Heuristic");
        break;
    }
}

double Bfs2DHeuristic::getMetricStartDistance(double x, double y, double z)
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

double Bfs2DHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    SMPL_ERROR("Unsupported goal type in BFS Heuristic");
    return 0.0;
}

Extension* Bfs2DHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

// 1. Find a reasonable base position near the goal
// 2. Compute 2D distance between robot state's base and target base.
// 3. Yaw dist bw base yaw and target base yaw.
//(NOT YET)// 4. Add 3D BFS distance between end-effector and goal xyz.
// 5. (Optional) Add Euclidean distance for rpy.
int Bfs2DHeuristic::GetGoalHeuristic(int state_id)
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
    int heuristic = 0;
    if( state_id != 0 ) {
        auto robot_state = (dynamic_cast<ManipLattice*>(planningSpace()))->extractState(state_id);
        int robot_grid[3];
        grid()->worldToGrid( robot_state[0], robot_state[1], 0,
                robot_grid[0], robot_grid[1], robot_grid[2] );
        heuristic = m_cost_per_cell*getBfsCostToGoal(robot_grid[0], robot_grid[1]);

        //if(dist < 0.5)
        //    return 0;
        //else
        //    base_dist = dist_xyz_target;

    } else {
        heuristic = 0;
    }

    return heuristic;
}

int Bfs2DHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("Bfs2DHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int Bfs2DHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("Bfs2DHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto Bfs2DHeuristic::getValuesVisualization() -> visual::Marker {
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

void Bfs2DHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    SMPL_DEBUG_NAMED(LOG, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs_2d.reset(new BFS_2D(xc, yc));
    int cell_count = xc*yc;
    int wall_count = 0;
    int projection_cell_thresh = m_2d_projection_thresh/grid()->resolution();

    for (int x = 0; x < xc; ++x) {
    for (int y = 0; y < yc; ++y) {
        double min_dist = std::numeric_limits<double>::max();
        for (int z = 0; z < zc; ++z) {
            if( z < projection_cell_thresh ){
                //Project occupancy grid up till a height of projection_thresh.
                min_dist = std::min(min_dist,  grid()->getDistance(x, y, z) );
            }
        }
        if(min_dist <= m_inflation_radius)
            m_bfs_2d->setWall(x, y);
            ++wall_count;
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

auto Bfs2DHeuristic::getWallsVisualization() const -> visual::Marker {

    std::vector<Vector3> voxels;
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
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

int Bfs2DHeuristic::getBfsCostToGoal(int x, int y) const {
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
