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
#include <smpl/heuristic/bfs_3d_base_heuristic.h>
#include <smpl/angles.h>

#define PI 3.14

namespace smpl {

static const char* LOG = "heuristic.bfs_3d_base";

Bfs3DBaseHeuristic::~Bfs3DBaseHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool Bfs3DBaseHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid, int thetac)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    if (grid == NULL) {
        return false;
    }

    m_grid = grid;
    m_thetac = thetac;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp != NULL) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    syncGridAndBfs();

    return true;
}

void Bfs3DBaseHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void Bfs3DBaseHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void Bfs3DBaseHeuristic::updateGoal(const GoalConstraint& goal)
{
    m_goal = goal;
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

        double delta = 0;
        double increment = 0.005;
        double arm_increment = 0.05;

        bool found_base = false;

        double theta_opt = PI/2 + ya;
        double theta = theta_opt;
        m_heuristic_base_poses.clear();
        for (int i=0; i<500; i++) {
            delta += increment;
            double arm_length = 0.45;
        for(int j=0 ;j<3; j++){
            arm_length += arm_increment;
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
        if(found_base)
            break;
        }
        if(!found_base){
        double arm_length = 0.45;
        for(int j=3 ;j<10; j++){
            arm_length += arm_increment;
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
        if(found_base)
            break;
        }
        }

        if(!m_heuristic_base_poses.size()){
            ROS_ERROR("No base position found");
            throw "No base position found.";
        }
        int gx, gy, gz;
        grid()->worldToGrid(
                m_heuristic_base_poses[0][0],
                m_heuristic_base_poses[0][1],
                0,
                gx, gy, gz);

        int gtheta = normalize_angle_positive(m_heuristic_base_poses[0][2]) / 6.28 * m_thetac;

        if (!m_bfs_3d_base->inBounds(gx, gy, gtheta)) {
            SMPL_ERROR_NAMED(LOG, "Base goal is out of BFS bounds");
            break;
        }

        SMPL_DEBUG_NAMED(LOG, "BFS3DBase: %d, %d, %d", gx, gy, gtheta);
        m_bfs_3d_base->run(gx, gy, gtheta);
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

double Bfs3DBaseHeuristic::getMetricStartDistance(double x, double y, double z)
{
    SMPL_ERROR("Unsupported function in BFS Heuristic");
    return 0.0;
}

double Bfs3DBaseHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    SMPL_ERROR("Unsupported function in BFS Heuristic");
    return 0.0;
}

Extension* Bfs3DBaseHeuristic::getExtension(size_t class_code)
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
int Bfs3DBaseHeuristic::GetGoalHeuristic(int state_id)
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
        int gtheta = normalize_angle_positive(robot_state[2]) / 6.28 * m_thetac;
        heuristic = m_cost_per_cell*getBfsCostToGoal(robot_grid[0], robot_grid[1], gtheta);
        //auto path = m_bfs_3d_base->getPath(robot_grid[0], robot_grid[1], gtheta);
        SMPL_DEBUG_NAMED(LOG, "x= %d, y=%d, theta= %d : %d", robot_grid[0], robot_grid[1], gtheta, heuristic);

    } else {
        heuristic = 0;
    }

    return heuristic;
}

int Bfs3DBaseHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("Bfs3DBaseHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int Bfs3DBaseHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("Bfs3DBaseHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

void Bfs3DBaseHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    SMPL_DEBUG_NAMED(LOG, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs_3d_base.reset(new BFS_3D_Base(xc, yc, m_thetac));
    int cell_count = xc*yc*m_thetac;
    int wall_count = 0;
    int projection_cell_thresh = m_2d_projection_thresh/grid()->resolution();

    for (int x = 0; x < xc; ++x) {
    for (int y = 0; y < yc; ++y) {
        //double min_dist = std::numeric_limits<double>::max();
        //for (int z = 0; z < zc; ++z) {
        //    if( z < projection_cell_thresh ){
        //        //Project occupancy grid up till a height of projection_thresh.
        //        min_dist = std::min(min_dist,  grid()->getDistance(x, y, z) );
        //    }
        //}
        double min_dist = grid()->getDistance(x, y, 10);
        if(min_dist <= m_inflation_radius)
            for(int th=0; th<m_thetac; ++th){
                m_bfs_3d_base->setWall(x, y, th);
                ++wall_count;
            }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int Bfs3DBaseHeuristic::getBfsCostToGoal(int x, int y, int theta) const {
    if (!m_bfs_3d_base->inBounds(x, y, theta)) {
        SMPL_DEBUG_NAMED(LOG, "Bfs3DHeuristic: Out of bounds.");
        return Infinity;
    }
    else if (m_bfs_3d_base->getDistance(x, y, theta) == BFS_3D_Base::WALL) {
        SMPL_DEBUG_NAMED(LOG, "Bfs3DHeuristic: Wall.");
        return Infinity;
    }
    else {
        return m_bfs_3d_base->getDistance(x, y, theta);
    }
}

std::vector< std::array<double, 3> > Bfs3DBaseHeuristic::getPath(int state_id){
    auto robot_state = (dynamic_cast<ManipLattice*>(planningSpace()))->extractState(state_id);
    int robot_grid[3];
    grid()->worldToGrid( robot_state[0], robot_state[1], 0,
            robot_grid[0], robot_grid[1], robot_grid[2] );
    int gtheta = normalize_angle_positive(robot_state[2]) / 6.28 * m_thetac;

    auto grid_path = m_bfs_3d_base->getPath( robot_grid[0], robot_grid[1], gtheta );
    std::vector< std::array<double, 3> > path;
    for(const auto& grid_state : grid_path ){
        std::array<double, 3> state;
        grid()->gridToWorld(grid_state[0], grid_state[1], 0, state[0], state[1], state[2]);
        state[2] = grid_state[2]*6.28/m_thetac;
        path.push_back(state);
    }
    return path;
}

auto Bfs3DBaseHeuristic::getValuesVisualization() -> visual::Marker {
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

            double cost_pct = (double)(m_cost_per_cell*getBfsCostToGoal(i, j, 1)) / (double)max_cost;

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
            "bfs3d_base_values");
}

} // namespace smpl
