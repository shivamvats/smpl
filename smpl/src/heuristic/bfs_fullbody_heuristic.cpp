////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <cmath>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/angles.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl {

static const char* LOG = "heuristic.bfs_fullbody";

BfsFullbodyHeuristic::~BfsFullbodyHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BfsFullbodyHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
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

void BfsFullbodyHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void BfsFullbodyHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void BfsFullbodyHeuristic::updateGoal(const GoalConstraint& goal)
{
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

        if (!m_bfs->inBounds(gx, gy, gz)) {
            SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
            break;
        }

        m_goal_cells.emplace_back(gx, gy, gz);

        m_bfs->run(gx, gy, gz);

        auto goal_pose = goal.pose;
        double goal_x = goal_pose.translation()[0];
        double goal_y = goal_pose.translation()[1];
        double base_x=0, base_y=0;
        double arm_length = 0.55;
        double robot_yaw_delta = 60 * 3.14/180;

        bool found_base = false;

        for (int i=0; i<7; i++) {
            double theta = i*0.2;
            double possible_x = goal_x + arm_length*cos(theta);
            double possible_y = goal_y + arm_length*sin(theta);
            double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x) + robot_yaw_delta;
            RobotState possible_state(planningSpace()->robot()->jointCount(), 0);
            possible_state[0] = possible_x;
            possible_state[1] = possible_y;
            possible_state[2] = possible_yaw;

            std::vector<double> heuristic_base_pose;
            if (planningSpace()->collisionChecker()->isStateValid(possible_state)) {
                heuristic_base_pose.push_back(possible_x);
                heuristic_base_pose.push_back(possible_y);
                heuristic_base_pose.push_back(possible_yaw);
                m_heuristic_base_poses.push_back(heuristic_base_pose);
                found_base = true;
            }
        }
        if (!found_base) {
            ROS_ERROR("Could not find a valid base state.");
        }
        else {
            ROS_ERROR( "%d Valid base pose found.", m_heuristic_base_poses.size() );
        }
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

            if (!m_bfs->inBounds(gx, gy, gz)) {
                SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
                continue;
            }

            cell_coords.push_back(gx);
            cell_coords.push_back(gy);
            cell_coords.push_back(gz);

            m_goal_cells.emplace_back(gx, gy, gz);
        }
        m_bfs->run(begin(cell_coords), end(cell_coords));
        break;
    }
    case GoalType::USER_GOAL_CONSTRAINT_FN:
    default:
        SMPL_ERROR("Unsupported goal type in BFS Heuristic");
        break;
    }
}

double BfsFullbodyHeuristic::getMetricStartDistance(double x, double y, double z)
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

double BfsFullbodyHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

Extension* BfsFullbodyHeuristic::getExtension(size_t class_code)
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
int BfsFullbodyHeuristic::GetGoalHeuristic(int state_id)
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
    double base_dist = 0;
    if( state_id != 0 ) {
        auto robot_state = (dynamic_cast<ManipLattice*>(planningSpace()))->extractState(state_id);
        auto target_base = m_heuristic_base_poses[0];

        std::vector<double> p_xyyaw{ robot_state[0], robot_state[1], robot_state[2] };

        double angle_p_target = atan2(target_base[1] - p_xyyaw[1], target_base[0] - p_xyyaw[0]);
        //Rotate towards target
        double dist_rot1 = fabs(shortest_angle_dist(p_xyyaw[2], angle_p_target));

        // Move to target xyz
        auto dist_xyz_target = sqrt( (p_xyyaw[0] - target_base[0])*(p_xyyaw[0] - target_base[0]) + p_xyyaw[1] - target_base[1]*(p_xyyaw[1] - target_base[1]) );

        // Achieve target yaw
        double dist_rot2 = fabs(shortest_angle_dist(angle_p_target, target_base[3]));

        //base_dist = std::sqrt( (p_xyz[0] - target_base[0])*(p_xyz[0] - target_base[0]) + (p_xyz[1] - target_base[1])*(p_xyz[1] - target_base[1]));// + fabs(shortest_angle_dist( target_base[2], Y));
        SMPL_DEBUG_NAMED(LOG, "rot1: %f, dist: %f, rot2: %f", dist_rot1, dist_xyz_target, dist_rot2);
        base_dist = dist_rot1 + dist_xyz_target + dist_rot2;
    } else {
        base_dist = 0;
    }

    int heuristic = m_cost_per_cell*base_dist + getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
    //SMPL_DEBUG_NAMED(LOG,"base: %f, bfs: %d", m_cost_per_cell*base_dist, getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z() ));
    SMPL_DEBUG_NAMED(LOG, "cost_per_cell: %d, base_dist: %f", m_cost_per_cell, base_dist);
    //int heuristic = m_cost_per_cell*base_dist;
    SMPL_DEBUG_NAMED(LOG, "Heuristic value: %d", heuristic);
    return heuristic;
}

int BfsFullbodyHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("BfsFullbodyHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int BfsFullbodyHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("BfsFullbodyHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto BfsFullbodyHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Vector3> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_bfs->isWall(x, y, z)) {
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

auto BfsFullbodyHeuristic::getValuesVisualization() -> visual::Marker
{
    bool all_invalid = true;
    for (auto& cell : m_goal_cells) {
        if (!m_bfs->isWall(cell.x, cell.y, cell.z)) {
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
        if (!m_bfs->isWall(cell.x, cell.y, cell.z)) {
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

        const int d = m_cost_per_cell * m_bfs->getDistance(c.x, c.y, c.z);

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
            if (!m_bfs->inBounds(sx, sy, sz) || m_bfs->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_bfs->getDistance(sx, sy, sz);
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

void BfsFullbodyHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
//    SMPL_DEBUG_NAMED(LOG, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int x = 0; x < xc; ++x) {
    for (int y = 0; y < yc; ++y) {
    for (int z = 0; z < zc; ++z) {
        const double radius = m_inflation_radius;
        if (grid()->getDistance(x, y, z) <= radius) {
            m_bfs->setWall(x, y, z);
            ++wall_count;
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int BfsFullbodyHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

} // namespace smpl
