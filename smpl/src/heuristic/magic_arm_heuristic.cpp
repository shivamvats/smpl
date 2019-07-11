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

#include <smpl/heuristic/magic_arm_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>

#include <math.h>

namespace smpl {

static const char* LOG = "heuristic.magic_arm";

MagicArmHeuristic::~MagicArmHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MagicArmHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
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

void MagicArmHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void MagicArmHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void MagicArmHeuristic::updateGoal(const GoalConstraint& goal)
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

double MagicArmHeuristic::getMetricStartDistance(double x, double y, double z)
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

double MagicArmHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

Extension* MagicArmHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int MagicArmHeuristic::GetGoalHeuristic(int state_id)
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

    //if(heuristic/m_cost_per_cell > 3){
    if(p.z() < 0.7){
        auto extract_ext = planningSpace()->getExtension<ExtractRobotStateExtension>();
        RobotState robot_state = extract_ext->extractState(state_id);
        double magic_j1= -0.67;//-1.0;
        double magic_j2= -0.625;//-1.0;
        double magic_j3= 0.639;//-1.0;
        double magic_j4= -1;//-1.6;
        double delta_right_j1 = fabs(robot_state[3] - magic_j1);
        double delta_right_j2 = fabs(robot_state[4] - magic_j2);
        double delta_right_j3 = fabs(robot_state[5] - magic_j3);
        double delta_right_j4 = fabs(robot_state[6] - magic_j4);
        SMPL_ERROR("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                robot_state[0], robot_state[0], robot_state[2], robot_state[3], robot_state[4],
                robot_state[5], robot_state[6], robot_state[7], robot_state[8], robot_state[9]);
        //SMPL_ERROR("j4 delta: %f", m_cost_per_cell*delta_right_j4);
        heuristic = 10*m_cost_per_cell*(delta_right_j1 + delta_right_j2 + delta_right_j3 + delta_right_j4);
    }
    else{
    heuristic = getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
    }
    SMPL_DEBUG_NAMED(LOG, "BFS Heuristic: h(%f, %f, %f) = %d", p.x(), p.y(), p.z(), heuristic);
    return heuristic;
}

int MagicArmHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("MagicArmHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MagicArmHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("MagicArmHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

void MagicArmHeuristic::syncGridAndBfs()
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

int MagicArmHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
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
