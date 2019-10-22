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

#include <smpl/heuristic/multi_frame_bfs_heuristic.h>

// project includes
#include <smpl/bfs/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace smpl {

static const char* LOG = "heuristic.mfbfs";

MultiFrameBfsHeuristic::~MultiFrameBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiFrameBfsHeuristic::init(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid)
{
    if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_pos_offset[0] = m_pos_offset[1] = m_pos_offset[2] = 0.0;

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_ers = space->getExtension<ExtractRobotStateExtension>();
    if (m_ers) {
        SMPL_INFO_NAMED(LOG, "Got Extract Robot State Extension!");
    }
    m_fk_iface = space->robot()->getExtension<ForwardKinematicsInterface>();
    if (m_fk_iface) {
        SMPL_INFO_NAMED(LOG, "Got Forward Kinematics Interface!");
    }
    syncGridAndBfs();

    return true;
}

void MultiFrameBfsHeuristic::setOffset(double x, double y, double z)
{
    m_pos_offset[0] = x;
    m_pos_offset[1] = y;
    m_pos_offset[2] = z;
}

void MultiFrameBfsHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void MultiFrameBfsHeuristic::setCostPerCell(int cost)
{
    m_cost_per_cell = cost;
}

Extension* MultiFrameBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

void MultiFrameBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    SMPL_DEBUG_NAMED(LOG, "Update goal");

    Affine3 offset_pose =
            goal.pose *
            Translation3(m_pos_offset[0], m_pos_offset[1], m_pos_offset[2]);

    int ogx, ogy, ogz;
    grid()->worldToGrid(
            offset_pose.translation()[0],
            offset_pose.translation()[1],
            offset_pose.translation()[2],
            ogx, ogy, ogz);

    int plgx, plgy, plgz;
    grid()->worldToGrid(
            goal.pose.translation()[0],
            goal.pose.translation()[1],
            goal.pose.translation()[2],
            plgx, plgy, plgz);

    SMPL_DEBUG_NAMED(LOG, "Setting the Two-Point BFS heuristic goals (%d, %d, %d), (%d, %d, %d)", ogx, ogy, ogz, plgx, plgy, plgz);

    if (!m_bfs->inBounds(ogx, ogy, ogz) ||
        !m_ee_bfs->inBounds(plgx, plgy, plgz))
    {
        SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
        return;
    }

    m_bfs->run(ogx, ogy, ogz);
    m_ee_bfs->run(plgx, plgy, plgz);
}

double MultiFrameBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: shamefully copied from BfsHeuristic
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

double MultiFrameBfsHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

int MultiFrameBfsHeuristic::GetGoalHeuristic(int state_id)
{
    return getGoalHeuristic(state_id, true);
}

int MultiFrameBfsHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("MultiFrameBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiFrameBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    } else {
        SMPL_WARN_ONCE("MultiFrameBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto MultiFrameBfsHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Vector3> points;
    const int dimX = grid()->numCellsX();
    const int dimY = grid()->numCellsY();
    const int dimZ = grid()->numCellsZ();
    for (int z = 0; z < dimZ; z++) {
    for (int y = 0; y < dimY; y++) {
    for (int x = 0; x < dimX; x++) {
        if (m_bfs->isWall(x, y, z)) {
            Vector3 p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            points.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", points.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    return visual::MakeCubesMarker(
            points,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");
}

auto MultiFrameBfsHeuristic::getValuesVisualization() const -> visual::Marker
{
    // factor in the ee bfs values? This doesn't seem to make a whole lot of
    // sense since the color would be derived from colocated cell values
    const bool factor_ee = false;

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = getGoalHeuristic(planningSpace()->getStartStateID(), factor_ee);

    const int edge_cost = m_cost_per_cell;

    int max_cost = (int)(1.1 * start_heur);

    // ...and this will also flush the bfs...

    std::vector<Vector3> points;
    std::vector<visual::Color> colors;
    for (int z = 0; z < grid()->numCellsZ(); ++z) {
    for (int y = 0; y < grid()->numCellsY(); ++y) {
    for (int x = 0; x < grid()->numCellsX(); ++x) {
        // skip cells without valid distances from the start
        if (m_bfs->isWall(x, y, z) || m_bfs->isUndiscovered(x, y, z)) {
            continue;
        }

        int d = edge_cost * m_bfs->getDistance(x, y, z);
        int eed = factor_ee ? edge_cost * m_ee_bfs->getDistance(x, y, z) : 0;
        double cost_pct = (double)combine_costs(d, eed) / (double)(max_cost);

        if (cost_pct > 1.0) {
            continue;
        }

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

        Vector3 p;
        grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        colors.push_back(color);
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

int MultiFrameBfsHeuristic::getGoalHeuristic(int state_id, bool use_ee) const
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    int h_planning_frame = 0;
    if (m_pp) {
        Vector3 p;
        if (m_pp->projectToPoint(state_id, p)) {
            Eigen::Vector3i dp;
            grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
            h_planning_frame = getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
        }
    }

    int h_planning_link = 0;
    if (use_ee && m_ers && m_fk_iface) {
        const RobotState& state = m_ers->extractState(state_id);
        auto pose = m_fk_iface->computeFK(state);
        Eigen::Vector3i eex;
        grid()->worldToGrid(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2],
                eex[0], eex[1], eex[2]);
        h_planning_link = getBfsCostToGoal(*m_ee_bfs, eex[0], eex[1], eex[2]);
    }

    return combine_costs(h_planning_frame, h_planning_link);
}

void MultiFrameBfsHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    m_ee_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
    for (int y = 0; y < yc; ++y) {
    for (int x = 0; x < xc; ++x) {
        const double radius = m_inflation_radius;
        if (grid()->getDistance(x, y, z) <= radius) {
            m_bfs->setWall(x, y, z);
            m_ee_bfs->setWall(x, y, z);
            ++wall_count;
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int MultiFrameBfsHeuristic::getBfsCostToGoal(
    const BFS_3D& bfs, int x, int y, int z) const
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

int MultiFrameBfsHeuristic::combine_costs(int c1, int c2) const
{
    return c1 + c2;
}

} // namespace smpl
