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

#include <smpl/heuristic/bfs_heuristic_rot.h>

// project includes
#include <smpl/bfs/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/angles.h>

namespace smpl {

static const char* LOG = "heuristic.bfs_rot";

BfsHeuristicRot::~BfsHeuristicRot()
{
    // empty to allow forward declaration of BFS_3D
}

bool BfsHeuristicRot::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{
    if(!BfsHeuristic::init(space, grid))
        return false;
    m_extract_ext = space->getExtension<ExtractRobotStateExtension>();
    if (m_extract_ext)
        SMPL_INFO("Got Exract State Extension");
    else {
        SMPL_ERROR("Extract Extension Null.");
        return false;
    }
    return true;
}

Extension* BfsHeuristicRot::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int BfsHeuristicRot::GetGoalHeuristic(int state_id)
{
    if(state_id == planningSpace()->getGoalStateID())
        return 0;

    if (m_pp == NULL) {
        return 0;
    }

    Vector3 p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    RobotState robot_state = m_extract_ext->extractState(state_id);

    double target_yaw = atan2(p.y() - robot_state[1], p.x() - robot_state[0]);
    double yaw_dist = shortest_angle_dist(target_yaw, robot_state[2]);

    int end_eff_heuristic = getBfsCostToGoal(dp.x(), dp.y(), dp.z());
    int heuristic = end_eff_heuristic + 0.2*getCostPerCell()*yaw_dist;
    SMPL_DEBUG_NAMED(LOG, "BfsRot Heuristic: h(%f, %f, %f) = %d + %d", p.x(), p.y(), p.z(), end_eff_heuristic, 0.1*getCostPerCell()*yaw_dist );
    return heuristic;
}

} // namespace smpl
