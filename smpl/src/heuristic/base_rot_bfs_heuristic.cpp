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

#include <smpl/heuristic/base_rot_bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/angles.h>

namespace smpl {

static const char* LOG = "heuristic.base_rot_bfs";

BaseRotBfsHeuristic::~BaseRotBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BaseRotBfsHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid, double rot_angle)
{
    if (!BfsHeuristic::init(space, grid)) {
        return false;
    }
    m_rot_angle = rot_angle;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp != NULL) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_extract_ext = space->getExtension<ExtractRobotStateExtension>();
    if (m_extract_ext!= NULL) {
        SMPL_INFO_NAMED(LOG, "Got Extract Robot Extension!");
    } else
        return false;

    return true;
}


Extension* BaseRotBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<BaseRotBfsHeuristic>()) {
        return this;
    }
    return nullptr;
}

int BaseRotBfsHeuristic::GetGoalHeuristic(int state_id)
{
    if (m_pp == NULL) {
        return 0;
    }

    RobotState robot_state = m_extract_ext->extractState(state_id);
    double new_orientation = normalize_angle(robot_state[2] + m_rot_angle);
    robot_state[2] = new_orientation;

    Vector3 p;
    if (!m_pp->projectToPoint(robot_state, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    int heuristic = getBfsCostToGoal(dp.x(), dp.y(), dp.z());
    SMPL_DEBUG_NAMED(LOG, "BFS Heuristic: h(%f, %f, %f) = %d", p.x(), p.y(), p.z(), heuristic);
    return heuristic;
}

} // namespace smpl
