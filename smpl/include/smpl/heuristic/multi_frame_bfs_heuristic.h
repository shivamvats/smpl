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

#ifndef SMPL_MULTI_FRAME_BFS_HEURISTIC_H
#define SMPL_MULTI_FRAME_BFS_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/occupancy_grid.h>
#include <smpl/debug/marker.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/bfs/bfs3d.h>

namespace smpl {

class MultiFrameBfsHeuristic : public RobotHeuristic
{
public:

    virtual ~MultiFrameBfsHeuristic();

    bool init(RobotPlanningSpace* space, const OccupancyGrid* grid);

    void setOffset(double x, double y, double z);

    double inflationRadius() const { return m_inflation_radius; }
    void setInflationRadius(double radius);
    int costPerCell() const { return m_cost_per_cell; }
    void setCostPerCell(int cost);

    auto grid() const -> const OccupancyGrid* { return m_grid; }

    auto getWallsVisualization() const -> visual::Marker;
    auto getValuesVisualization() const -> visual::Marker;

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    void updateGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

private:

    const OccupancyGrid* m_grid = nullptr;

    PointProjectionExtension* m_pp = nullptr;
    ExtractRobotStateExtension* m_ers = nullptr;
    ForwardKinematicsInterface* m_fk_iface = nullptr;

    std::unique_ptr<BFS_3D> m_bfs;
    std::unique_ptr<BFS_3D> m_ee_bfs;

    double m_pos_offset[3];

    double m_inflation_radius = 0.0;
    int m_cost_per_cell = 1;

    int getGoalHeuristic(int state_id, bool use_ee) const;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;

    inline
    int combine_costs(int c1, int c2) const;
};

} // namespace smpl

#endif
