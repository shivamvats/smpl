////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef SMPL_WORKSPACE_LATTICE_H
#define SMPL_WORKSPACE_LATTICE_H

// standard includes
#include <chrono>
#include <vector>

// system includes
#include <sbpl/headers.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/robot_model.h>
#include <smpl/time.h>
#include <smpl/types.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/workspace_lattice_base.h>
#include <smpl/graph/workspace_lattice_types.h>

namespace smpl {

struct WorkspaceLatticeActionSpace;

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
struct WorkspaceLattice :
    public WorkspaceLatticeBase,
    public PoseProjectionExtension,
    public ExtractRobotStateExtension
{
    WorkspaceLatticeState* m_goal_entry = NULL;
    int m_goal_state_id = -1;

    WorkspaceLatticeState* m_start_entry = NULL;
    int m_start_state_id = -1;

    // maps state -> id
    using StateKey = WorkspaceLatticeState;
    using StateHash = PointerValueHash<StateKey>;
    using StateEqual = PointerValueEqual<StateKey>;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps id -> state
    std::vector<WorkspaceLatticeState*> m_states;

    clock::time_point m_t_start;
    mutable bool m_near_goal = false; // mutable for assignment in isGoal

    WorkspaceLatticeActionSpace* m_actions = NULL;

    std::string m_viz_frame_id;

    ~WorkspaceLattice();

    void setVisualizationFrameId(const std::string& frame_id);
    auto visualizationFrameId() const -> const std::string&;

    /// \name Reimplemented Public Functions from WorkspaceLatticeBase
    ///@{
    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const Params& params,
        WorkspaceLatticeActionSpace* actions);
    ///@}

    /// \name Required Functions from PoseProjectionExtension
    ///@{
    bool projectToPose(int state_id, Affine3& pose) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    int getStartStateID() const override;
    int getGoalStateID() const override;

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    auto extractState(int state_id) -> const RobotState& override;

    /// \name Required Public Functions from Extension
    ///@{
    auto getExtension(size_t class_code) -> Extension* override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    void PrintState(
        int state_id,
        bool verbose,
        FILE* fout = nullptr) override;
    ///@}

    /// \name Reimplemented Functions from DiscreteSpaceInformation
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;
    int GetTrueCost(int parent_id, int child_id) override;
    ///@}

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalJointState(const GoalConstraint& goal);
    bool setUserGoal(const GoalConstraint& goal);

    int reserveHashEntry();
    int createState(const WorkspaceCoord& coord);
    auto getState(int state_id) const -> WorkspaceLatticeState*;
    int getOrCreateState(const WorkspaceCoord& coord, const RobotState& state);

    bool checkAction(
        const RobotState& state,
        const WorkspaceAction& action,
        RobotState* final_rstate = NULL);

    int computeCost(
        const WorkspaceLatticeState& src,
        const WorkspaceLatticeState& dst);

    bool checkLazyAction(
        const RobotState& state,
        const WorkspaceAction& action,
        RobotState* final_rstate = nullptr);

    bool isGoal(const WorkspaceState& state, const RobotState& robot_state) const;

    auto getStateVisualization(const RobotState& state, const std::string& ns)
        -> std::vector<visual::Marker>;
};

} // namespace smpl

#endif

