////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_MANIP_LATTICE_MULTI_REP_H
#define SMPL_MANIP_LATTICE_MULTI_REP_H

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/action_space.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl {

/// \class Discrete space constructed by expliciting discretizing each joint
class ManipLatticeMultiRep :
    public ManipLattice {
public:

    ~ManipLatticeMultiRep();

    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        std::vector<ActionSpace*>& action_spaces);

    auto actionSpaces() -> std::vector<ActionSpace*> { return m_action_spaces; }
    auto actionSpaces() const -> const std::vector<ActionSpace*> { return m_action_spaces; }

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    void GetSuccs(
        int state_id,
        int rep_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    ///@}

protected:

    int cost(
        ManipLatticeState* HashEntry1,
        ManipLatticeState* HashEntry2,
        bool bState2IsGoal) const;

    bool checkAction(const RobotState& state, const Action& action);

private:

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    std::vector<ActionSpace*> m_action_spaces;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;
    std::vector<bool> m_bounded;

    std::vector<int> m_coord_vals;
    std::vector<double> m_coord_deltas;

    int m_goal_state_id = -1;
    int m_start_state_id = -1;

    // maps from coords to stateID
    typedef ManipLatticeState StateKey;
    typedef PointerValueHash<StateKey> StateHash;
    typedef PointerValueEqual<StateKey> StateEqual;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    std::string m_viz_frame_id;

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalPoses(const GoalConstraint& goal);
    bool setGoalConfiguration(const GoalConstraint& goal);
    bool setUserGoal(const GoalConstraint& goal);

    void startNewSearch();

    /// \name planning
    ///@{
    ///@}
};

} // namespace smpl

#endif