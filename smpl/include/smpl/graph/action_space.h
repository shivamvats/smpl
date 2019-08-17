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

#ifndef SMPL_ACTION_SPACE_H
#define SMPL_ACTION_SPACE_H

// project includes
#include <smpl/types.h>

namespace smpl {

using RepId = unsigned int;

class RobotPlanningSpace;
struct GoalConstraint;

class ActionSpace
{
public:

    virtual ~ActionSpace();

    virtual bool init(RobotPlanningSpace* space);

    auto planningSpace() -> RobotPlanningSpace* { return m_space; }
    auto planningSpace() const -> const RobotPlanningSpace* { return m_space; }

    /// \brief Return the set of actions available from a state.
    ///
    /// Each action consists of a sequence of waypoints from the source state
    /// describing the approximate motion the robot will take to reach a
    /// successor state. The sequence of waypoints need not contain the the
    /// source state. The motion between waypoints will be checked via the set
    /// CollisionChecker's isStateToStateValid function during a search.
    virtual bool apply(const RobotState& parent, std::vector<Action>& actions) = 0;

    virtual void updateStart(const RobotState& state) { }
    virtual void updateGoal(const GoalConstraint& goal) { }

private:

    RobotPlanningSpace* m_space = nullptr;
};

class MultiActionSpace : virtual public ActionSpace {
    public:

    virtual ~MultiActionSpace();

    virtual bool apply(RepId rep_id, const RobotState& parent, std::vector<Action>& actions) = 0;

}

} // namespace smpl

#endif
