////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/graph/manip_lattice_action_space.h>

// standard includes
#include <limits>
#include <numeric>
#include <cmath>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {

bool ManipLatticeActionSpace::init(ManipLattice* space)
{
    if (!ActionSpace::init(space)) {
        return false;
    }

    // NOTE: other default thresholds will be set in readParameters, with
    // default values specified in PlanningParams
    m_mprim_thresh[MotionPrimitive::Type::LONG_DISTANCE] =
            std::numeric_limits<double>::infinity();

    clear();

    RobotModel* robot = planningSpace()->robot();

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();

    if (!m_fk_iface) {
        SMPL_WARN("Manip Lattice Action Set requires Forward Kinematics Interface");
    }

    if (!m_ik_iface) {
        SMPL_WARN("Manip Lattice Action Set recommends Inverse Kinematics Interface");
    }

    useMultipleIkSolutions(false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
    useAmp(MotionPrimitive::SNAP_TO_RPY, false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
    useAmp(MotionPrimitive::SHORT_DISTANCE, false);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_RPY, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, 0.2);
    ampThresh(MotionPrimitive::SHORT_DISTANCE, 0.2);

    return true;
}

/// \brief Load motion primitives from file.
///
/// Read in the discrete variable deltas for each motion. If short distance
/// motion primitives are included in the file, they are also enabled.
///
/// Action Set File Format
///
/// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
/// dv11         dv12        ... dv1m
/// ...
/// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
/// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
/// ...
/// dvi1         dvi2        ... dvim
bool ManipLatticeActionSpace::load(const std::string& action_filename)
{
    ROS_ERROR("%s", action_filename.c_str());
    FILE* fCfg = fopen(action_filename.c_str(), "r");
    if (!fCfg) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    char sTemp[1024] = { 0 };
    int nrows = 0;
    int ncols = 0;
    int short_mprims = 0;

    // read and check header
    if (fscanf(fCfg, "%1023s", sTemp) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        SMPL_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // read number of actions
    if (fscanf(fCfg, "%d", &nrows) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read length of joint array
    if (fscanf(fCfg, "%d", &ncols) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read number of short distance motion primitives
    if (fscanf(fCfg, "%d", &short_mprims) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    if (short_mprims == nrows) {
        SMPL_WARN("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }

    std::vector<double> mprim(ncols, 0);

    bool have_short_dist_mprims = short_mprims > 0;
    if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }

    ManipLattice* lattice = static_cast<ManipLattice*>(planningSpace());

    for (int i = 0; i < nrows; ++i) {
        // read joint delta
        for (int j = 0; j < ncols; ++j) {
            double d;
            if (fscanf(fCfg, "%lf", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1.");
                return false;
            }
            if (feof(fCfg)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
            mprim[j] = d * lattice->resolutions()[j];
            SMPL_DEBUG("Got %0.3f deg -> %0.3f rad", d, mprim[j]);
        }

        if (i < (nrows - short_mprims)) {
            addMotionPrim(mprim, false);
        } else {
            addMotionPrim(mprim, true);
        }
    }

    fclose(fCfg);
    return true;
}

/// \brief Add a long or short distance motion primitive to the action set
/// \param mprim The angle delta for each joint, in radians
/// \param short_dist true = short distance; false = long distance
/// \param add_converse Whether to add the negative of this motion primitive
///     to the action set
void ManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }

    m.action.push_back(mprim);
    m_mprims.push_back(m);

    if (add_converse) {
        for (RobotState& state : m.action) {
            for (size_t i = 0; i < state.size(); ++i) {
                state[i] *= -1.0;
            }
        }
        m_mprims.push_back(m);
    }
}

/// \brief Remove long and short motion primitives and disable adaptive motions.
///
/// Thresholds for short distance and adaptive motions are retained
void ManipLatticeActionSpace::clear()
{
    m_mprims.clear();

    // add all amps to the motion primitive set
    MotionPrimitive mprim;

    mprim.type = MotionPrimitive::SNAP_TO_RPY;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = (i == MotionPrimitive::Type::LONG_DISTANCE);
    }
}

int ManipLatticeActionSpace::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ManipLatticeActionSpace::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type) const
{
    return m_mprim_enabled[type];
}

bool ManipLatticeActionSpace::useMultipleIkSolutions() const
{
    return m_use_multiple_ik_solutions;
}

bool ManipLatticeActionSpace::useLongAndShortPrims() const
{
    return m_use_long_and_short_dist_mprims;
}

double ManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type) const
{
    return m_mprim_thresh[type];
}

void ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type, bool enable)
{
    m_mprim_enabled[type] = enable;
}

void ManipLatticeActionSpace::useMultipleIkSolutions(bool enable)
{
    m_use_multiple_ik_solutions = enable;
}

void ManipLatticeActionSpace::useLongAndShortPrims(bool enable)
{
    m_use_long_and_short_dist_mprims = enable;
}

void ManipLatticeActionSpace::ampThresh(
    MotionPrimitive::Type type,
    double thresh)
{
    if (type != MotionPrimitive::LONG_DISTANCE) {
        m_mprim_thresh[type] = thresh;
    }
}

auto ManipLatticeActionSpace::getStartGoalDistances(const RobotState& state)
    -> std::pair<double, double>
{
    if (!m_fk_iface) {
        return std::make_pair(0.0, 0.0);
    }

    auto pose = m_fk_iface->computeFK(state);

    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristic* h = planningSpace()->heuristic(0);
        double start_dist = h->getMetricStartDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        double goal_dist = h->getMetricGoalDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        return std::make_pair(start_dist, goal_dist);
    } else {
        return std::make_pair(0.0, 0.0);
    }
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    double goal_dist, start_dist;
    std::tie(start_dist, goal_dist) = getStartGoalDistances(parent);

    for (auto& prim : m_mprims) {
        (void)getAction(parent, goal_dist, start_dist, prim, actions);
    }

    if (actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

bool ManipLatticeActionSpace::getAction(
    const RobotState& parent,
    double goal_dist,
    double start_dist,
    const MotionPrimitive& mp,
    std::vector<Action>& actions)
{
    if (!mprimActive(start_dist, goal_dist, mp.type)) {
        return false;
    }

    GoalType goal_type = planningSpace()->goal().type;
    auto& goal_pose = planningSpace()->goal().pose;

    switch (mp.type) {
    case MotionPrimitive::LONG_DISTANCE:  // fall-through
    case MotionPrimitive::SHORT_DISTANCE:
    {
        Action action;
        if (!applyMotionPrimitive(parent, mp, action)) {
            return false;
        }
        actions.push_back(std::move(action));
        return true;
    }
    case MotionPrimitive::SNAP_TO_RPY:
    {
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_XYZ,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_RPY,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
            return computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::UNRESTRICTED,
                    actions);
        }

        // goal is 7dof; instead of computing IK, use the goal itself as the IK
        // solution
        Action action = { planningSpace()->goal().angles };
        actions.push_back(std::move(action));

        return true;
    }
    default:
        SMPL_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ManipLatticeActionSpace::applyMotionPrimitive(
    const RobotState& state,
    const MotionPrimitive& mp,
    Action& action)
{
    action = mp.action;
    for (size_t i = 0; i < action.size(); ++i) {
        if (action[i].size() != state.size()) {
            return false;
        }

        double theta;
        theta = state[2];
        auto r = action[i][0];
        // Resolve global x component.
        action[i][0] = cos(theta)*r + state[0];
        action[i][1] = sin(theta)*r + state[1];
        for (size_t j = 2; j < action[i].size(); ++j) {
            action[i][j] = action[i][j] + state[j];
        }
    }
    return true;
}

bool ManipLatticeActionSpace::computeIkAction(
    const RobotState& state,
    const Affine3& goal,
    double dist_to_goal,
    ik_option::IkOption option,
    std::vector<Action>& actions)
{
    if (!m_ik_iface) {
        return false;
    }

    if (m_use_multiple_ik_solutions) {
        //get actions for multiple ik solutions
        std::vector<RobotState> solutions;
        if (!m_ik_iface->computeIK(goal, state, solutions, option)) {
            return false;
        }
        for (auto& solution : solutions) {
            Action action = { std::move(solution) };
            actions.push_back(std::move(action));
        }
    } else {
        //get single action for single ik solution
        RobotState ik_sol;
        if (!m_ik_iface->computeIK(goal, state, ik_sol)) {
            return false;
        }

        Action action = { std::move(ik_sol) };
        actions.push_back(std::move(action));
    }

    return true;
}

bool ManipLatticeActionSpace::mprimActive(
    double start_dist,
    double goal_dist,
    MotionPrimitive::Type type) const
{
    // TODO: this seems a awkward..."short distance" motion primitives should be
    // the default since "long distance" primitives are usually implemented as
    // an optimization in regions far from the start or goal, and often we
    // always need "short distance" motion primitives near the start and goal.
    // -Andrew
    if (type == MotionPrimitive::LONG_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return true;
        }
        const bool near_goal =
                goal_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_start =
                start_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_endpoint = near_goal || near_start;
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] && near_endpoint);
    } else if (type == MotionPrimitive::SHORT_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return m_mprim_enabled[type];
        }
        const bool near_goal = goal_dist <= m_mprim_thresh[type];
        const bool near_start = start_dist <= m_mprim_thresh[type];
        const bool near_endpoint = near_goal || near_start;
        return m_mprim_enabled[type] && near_endpoint;
    } else {
        return m_mprim_enabled[type] && goal_dist <= m_mprim_thresh[type];
    }
}

bool ManipLatticeMultiActionSpace::init(ManipLattice* space){
    clear();
    m_rep_mprims.resize(numReps());
    return ManipLatticeActionSpace::init(space);
}

bool ManipLatticeMultiActionSpace::load( const std::string& action_filename ){
    return load( 0, action_filename );
}

bool ManipLatticeMultiActionSpace::load( RepId rep_id, const std::string& action_filename ) {
    ROS_ERROR("%s", action_filename.c_str());
    FILE* fCfg = fopen(action_filename.c_str(), "r");
    if (!fCfg) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    char sTemp[1024] = { 0 };
    int nrows = 0;
    int ncols = 0;
    int short_mprims = 0;

    // read and check header
    if (fscanf(fCfg, "%1023s", sTemp) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        SMPL_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // read number of actions
    if (fscanf(fCfg, "%d", &nrows) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read length of joint array
    if (fscanf(fCfg, "%d", &ncols) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read number of short distance motion primitives
    if (fscanf(fCfg, "%d", &short_mprims) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    if (short_mprims == nrows) {
        SMPL_WARN("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }

    std::vector<double> mprim(ncols, 0);

    bool have_short_dist_mprims = short_mprims > 0;
    if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }

    ManipLattice* lattice = static_cast<ManipLattice*>(planningSpace());

    for (int i = 0; i < nrows; ++i) {
        // read joint delta
        for (int j = 0; j < ncols; ++j) {
            double d;
            if (fscanf(fCfg, "%lf", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1.");
                return false;
            }
            if (feof(fCfg)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
            mprim[j] = d * lattice->resolutions()[j];
            SMPL_DEBUG("Got %0.3f deg -> %0.3f rad", d, mprim[j]);
        }

        if (i < (nrows - short_mprims)) {
            addMotionPrim(rep_id, mprim, false);
        } else {
            addMotionPrim(rep_id, mprim, true);
        }
    }

    fclose(fCfg);
    return true;
}

void ManipLatticeMultiActionSpace::addMotionPrim(
        const std::vector<double>& mprim,
        bool short_dist_mprim,
        bool add_converse){
    addMotionPrim( 0, mprim, short_dist_mprim, add_converse );
}

void ManipLatticeMultiActionSpace::addMotionPrim(
        RepId rep_id,
        const std::vector<double>& mprim,
        bool short_dist_mprim,
        bool add_converse){
    assert(rep_id < numReps());

    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }

    m.action.push_back(mprim);
    m_rep_mprims[rep_id].push_back(m);

    if (add_converse) {
        for (RobotState& state : m.action) {
            for (size_t i = 0; i < state.size(); ++i) {
                state[i] *= -1.0;
            }
        }
        m_rep_mprims[rep_id].push_back(m);
    }
}

void ManipLatticeMultiActionSpace::clear(){
    ManipLatticeActionSpace::clear();
    m_rep_mprims.clear();
}

bool ManipLatticeMultiActionSpace::apply(
        const RobotState& _parent,
        std::vector<Action>& _actions ){
    return apply(0, _parent, _actions);
}

bool ManipLatticeMultiActionSpace::apply(
        RepId _rep_id,
        const RobotState& _parent,
        std::vector<Action>& _actions ){

    double goal_dist, start_dist;
    std::tie(start_dist, goal_dist) = getStartGoalDistances(_parent);

    for (auto& prim : m_rep_mprims[_rep_id]) {
        (void)getAction(_parent, goal_dist, start_dist, prim, _actions);
    }

    if (_actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

} // namespace smpl
