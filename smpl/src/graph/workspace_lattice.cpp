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

#include <smpl/graph/workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/graph/workspace_lattice_action_space.h>

auto std::hash<smpl::WorkspaceLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace smpl {

std::ostream& operator<<(std::ostream& o, const WorkspaceLatticeState& s)
{
    o << "{ coord: " << s.coord << ", state: " << s.state << " }";
    return o;
}

template <
    class InputIt,
    class Equal = std::equal_to<typename std::iterator_traits<InputIt>::value_type>>
bool all_equal(InputIt first, InputIt last, typename std::iterator_traits<InputIt>::reference val)
{
    typedef typename std::iterator_traits<InputIt>::reference reference;
    return std::all_of(
            first, last,
            [&val](reference a) { return Equal()(a, val); });
}

WorkspaceLattice::~WorkspaceLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

void WorkspaceLattice::setVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

const std::string& WorkspaceLattice::visualizationFrameId() const
{
    return m_viz_frame_id;
}

bool WorkspaceLattice::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const Params& _params,
    WorkspaceLatticeActionSpace* actions)
{
    if (!WorkspaceLatticeBase::init(_robot, checker, _params)) {
        return false;
    }

    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector
    WorkspaceCoord fake_coord;
    m_goal_state_id = createState(fake_coord);
    m_goal_entry = getState(m_goal_state_id);
    SMPL_DEBUG_NAMED(G_LOG, "  goal state has id %d", m_goal_state_id);

    SMPL_DEBUG_NAMED(G_LOG, "initialize environment");

    m_actions = actions;
    return true;
}

bool WorkspaceLattice::projectToPose(int state_id, Affine3& pose)
{
    if (state_id == getGoalStateID()) {
        pose = goal().pose;
        return true;
    }

    WorkspaceLatticeState* state = getState(state_id);

    double p[6];
    poseCoordToWorkspace(&state->coord[0], &p[0]);

    pose = Translation3(p[FK_PX], p[FK_PY], p[FK_PZ]) *
            AngleAxis(p[FK_QZ], Vector3::UnitZ()) *
            AngleAxis(p[FK_QY], Vector3::UnitY()) *
            AngleAxis(p[FK_QX], Vector3::UnitX());
    return true;
}

bool WorkspaceLattice::setStart(const RobotState& state)
{
    if (!initialized()) {
        SMPL_ERROR_NAMED(G_LOG, "cannot set start state on uninitialized lattice");
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "set the start state");
    if (state.size() < robot()->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "start state contains insufficient coordinate positions");
        return false;
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state: " << state);

    if (!robot()->checkJointLimits(state)) {
        SMPL_ERROR_NAMED(G_LOG, "start state violates joint limits");
        return false;
    }

    if (!collisionChecker()->isStateValid(state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_WARN_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(state));
        SMPL_WARN("start state is in collision");
        return false;
    }

    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(state, vis_name));
    WorkspaceCoord start_coord;
    stateRobotToCoord(state, start_coord);

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord: " << start_coord);

    m_start_state_id = getOrCreateState(start_coord, state);
    m_start_entry = getState(m_start_state_id);

    return RobotPlanningSpace::setStart(state);
}

bool WorkspaceLattice::setGoal(const GoalConstraint& goal)
{
    switch (goal.type) {
    case GoalType::XYZ_RPY_GOAL:
        return setGoalPose(goal);
    case GoalType::JOINT_STATE_GOAL:
        return setGoalJointState(goal);
    case GoalType::USER_GOAL_CONSTRAINT_FN:
        return setUserGoal(goal);
    case GoalType::XYZ_GOAL:
    case GoalType::MULTIPLE_POSE_GOAL:
    default:
        SMPL_WARN("Unimplemented goal type %d", (int)goal.type);
        return false;
    }
}

int WorkspaceLattice::getStartStateID() const
{
    if (m_start_state_id >= 0 && m_goal_state_id >= 0) {
        WorkspaceLatticeState* start_state = getState(m_start_state_id);
        WorkspaceState cont_state;
        stateCoordToWorkspace(start_state->coord, cont_state);
        if (isGoal(cont_state, start_state->state)) {
            return m_goal_state_id;
        }
    }
    return m_start_state_id;
}

int WorkspaceLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

bool WorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    path.clear();

    if (ids.empty()) {
        return true;
    }

    if (ids.size() == 1) {
        auto state_id = ids[0];
        if (state_id == getGoalStateID()) {
            auto* state = getState(m_start_state_id);
            if (!state) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state state for state %d", m_start_state_id);
                return false;
            }
            path.push_back(state->state);
        } else {
            auto* state = getState(state_id);
            if (!state) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(state->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
        return true;
    }

    {
        auto* start_entry = getState(ids[0]);
        path.push_back(start_entry->state);
    }

    for (size_t i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "cannot determine goal state successors during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            // TODO: variant of get succs that returns unique state ids
            auto* prev_entry = getState(prev_id);
            std::vector<WorkspaceAction> actions;
            m_actions->apply(*prev_entry, actions);

            WorkspaceLatticeState* best_goal_entry = NULL;
            auto best_cost = std::numeric_limits<int>::max();

            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                auto& action = actions[aidx];

                RobotState final_rstate;
                if (!checkAction(prev_entry->state, action, &final_rstate)) {
                    continue;
                }

                auto& final_state = action.back();
                if (!isGoal(final_state, final_rstate)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = createState(goal_coord);
                auto* goal_state = getState(goal_id);

                // shouldn't have created a new state, so no need to set the
                // continuous state counterpart
                assert(goal_state->state.size() == robot()->jointVariableCount());

                best_cost = 30; // Hardcoded primitive value in GetSuccs
                best_goal_entry = goal_state;
                break;
            }

            if (!best_goal_entry) {
                SMPL_ERROR_NAMED(G_LOG, "failed to find valid goal successor during path extraction");
                return false;
            }

            path.push_back(best_goal_entry->state);
        } else {
            auto* state_entry = getState(curr_id);
            path.push_back(state_entry->state);
        }
    }

    return true;
}

auto WorkspaceLattice::extractState(int state_id) -> const RobotState&
{
    return this->m_states[state_id]->state;
}

Extension* WorkspaceLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<WorkspaceLattice>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PoseProjectionExtension>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }
    return nullptr;
}

void WorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    // clear the successor arrays
    succs->clear();
    costs->clear();

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    auto* parent_entry = getState(state_id);

    assert(parent_entry);
    assert(parent_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    std::vector<WorkspaceAction> actions;
    m_actions->apply(*parent_entry, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        RobotState final_rstate;
        if (!checkAction(parent_entry->state, action, &final_rstate)) {
            continue;
        }

        auto& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        auto succ_id = getOrCreateState(succ_coord, final_rstate);
        auto* succ_state = getState(succ_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(final_state, final_rstate);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        auto edge_cost = computeCost(*parent_entry, *succ_state);
        costs->push_back(edge_cost);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", edge_cost);
    }
}

void WorkspaceLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
}

void WorkspaceLattice::PrintState(int state_id, bool verbose, FILE* fout)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    if (!fout) {
        fout = stdout;
    }

    WorkspaceLatticeState* state = getState(state_id);

    std::stringstream ss;
    ss << *state;

    if (fout == stdout) {
        SMPL_DEBUG_NAMED(G_LOG, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        SMPL_WARN_NAMED(G_LOG, "%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

void WorkspaceLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    succs->clear();
    costs->clear();

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    WorkspaceLatticeState* state_entry = getState(state_id);

    assert(state_entry);
    assert(state_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  state: " << state_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(state_entry->state, vis_name));

    std::vector<WorkspaceAction> actions;
    m_actions->apply(*state_entry, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        RobotState final_rstate;
        if (!checkLazyAction(state_entry->state, action, &final_rstate)) {
            continue;
        }

        auto& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        auto succ_id = getOrCreateState(succ_coord, final_rstate);
        auto* succ_state = getState(succ_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(final_state, final_rstate);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        auto edge_cost = computeCost(*state_entry, *succ_state);
        costs->push_back(edge_cost);

        true_costs->push_back(false);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", edge_cost);
    }
}

int WorkspaceLattice::GetTrueCost(int parent_id, int child_id)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Evaluate cost of transition %d -> %d", parent_id, child_id);

    assert(parent_id >= 0 && parent_id < (int)m_states.size());
    assert(child_id >= 0 && child_id < (int)m_states.size());

    WorkspaceLatticeState* parent_entry = getState(parent_id);
    WorkspaceLatticeState* child_entry = getState(child_id);
    assert(parent_entry && parent_entry->coord.size() == m_dof_count);
    assert(child_entry && child_entry->coord.size() == m_dof_count);

    std::vector<WorkspaceAction> actions;
    m_actions->apply(*parent_entry, actions);

    auto goal_edge = (child_id == m_goal_state_id);

    WorkspaceCoord succ_coord;
    auto best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        auto& action = actions[aidx];

        RobotState final_rstate;
        if (!checkAction(parent_entry->state, action, &final_rstate)) {
            continue;
        }

        stateWorkspaceToCoord(action.back(), succ_coord);

        if (goal_edge) {
            if (!isGoal(action.back(), final_rstate)) {
                continue;
            }
        } else {
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        auto edge_cost = /* TODO: compute cost */ 30;
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost != std::numeric_limits<int>::max()) {
        return best_cost;
    } else {
        return -1;
    }
}

bool WorkspaceLattice::setGoalPose(const GoalConstraint& goal)
{
    if (!initialized()) {
        SMPL_ERROR_NAMED(G_LOG, "cannot set goal pose on uninitialized lattice");
        return false;
    }

    auto* vis_name = "goal_pose";
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(goal.pose, m_viz_frame_id, vis_name));

    SMPL_DEBUG_NAMED(G_LOG, "set the goal state");

    // check if an IK solution exists for the goal pose before we do
    // the search we plan even if there is no solution
    RobotState seed(robot()->jointVariableCount(), 0);
    RobotState ik_solution;
    if (!m_ik_iface->computeIK(goal.pose, seed, ik_solution)) {
        SMPL_WARN("No valid IK solution for the goal pose.");
    }

    SMPL_DEBUG_NAMED(G_LOG, "  xyz (meters): (%0.2f, %0.2f, %0.2f)", goal.pose.translation()[0], goal.pose.translation()[1], goal.pose.translation()[2]);
    SMPL_DEBUG_NAMED(G_LOG, "  tol (meters): (%0.3f, %0.3f, %0.3f)", goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2]);
    double y, p, r;
    angles::get_euler_zyx(goal.pose.rotation(), y, p, r);
    SMPL_DEBUG_NAMED(G_LOG, "  rpy (radians): (%0.2f, %0.2f, %0.2f)", r, p, y);
    SMPL_DEBUG_NAMED(G_LOG, "  tol (radians): (%0.3f, %0.3f, %0.3f)", goal.rpy_tolerance[0], goal.rpy_tolerance[1], goal.rpy_tolerance[2]);

    m_near_goal = false;
    m_t_start = clock::now();

    return RobotPlanningSpace::setGoal(goal);
}

bool WorkspaceLattice::setGoalJointState(const GoalConstraint& goal)
{
    m_near_goal = false;
    m_t_start = clock::now();

    return RobotPlanningSpace::setGoal(goal);
}

bool WorkspaceLattice::setUserGoal(const GoalConstraint& goal)
{
    m_near_goal = false;
    m_t_start = clock::now();
    return RobotPlanningSpace::setGoal(goal);
}

int WorkspaceLattice::reserveHashEntry()
{
    auto* state = new WorkspaceLatticeState;
    int state_id = (int)this->m_states.size();
    m_states.push_back(state);

    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

/// Create a state entry for a given coordinate and return its id
///
/// If an entry already exists for the coordinate, the id corresponding to that
/// entry is returned; otherwise, a new entry is created and its id returned.
int WorkspaceLattice::createState(const WorkspaceCoord& coord)
{
    WorkspaceLatticeState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit != m_state_to_id.end()) {
        return sit->second;
    }

    int new_id = (int)m_states.size();

    // create a new entry
    auto* state_entry = new WorkspaceLatticeState(state);

    // map id <-> state
    m_states.push_back(state_entry);
    m_state_to_id[state_entry] = new_id;

    int* indices = new int[NUMOFINDICES_STATEID2IND];
    std::fill(indices, indices + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(indices);

    return new_id;
}

/// Retrieve a state by its id.
///
/// The id is not checked for validity and the state is assumed to have already
/// been created, either by GetSuccs during a search or by designating a new
/// start or goal state.
WorkspaceLatticeState* WorkspaceLattice::getState(int state_id) const
{
    assert(state_id >= 0 && state_id < m_states.size());
    return m_states[state_id];
}

int WorkspaceLattice::getOrCreateState(
    const WorkspaceCoord& coord,
    const RobotState& robot_state)
{
    auto state = smpl::WorkspaceLatticeState();
    state.coord = coord;
    auto sit = this->m_state_to_id.find(&state);
    if (sit != this->m_state_to_id.end()) {
        return sit->second;
    }

    auto new_id = (int)this->m_states.size();

    // create a new entry
    auto* state_entry = new smpl::WorkspaceLatticeState(state);

    // map id <-> state
    this->m_states.push_back(state_entry);
    this->m_state_to_id[state_entry] = new_id;

    int* indices = new int[NUMOFINDICES_STATEID2IND];
    std::fill(indices, indices + NUMOFINDICES_STATEID2IND, -1);
    this->StateID2IndexMapping.push_back(indices);

    this->m_states[new_id]->state = robot_state;

    return new_id;
}

bool WorkspaceLattice::isGoal(
    const WorkspaceState& state,
    const RobotState& robot_state) const
{
    // check position
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
        for (int i = 0; i < goal().angles.size(); i++) {
            if (fabs(robot_state[i] - goal().angles[i]) > goal().angle_tolerances[i]) {
                return false;
            }
        }
        return true;
    case GoalType::XYZ_RPY_GOAL: {
        auto dx = std::fabs(state[FK_PX] - goal().pose.translation()[0]);
        auto dy = std::fabs(state[FK_PY] - goal().pose.translation()[1]);
        auto dz = std::fabs(state[FK_PZ] - goal().pose.translation()[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                auto now = clock::now();
                auto time_to_goal_region =
                        std::chrono::duration<double>(now - m_t_start).count();
                m_near_goal = true;
                SMPL_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
            }

            Quaternion qg(goal().pose.rotation());
            Quaternion q(
                    AngleAxis(state[FK_QZ], Vector3::UnitZ()) *
                    AngleAxis(state[FK_QY], Vector3::UnitY()) *
                    AngleAxis(state[FK_QX], Vector3::UnitX()));

            if (q.dot(qg) < 0.0) {
                qg = Quaternion(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

//            const double theta = angles::normalize_angle(AngleAxis(qg.conjugate() * q).angle());
            auto theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < goal().rpy_tolerance[0]) {
                return true;
            }
        }
        return false;
    }   break;
    case GoalType::XYZ_GOAL: {
        SMPL_WARN_ONCE("WorkspaceLattice xyz goals not implemented");
        return false;
    }   break;
    case GoalType::USER_GOAL_CONSTRAINT_FN:
        return goal().check_goal(goal().check_goal_user, state);
    default:
        return false;
    }
}

auto WorkspaceLattice::getStateVisualization(
    const RobotState& state,
    const std::string& ns)
    -> std::vector<visual::Marker>
{
    auto markers = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

bool WorkspaceLattice::checkAction(
    const RobotState& state,
    const WorkspaceAction& action,
    RobotState* final_robot_state)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        auto& waypoint = action[widx];

        SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "        " << widx << ": " << waypoint);

        RobotState seed = state;
        // copy over seed angles from the intermediate state
        for (int i = 0; i < this->freeAngleCount(); ++i) {
            seed[this->m_fangle_indices[i]] = waypoint[6 + i];
        }

        RobotState irstate;
        if (!stateWorkspaceToRobot(waypoint, seed, irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "         -> failed to find ik solution");
            return false;
        }

        if (!robot()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> violates joint limits");
            return false;
        }

        wptraj.push_back(std::move(irstate));
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!collisionChecker()->isStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path to first waypoint in collision");
        return false;
    }

    for (size_t widx = 1; widx < wptraj.size(); ++widx) {
        auto& prev_wp = wptraj[widx - 1];
        auto& curr_wp = wptraj[widx];
        if (!collisionChecker()->isStateToStateValid(prev_wp, curr_wp)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path between waypoints in collision");
            return false;
        }
    }

    if (final_robot_state != NULL) {
        *final_robot_state = wptraj.back();
    }

    return true;
}

int WorkspaceLattice::computeCost(
    const WorkspaceLatticeState& src,
    const WorkspaceLatticeState& dst)
{
    return 30;
}

bool WorkspaceLattice::checkLazyAction(
    const RobotState& state,
    const WorkspaceAction& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        auto& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        " << widx << ": " << istate);

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, state, irstate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "         -> failed to find ik solution");
            return false;
        }

        wptraj.push_back(irstate);

        if (!robot()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> violates joint limits");
            return false;
        }
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

} // namespace smpl
