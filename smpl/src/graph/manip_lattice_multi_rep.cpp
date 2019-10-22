
#include <smpl/graph/manip_lattice_multi_rep.h>

// standard includes
#include <iomanip>
#include <sstream>
#include <stdlib.h>

// system includes
#include <sbpl/planners/planner.h>

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/spatial.h>
#include "../profiling.h"
#include <smpl/utils.h>

namespace smpl {

ManipLatticeMultiRep::~ManipLatticeMultiRep() { }

bool ManipLatticeMultiRep::init(
        RobotModel* _robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        MultiActionSpace* action_space)
{
    SMPL_DEBUG_NAMED(G_LOG, "Initialize Manip Lattice Multi Rep");

    if (resolutions.size() != _robot->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "Insufficient variable resolutions for robot model");
        return false;
    }

    if (!RobotPlanningSpace::init(_robot, checker)) {
        SMPL_ERROR_NAMED(G_LOG, "Failed to initialize Robot Planning Space");
        return false;
    }

    m_fk_iface = _robot->getExtension<ForwardKinematicsInterface>();

    m_min_limits.resize(_robot->jointVariableCount());
    m_max_limits.resize(_robot->jointVariableCount());
    m_continuous.resize(_robot->jointVariableCount());
    m_bounded.resize(_robot->jointVariableCount());
    for (int jidx = 0; jidx < _robot->jointVariableCount(); ++jidx) {
        m_min_limits[jidx] = _robot->minPosLimit(jidx);
        m_max_limits[jidx] = _robot->maxPosLimit(jidx);
        m_continuous[jidx] = _robot->isContinuous(jidx);
        m_bounded[jidx] = _robot->hasPosLimit(jidx);

        SMPL_DEBUG_NAMED(G_LOG, "variable %d: { min: %f, max: %f, continuous: %s, bounded: %s }",
            jidx,
            m_min_limits[jidx],
            m_max_limits[jidx],
            m_continuous[jidx] ? "true" : "false",
            m_bounded[jidx] ? "true" : "false");
    }

    m_goal_state_id = reserveHashEntry();
    SMPL_DEBUG_NAMED(G_LOG, "  goal state has state ID %d", m_goal_state_id);

    std::vector<int> discretization(_robot->jointVariableCount());
    std::vector<double> deltas(_robot->jointVariableCount());
    for (size_t vidx = 0; vidx < _robot->jointVariableCount(); ++vidx) {
        if (m_continuous[vidx]) {
            discretization[vidx] = (int)std::round((2.0 * M_PI) / resolutions[vidx]);
            deltas[vidx] = (2.0 * M_PI) / (double)discretization[vidx];
        } else if (m_bounded[vidx]) {
            auto span = std::fabs(m_max_limits[vidx] - m_min_limits[vidx]);
            discretization[vidx] = std::max(1, (int)std::round(span / resolutions[vidx]));
            deltas[vidx] = span / (double)discretization[vidx];
        } else {
            discretization[vidx] = std::numeric_limits<int>::max();
            deltas[vidx] = resolutions[vidx];
        }
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord vals: " << discretization);
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord deltas: " << deltas);

    m_coord_vals = std::move(discretization);
    m_coord_deltas = std::move(deltas);

    m_multi_action_space = action_space;

    return true;
}

void ManipLatticeMultiRep::GetSuccs( int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs ){
    // Call the default/full action space.
    GetSuccs( state_id, 0, succs, costs );
}

void ManipLatticeMultiRep::GetSuccs( int state_id,
        int rep_id,
        std::vector<int>* succs,
        std::vector<int>* costs ) {
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(rep_id < m_multi_action_space->numReps() && "representation id outside bounds.");

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << parent_entry->state);

    auto* vis_name = "expansion";
    float c = 0.6;
    visual::Color color{0, 0, 0, c};
    switch(rep_id){
        case 0:
            color.r = 1;
            break;
        case 1:
            color.g = 1;
            break;
        case 2:
            color.b = 1;
            break;
        default:
            break;
    }
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name, color));

    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!m_multi_action_space->apply(rep_id, parent_entry->state, actions)) {
        SMPL_WARN("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        //SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        //SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));

        // log successor details
        //SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %zu", i);
        //SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        id: %5i", succ_state_id);
        //SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_coord);
        //SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_entry->state);
        //SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Got %d goal successors!", goal_succ_count);
    }
}

// This cost function makes the planner minimize the number of actions (all
// actions considered equivalent).
int ManipLatticeMultiRep::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    bool bState2IsGoal) const
{
    auto& start_state = HashEntry1->state;
    auto& end_state = HashEntry2->state;
    auto DefaultCostMultiplier = 1000;
    /**
    std::vector<double> W( start_state.size(), 1 );
    W[0]*= 10;
    W[1]*= 10;
    W[2]*= 1;
    W[3]*= 10;
    W[4]*= 10;
    W[5]*= 10;
    W[6]*= 20;
    W[7]*= 20;
    W[8]*= 20;
    W[9]*= 20;
    int cost = DefaultCostMultiplier*euclideanDistance<double>( start_state, end_state, W );
    **/
    return DefaultCostMultiplier;//cost;
}

bool ManipLatticeMultiRep::checkAction(const RobotState& state, const Action& action)
{
    std::uint32_t violation_mask = 0x00000000;

    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        " << iidx << ": " << istate);

        // check joint limits
        if (!robot()->checkJointLimits(istate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!collisionChecker()->isStateValid(istate))
//        {
//            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> in collision);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
    if (!collisionChecker()->isStateToStateValid(state, action[0])) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> path to first waypoint in collision");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        auto& prev_istate = action[j - 1];
        auto& curr_istate = action[j];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate))
        {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> path between waypoints %zu and %zu in collision", j - 1, j);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

static
bool WithinPositionTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    auto dx = std::fabs(A.translation()[0] - B.translation()[0]);
    auto dy = std::fabs(A.translation()[1] - B.translation()[1]);
    auto dz = std::fabs(A.translation()[2] - B.translation()[2]);
    return dx <= tol[0] && dy <= tol[1] && dz <= tol[2];
}

static
bool WithinOrientationTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    Quaternion qg(B.rotation());
    Quaternion q(A.rotation());
    if (q.dot(qg) < 0.0) {
        qg = Quaternion(-qg.w(), -qg.x(), -qg.y(), -qg.z());
    }

    auto theta = normalize_angle(2.0 * acos(q.dot(qg)));
    return theta < tol[0];
}

static
auto WithinTolerance(
    const Affine3& A,
    const Affine3& B,
    const double xyz_tolerance[3],
    const double rpy_tolerance[3])
    -> std::pair<bool, bool>
{
    if (WithinPositionTolerance(A, B, xyz_tolerance)) {
        if (WithinOrientationTolerance(A, B, rpy_tolerance)) {
            return std::make_pair(true, true);
        } else {
            return std::make_pair(true, false);
        }
    }
    return std::make_pair(false, false);
}


bool ManipLatticeMultiRep::setStart(const RobotState& state)
{
    SMPL_DEBUG_NAMED(G_LOG, "set the start state");

    if ((int)state.size() < robot()->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "start state does not contain enough joint positions");
        return false;
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state: " << state);

    // check joint limits of starting configuration
    if (!robot()->checkJointLimits(state, true)) {
        SMPL_WARN(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    if (!collisionChecker()->isStateValid(state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_WARN_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(state));
        SMPL_WARN(" -> in collision");
        return false;
    }

    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(state, vis_name));

    // get arm position in environment
    auto start_coord = RobotCoord(robot()->jointVariableCount());
    stateToCoord(state, start_coord);
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord: " << start_coord);

    m_start_state_id = getOrCreateState(start_coord, state);

    m_multi_action_space->updateStart(state);

    // notify observers of updated start state
    return RobotPlanningSpace::setStart(state);
}

bool ManipLatticeMultiRep::setGoal(const GoalConstraint& goal)
{
    auto success = false;

    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
        success = setGoalPose(goal);
        break;
    case GoalType::MULTIPLE_POSE_GOAL:
        success = setGoalPoses(goal);
        break;
    case GoalType::JOINT_STATE_GOAL:
        success = setGoalConfiguration(goal);
        break;
    case GoalType::USER_GOAL_CONSTRAINT_FN:
        success = setUserGoal(goal);
        break;
    default:
        return false;
    }

    if (success) {
        m_multi_action_space->updateGoal(goal);
    }

    return success;
}

bool ManipLatticeMultiRep::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    if (idpath.empty()) {
        return true;
    }

    std::vector<RobotState> opath;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        auto state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            auto* entry = getHashEntry(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            opath.push_back(entry->state);
        } else {
            auto* entry = getHashEntry(state_id);

            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            opath.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(opath.back(), vis_name));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    // grab the first point
    {
        auto* entry = getHashEntry(idpath[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            SMPL_DEBUG_NAMED(G_LOG, "Search for transition to goal state");

            ManipLatticeState* prev_entry = m_states[prev_id];
            auto& prev_state = prev_entry->state;

            std::vector<Action> actions;
            //XXX At the goal, consider the full Action Space.
            if (!m_multi_action_space->apply(prev_state, actions)) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get actions while extracting the path");
                return false;
            }

            // find the goal state corresponding to the cheapest valid action
            ManipLatticeState* best_goal_state = nullptr;
            RobotCoord succ_coord(robot()->jointVariableCount());
            int best_cost = std::numeric_limits<int>::max();
            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                auto& action = actions[aidx];

                // skip non-goal states
                if (!isGoal(action.back())) {
                    continue;
                }

                // check the validity of this transition
                if (!checkAction(prev_state, action)) {
                    continue;
                }

                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);

                auto edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_goal_state = succ_entry;
                }
            }

            if (!best_goal_state) {
                SMPL_ERROR_STREAM_NAMED(G_LOG, "Failed to find valid goal successor from state " << prev_entry->state << " during path extraction");
                return false;
            }

            opath.push_back(best_goal_state->state);
        } else {
            auto* entry = getHashEntry(curr_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry state %d", curr_id);
                return false;
            }

            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Extract successor state " << entry->state);
            opath.push_back(entry->state);
        }
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

Extension* ManipLatticeMultiRep::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }

    if (class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<PoseProjectionExtension>())
    {
        if (m_fk_iface) {
            return this;
        }
    }

    return nullptr;
}


/*
/// Set a 6-dof goal pose for the planning link
bool ManipLatticeMultiRep::setGoalPose(const GoalConstraint& gc)
{
    auto* vis_name = "goal_pose";
    ROS_ERROR("Visualizing the goal pose.");
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(gc.pose, m_viz_frame_id, vis_name));

    using namespace std::chrono;
    auto now = clock::now();
    auto now_s = duration_cast<duration<double>>(now.time_since_epoch());
    SMPL_DEBUG_NAMED(G_LOG, "time: %f", now_s.count());
    SMPL_DEBUG_NAMED(G_LOG, "A new goal has been set.");
    SMPL_DEBUG_NAMED(G_LOG, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", gc.pose.translation()[0], gc.pose.translation()[1], gc.pose.translation()[2]);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (meters): %0.3f", gc.xyz_tolerance[0]);
    double yaw, pitch, roll;
    get_euler_zyx(gc.pose.rotation(), yaw, pitch, roll);
    SMPL_DEBUG_NAMED(G_LOG, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", roll, pitch, yaw);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (radians): %0.3f", gc.rpy_tolerance[0]);


    // set the (modified) goal
    return RobotPlanningSpace::setGoal(gc);
}

bool ManipLatticeMultiRep::setGoalPoses(const GoalConstraint& gc)
{
    // TODO: a visualization would be nice
    return RobotPlanningSpace::setGoal(gc);
}

/// \brief Set a full joint configuration goal.
bool ManipLatticeMultiRep::setGoalConfiguration(const GoalConstraint& goal)
{
    if (goal.angles.size() != robot()->jointVariableCount() ||
        goal.angle_tolerances.size() != robot()->jointVariableCount())
    {
        return false;
    }

    auto vis_name = "target_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(goal.angles, vis_name));

    SMPL_INFO_NAMED(G_LOG, "A new goal has been set");
    SMPL_INFO_STREAM_NAMED(G_LOG, "  config: " << goal.angles);
    SMPL_INFO_STREAM_NAMED(G_LOG, "  tolerance: " << goal.angle_tolerances);

    // notify observers of updated goal
    return RobotPlanningSpace::setGoal(goal);
}

bool ManipLatticeMultiRep::setUserGoal(const GoalConstraint& goal)
{
    return RobotPlanningSpace::setGoal(goal);
}
*/

} // namespace smpl
