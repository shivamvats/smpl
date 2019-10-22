////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_PLANNER_INTERFACE_H
#define SMPL_PLANNER_INTERFACE_H

// standard includes
#include <map>
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sbpl/headers.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/debug/marker.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/heuristic/robot_heuristic.h>

class SBPLPlanner;

namespace smpl {

using PlanningSpaceFactory = std::function<
        std::unique_ptr<RobotPlanningSpace>(
                RobotModel*, CollisionChecker*, const PlanningParams&)>;

using HeuristicFactory = std::function<
        std::unique_ptr<RobotHeuristic>(
                RobotPlanningSpace*, const PlanningParams&)>;

using PlannerFactory = std::function<
        std::unique_ptr<SBPLPlanner>(
                RobotPlanningSpace*, RobotHeuristic*, const PlanningParams&)>;

using MHPlannerFactory = std::function<
        std::unique_ptr<SBPLPlanner>(
                RobotPlanningSpace*, std::vector<RobotHeuristic*>, const PlanningParams&)>;


using GoalConstraints = std::vector<moveit_msgs::Constraints>;

class PlannerInterface
{
public:

    PlannerInterface(
        RobotModel* robot,
        CollisionChecker* checker,
        OccupancyGrid* grid);

    ~PlannerInterface();

    bool init(const PlanningParams& params);

    bool solve(
        const moveit_msgs::PlanningScene& planning_scene,
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res);

    static
    bool SupportsGoalConstraints(
        const GoalConstraints& constraints,
        std::string& why);

    bool canServiceRequest(
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res) const;

    auto space() const -> const RobotPlanningSpace* { return m_pspace.get(); }
    auto search() const -> const SBPLPlanner* { return m_planner.get(); }

    using heuristic_iterator =
            std::map<std::string, std::unique_ptr<RobotHeuristic>>::const_iterator;

    auto heuristics() const -> std::pair<heuristic_iterator, heuristic_iterator> {
        return std::make_pair(begin(m_heuristics), end(m_heuristics));
    }

    /// @brief Return planning statistics from the last call to solve.
    ///
    /// Possible keys to statistics include:
    ///     "initial solution planning time"
    ///     "initial epsilon"
    ///     "initial solution expansions"
    ///     "final epsilon planning time"
    ///     "final epsilon"
    ///     "solution epsilon"
    ///     "expansions"
    ///     "solution cost"
    ///
    /// @return The statistics
    auto getPlannerStats() -> std::map<std::string, double>;

    /// \name Visualization
    ///@{

    auto getBfsWallsVisualization() const -> visual::Marker;
    auto getBfsValuesVisualization() const -> visual::Marker;

    auto makePathVisualization(const std::vector<RobotState>& path) const
        -> std::vector<visual::Marker>;
    ///@}

    std::map<std::string, std::unique_ptr<RobotHeuristic>> m_heuristics;
    std::vector<RobotHeuristic*> m_heuristic_vector;

protected:

    RobotModel* m_robot;
    CollisionChecker* m_checker;
    OccupancyGrid* m_grid;

    ForwardKinematicsInterface* m_fk_iface;

    PlanningParams m_params;

    int m_num_heuristics;
    // params
    bool m_initialized;

    std::map<std::string, PlanningSpaceFactory> m_space_factories;
    std::map<std::string, HeuristicFactory> m_heuristic_factories;
    std::map<std::string, PlannerFactory> m_planner_factories;
    std::map<std::string, MHPlannerFactory> m_mh_planner_factories;

    // planner components

    std::unique_ptr<RobotPlanningSpace> m_pspace;
    std::unique_ptr<SBPLPlanner> m_planner;

    int m_sol_cost;

    std::string m_planner_id;

    // Set start configuration
    bool setGoal(const GoalConstraints& v_goal_constraints);
    bool setStart(const moveit_msgs::RobotState& state);

    // Retrieve plan from sbpl
    bool plan(double allowed_time, std::vector<RobotState>& path);

    bool parsePlannerID(
        const std::string& planner_id,
        std::string& space_name,
        std::vector<std::string>& heuristic_names,
        std::string& search_name) const;

    bool reinitPlanner(const std::string& planner_id);

    void postProcessPath(std::vector<RobotState>& path) const;
};

} // namespace smpl

#endif
