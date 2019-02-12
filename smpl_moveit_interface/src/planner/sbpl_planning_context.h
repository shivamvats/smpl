#ifndef sbpl_interface_SBPLPlanningContext_h
#define sbpl_interface_SBPLPlanningContext_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/distance_map/distance_map_interface.h>

// project includes
#include <smpl_moveit_interface/planner/moveit_robot_model.h>

#include "moveit_collision_checker.h"

namespace sbpl_interface {

class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:

    typedef planning_interface::PlanningContext Base;

    /// \brief Construct an SBPL Planning Context with a pre-constructed
    SBPLPlanningContext(
        MoveItRobotModel* robot_model,
        const std::string& name,
        const std::string& group);

    virtual ~SBPLPlanningContext();

    /// \sa planning_interface::PlanningContext::solve(planning_interface::MotionPlanResponse&)
    virtual bool solve(planning_interface::MotionPlanResponse& res);

    /// \sa planning_interface::PlanningContext::solve(planning_interface::MotionPlanDetailedResponse&)
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

    /// \sa planning_interface::PlanningContext::terminate
    virtual bool terminate();

    /// \sa planning_interface::PlanningContext::clear
    virtual void clear();

    /// \brief Initialize the SBPL Planning Context
    ///
    /// This phase of initialization only initializes structures that do not
    /// depend on having an active PlanningScene or MotionPlanRequest. A second
    /// phase of initialization happens on the first call to solve() which will
    /// initialize structures dependant on either of these things.
    ///
    /// The MoveItRobotModel passed to the constructor must be initialized
    /// before this initialization is possible.
    bool init(const std::map<std::string, std::string>& config);

    // sbpl planner components
    MoveItRobotModel* m_robot_model;
    std::unique_ptr<MoveItCollisionChecker> m_collision_checker;

    std::unique_ptr<smpl::OccupancyGrid> m_grid;

    std::unique_ptr<smpl::PlannerInterface> m_planner;

    std::map<std::string, std::string> m_config;
    smpl::PlanningParams m_pp;

    // The smpl-ized planner id ((search, heuristic, graph) triple)
    std::string m_planner_id;

    bool m_use_grid;
    double m_grid_res_x;
    double m_grid_res_y;
    double m_grid_res_z;
    double m_grid_inflation_radius;

    moveit_msgs::WorkspaceParameters m_prev_workspace;
    planning_scene::PlanningSceneConstPtr m_prev_scene;
};

MOVEIT_CLASS_FORWARD(SBPLPlanningContext);

} // namespace sbpl_interface

#endif
