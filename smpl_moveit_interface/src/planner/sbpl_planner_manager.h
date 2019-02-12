#ifndef sbpl_interface_sbpl_planner_manager_h
#define sbpl_interface_sbpl_planner_manager_h

// system includes
#include <XmlRpcValue.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <smpl/debug/visualizer_ros.h>

// project includes
#include <smpl_moveit_interface/planner/moveit_robot_model.h>

namespace sbpl_interface {

MOVEIT_CLASS_FORWARD(SBPLPlanningContext);

using MotionPlanRequest = planning_interface::MotionPlanRequest;
using PlanningContextPtr = planning_interface::PlanningContextPtr;
using PlannerConfigurationMap = planning_interface::PlannerConfigurationMap;
using PlannerConfigurationSettings = planning_interface::PlannerConfigurationSettings;

class SBPLPlannerManager : public planning_interface::PlannerManager
{
public:

    static const std::string DefaultPlanningAlgorithm;

    using Base = planning_interface::PlannerManager;

    SBPLPlannerManager();
    virtual ~SBPLPlannerManager();

    /// \name planning_interface::PlannerManager Interface
    ///@{

    /// \sa planning_interface::PlannerManager::initialize()
    bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns) override;

    /// \sa planning_interface::PlannerManger::getDescription()
    auto getDescription() const -> std::string override;

    /// \sa planning_interface::PlannerManager:::getPlanningAlgorithms()
    void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

    /// \sa planning_interface::PlannerManager::getPlanningContext()
    auto getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const
        -> PlanningContextPtr override;

    /// \sa planning_interface::PlannerManager::canServiceRequest()
    bool canServiceRequest(const MotionPlanRequest& req) const override;

    /// \sa planning_interface::PlannerManager::setPlannerConfigurations()
    void setPlannerConfigurations(const PlannerConfigurationMap& pcs) override;

    ///@}

    moveit::core::RobotModelConstPtr m_robot_model;

    // per-group sbpl robot model
    // TODO: make unique per context instance
    std::map<std::string, std::unique_ptr<MoveItRobotModel>> m_sbpl_models;

    // per-configuration context
    std::map<std::string, SBPLPlanningContextPtr> m_contexts;

    smpl::VisualizerROS m_viz;

    PlannerConfigurationMap map;
};

MOVEIT_CLASS_FORWARD(SBPLPlannerManager);

} // namespace sbpl_interface

#endif
