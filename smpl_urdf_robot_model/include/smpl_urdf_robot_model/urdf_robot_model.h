#ifndef SMPL_URDF_ROBOT_MODEL_URDF_ROBOT_MODEL_H
#define SMPL_URDF_ROBOT_MODEL_URDF_ROBOT_MODEL_H

// system includes
#include <smpl/robot_model.h>

// project includes
#include <smpl_urdf_robot_model/robot_state.h>
#include <smpl_urdf_robot_model/robot_model.h>

namespace smpl {
namespace urdf {

struct RobotModel;

struct URDFRobotModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface
{
    struct VariableProperties
    {
        double min_position;
        double max_position;
        double vel_limit;
        double acc_limit;
        bool continuous;
        bool bounded;
    };

    const ::smpl::urdf::RobotModel* robot_model = NULL;

    // persistent robot state to cache unchanging transforms
    RobotState robot_state;

    std::vector<VariableProperties> vprops;
    std::vector<int> planning_to_state_variable;
    const Link* planning_link = NULL;

    auto computeFK(const smpl::RobotState& state)
        -> Eigen::Affine3d override;

    const smpl::Affine3* getLinkTransform( const std::string& link_name ) override
    {
        auto* link = GetLink(robot_model, &link_name);
        auto frame = GetLinkTransform(&robot_state, link);
        return frame;
    }

    double minPosLimit(int jidx) const override;
    double maxPosLimit(int jidx) const override;
    bool hasPosLimit(int jidx) const override;
    bool isContinuous(int jidx) const override;
    double velLimit(int jidx) const override;
    double accLimit(int jidx) const override;
    bool checkJointLimits(
        const smpl::RobotState& state,
        bool verbose = false) override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;
};

bool Init(
    URDFRobotModel* urdf_model,
    const RobotModel* robot_model,
    const std::vector<std::string>* planning_joint_names);
bool Init(
    URDFRobotModel* urdf_model,
    const RobotModel* robot_model,
    const std::vector<const Joint*>* planning_joints);

bool SetPlanningLink(URDFRobotModel* urdf_model, const char* link_name);
bool SetPlanningLink(URDFRobotModel* urdf_model, const std::string* link_name);
bool SetPlanningLink(URDFRobotModel* urdf_model, const Link* link);

void SetReferenceState(URDFRobotModel* urdf_model, const double* positions);

} // namespace urdf
} // namespace smpl

#endif

