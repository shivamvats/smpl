#ifndef TRAC_IK_ROBOT_MODEL_TRAC_IK_ROBOT_MODEL_H
#define TRAC_IK_ROBOT_MODEL_TRAC_IK_ROBOT_MODEL_H

// standard includes
#include <memory>
#include <string>

// system includes
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/tree.hpp>
#include <smpl/robot_model.h>
#include <smpl_urdf_robot_model/smpl_urdf_robot_model.h>
#include <urdf/model.h>

namespace smpl {

class TracIKRobotModel :
    public virtual urdf::URDFRobotModel,
    public virtual InverseKinematicsInterface,
    public virtual RedundantManipulatorInterface
{
public:

    static const int DEFAULT_FREE_ANGLE_INDEX = 2;
    TracIKRobotModel() {}

    bool init(
        const std::string& robot_description,
        const std::string& base_link,
        const std::string& tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    auto getBaseLink() const -> const std::string&;
    auto getPlanningLink() const -> const std::string&;

    bool computeIKSearch(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution);

    void printRobotModelInformation();

    /// \name RedundantManipulatorInterface
    /// @{
    const int redundantVariableCount() const override { return 0; }
    const int redundantVariableIndex(int vidx) const override { return 0.0; }
    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution) override;
    /// @}

    /// \name InverseKinematicsInterface Interface
    ///@{
    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        std::vector<RobotState>& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> Extension* override;
    ///@}

public:

    ::urdf::Model m_urdf;

    urdf::RobotModel m_robot_model;

    const urdf::Link* m_kinematics_link = NULL;

    std::string m_base_link;
    std::string m_tip_link;

    KDL::Tree m_tree;
    KDL::Chain m_chain;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> m_fk_solver;
    std::unique_ptr<TRAC_IK::TRAC_IK> m_ik_solver;

    // ik solver settings
    int m_max_iterations;
    double m_kdl_eps;

    // temporary storage
    KDL::JntArray m_jnt_pos_in;
    KDL::JntArray m_jnt_pos_out;

    // ik search configuration
    int m_free_angle;
    double m_search_discretization;
    double m_timeout;
};

} // namespace smpl

#endif
