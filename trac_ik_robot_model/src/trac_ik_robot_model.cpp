#include <trac_ik_robot_model/trac_ik_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/stl/memory.h>

namespace smpl {

namespace {
bool getCount(int& _count, int _max_count, int _min_count){
    if (_count > 0) {
        if (-_count >= _min_count) {
            _count = -_count;
            return true;
        } else if (_count + 1 <= _max_count) {
            _count = _count + 1;
            return true;
        } else {
            return false;
        }
    } else {
        if (1 - _count <= _max_count) {
            _count = 1 - _count;
            return true;
        } else if (_count - 1 >= _min_count) {
            _count = _count - 1;
            return true;
        } else {
            return false;
        }
    }
}
}

namespace {
bool Init(
    TracIKRobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle) {
    ROS_INFO("Initialize TracIK Robot Model");
    if (!model->m_urdf.initString(robot_description)) {
        ROS_ERROR("Failed to parse the URDF.");
        return false;
    }

    ROS_INFO("Initialize Robot Model");
    urdf::JointSpec world_joint;
    world_joint.name = "map";
    world_joint.origin = Eigen::Affine3d::Identity(); // IMPORTANT
    world_joint.axis = Eigen::Vector3d::Zero();
    world_joint.type = urdf::JointType::Fixed;
    if (!urdf::InitRobotModel(
            &model->m_robot_model, &model->m_urdf, &world_joint))
    {
        ROS_ERROR("Failed to initialize Robot Model");
        return false;
    }

    //ROS_INFO("Initialize KDL tree");
    //if (!tracik_solver.getKDLChain(chain);
//) {
        //ROS_ERROR("Failed to parse the kdl tree from robot description.");
        //return false;
    //}

    const double timeout = 0.005;
    const double eps = 1e-5;
    ROS_INFO("Construct TracIk solver");
    model->m_ik_solver = std::make_unique<TRAC_IK::TRAC_IK>(
            base_link, tip_link, "/robot_description", timeout, eps);

    ROS_INFO("Initialize KDL chain (%s, %s)", base_link.c_str(), tip_link.c_str());
    if(!model->m_ik_solver->getKDLChain(model->m_chain)){
        ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", base_link.c_str(), tip_link.c_str());
        return false;
    }

    model->m_base_link = base_link;
    model->m_tip_link = tip_link;
    model->m_kinematics_link = GetLink(&model->m_robot_model, &base_link);
    if (model->m_kinematics_link == NULL) {
        return false; // this shouldn't happen if the chain initialized successfully
    }

    ROS_INFO("Gather joints in chain");
    std::vector<std::string> planning_joints;
    for (auto i = (unsigned)0; i < model->m_chain.getNrOfSegments(); ++i) {
        auto& segment = model->m_chain.getSegment(i);
        auto& child_joint_name = segment.getJoint().getName();
        auto* joint = GetJoint(&model->m_robot_model, &child_joint_name);
        if (GetVariableCount(joint) > 1) {
            ROS_WARN("> 1 variable per joint.");
            return false;
        }
        if (GetVariableCount(joint) == 0) {
            continue;
        }
        planning_joints.push_back(child_joint_name);
    }

    ROS_INFO("Initialize URDF Robot Model with planning joints = %s", to_string(planning_joints).c_str());
    if (!urdf::Init(model, &model->m_robot_model, &planning_joints)) {
        ROS_ERROR("Failed to initialize URDF Robot Model");
        return false;
    }

    // do this after we've initialized the URDFRobotModel...
    model->planning_link = GetLink(&model->m_robot_model, &tip_link);
    if (model->planning_link == NULL) {
        return false; // this shouldn't happen either
    }

    // FK solver
    model->m_fk_solver = make_unique<KDL::ChainFkSolverPos_recursive>(model->m_chain);

    // IK Velocity solver
    //model->m_ik_vel_solver = make_unique<KDL::ChainIkSolverVel_pinv>(model->m_chain);

    // IK solver
    model->m_max_iterations = 200;

    model->m_jnt_pos_in.resize(model->m_chain.getNrOfJoints());
    model->m_jnt_pos_out.resize(model->m_chain.getNrOfJoints());
    model->m_free_angle = free_angle;
    model->m_search_discretization = 0.02;
    model->m_timeout = 0.05;

    return true;
}
}

bool TracIKRobotModel::init(
        const std::string& _robot_description,
        const std::string& _base_link,
        const std::string& _tip_link,
        int _free_angle){
    return Init(this, _robot_description, _base_link, _tip_link, _free_angle);
}

auto TracIKRobotModel::getBaseLink() const -> const std::string& {
    return m_base_link;
}

auto TracIKRobotModel::getPlanningLink() const -> const std::string&
{
    return m_tip_link;
}

static
void NormalizeAngles(TracIKRobotModel* model, KDL::JntArray* q)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        if (model->vprops[i].continuous) {
            (*q)(i) = smpl::angles::normalize_angle((*q)(i));
        }
    }
}

static
double GetSolverMinPosition(TracIKRobotModel* model, int vidx)
{
    if (model->vprops[vidx].continuous) {
        return -M_PI;
    } else {
        return model->vprops[vidx].min_position;
    }
}

bool TracIKRobotModel::computeIKSearch(
    const Eigen::Affine3d& _pose,
    const RobotState& _start,
    RobotState& _solution){
    // transform into kinematics and convert to kdl
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * _pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < _start.size(); i++) {
        m_jnt_pos_in(i) = _start[i];
    }

    // must be normalized for CartToJntSearch
    NormalizeAngles(this, &m_jnt_pos_in);

    auto initial_guess = m_jnt_pos_in(m_free_angle);

    auto start_time = smpl::clock::now();
    auto loop_time = 0.0;
    auto count = 0;

    auto num_positive_increments =
            (int)((GetSolverMinPosition(this, m_free_angle) - initial_guess) /
                    this->m_search_discretization);
    auto num_negative_increments =
            (int)((initial_guess - GetSolverMinPosition(this, m_free_angle)) /
                    this->m_search_discretization);

    while (loop_time < this->m_timeout) {
        if (m_ik_solver->CartToJnt(m_jnt_pos_in, frame_des, m_jnt_pos_out) >= 0) {
            NormalizeAngles(this, &m_jnt_pos_out);
            _solution.resize(_start.size());
            for (size_t i = 0; i < _solution.size(); ++i) {
                _solution[i] = m_jnt_pos_out(i);
            }
            return true;
        }
        if (!getCount(count, num_positive_increments, -num_negative_increments)) {
            return false;
        }
        m_jnt_pos_in(m_free_angle) = initial_guess + this->m_search_discretization * count;
        ROS_DEBUG("%d, %f", count, m_jnt_pos_in(m_free_angle));
        loop_time = to_seconds(smpl::clock::now() - start_time);
    }

    if (loop_time >= this->m_timeout) {
        ROS_DEBUG("IK Timed out in %f seconds", this->m_timeout);
        return false;
    } else {
        ROS_DEBUG("No IK solution was found");
        return false;
    }
    return false;
}

bool TracIKRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    if (option != ik_option::UNRESTRICTED) {
        return false;
    }

    return computeIKSearch(pose, start, solution);
}

bool TracIKRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    // NOTE: only returns one solution
    RobotState solution;
    if (computeIK(pose, start, solution)) {
        solutions.push_back(solution);
    }
    return solutions.size() > 0;
}

bool TracIKRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution)
{
    //transform into kinematics frame and convert to kdl
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    //seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        m_jnt_pos_in(i) = start[i];
    }

    //must be normalized for CartToJntSearch
    NormalizeAngles(this, &m_jnt_pos_in);

    if (m_ik_solver->CartToJnt(m_jnt_pos_in, frame_des, m_jnt_pos_out) < 0) {
        return false;
    }

    NormalizeAngles(this, &m_jnt_pos_out);

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = m_jnt_pos_out(i);
    }

    return true;
}

void TracIKRobotModel::printRobotModelInformation()
{
    leatherman::printKDLChain(m_chain, "robot_model");
}

auto TracIKRobotModel::getExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<InverseKinematicsInterface>()) return this;
    return URDFRobotModel::getExtension(class_code);
}

}
