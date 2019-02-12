#ifndef sbpl_interface_move_group_command_model_h
#define sbpl_interface_move_group_command_model_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#include <QtGui>
#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <boost/logic/tribool.hpp>
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlannerInterfaceDescription.h>
#include <moveit_msgs/ContactInformation.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <rviz/config.h>
#endif

#include <smpl_moveit_interface/interface/robot_command_model.h>

namespace sbpl_interface {

class MoveGroupCommandModel : public QObject
{
    Q_OBJECT

public:

    static constexpr double DefaultGoalPositionTolerance_m = 0.05;
    static constexpr double DefaultGoalOrientationTolerance_deg = 10.0;
    static constexpr double DefaultGoalJointTolerance_deg = 5.0;
    static const int DefaultNumPlanningAttempts = 1;
    static constexpr double DefaultAllowedPlanningTime_s = 10.0;
    static constexpr double DefaultWorkspaceMinX = -1.0;
    static constexpr double DefaultWorkspaceMinY = -1.0;
    static constexpr double DefaultWorkspaceMinZ = -1.0;
    static constexpr double DefaultWorkspaceMaxX =  1.0;
    static constexpr double DefaultWorkspaceMaxY =  1.0;
    static constexpr double DefaultWorkspaceMaxZ =  1.0;

    MoveGroupCommandModel();
    ~MoveGroupCommandModel();

    void Init();

    /// \brief Load a robot into the command model.
    ///
    /// If the robot model fails load from the given robot_description, the
    /// previous model remains and no robotLoaded signal is emitted.
    ///
    /// \param robot_description The name of the ROS parameter containing the
    ///     URDF and SRDF. The SRDF is derived from robot_description +
    ///     "_semantic".
    bool loadRobot(const std::string& robot_description);

    bool isRobotLoaded() const;

    /// \brief Return the model of the commanded robot.
    auto robotModel() const -> const moveit::core::RobotModelConstPtr&;

    /// \brief Return the state of the phantom robot used for commanding.
    auto robotState() const -> const moveit::core::RobotState*;

    auto robotStateValidity() const -> boost::tribool { return m_validity; }

    using Contacts = std::vector<moveit_msgs::ContactInformation>;
    auto contacts() const -> const Contacts&;

    bool readyToPlan() const;

    bool planToGoalPose();
    bool planToGoalConfiguration();
    bool moveToGoalPose();
    bool moveToGoalConfiguration();

    bool copyCurrentState();

    auto plannerInterfaces() const
        -> const std::vector<moveit_msgs::PlannerInterfaceDescription>&;

    auto availableFrames() const -> const std::vector<std::string>&;

    /// \name General/Robot Settings
    ///@{

    // Return the URDF string of the loaded robot, or empty if no robot is
    // loaded.
    auto robotDescription() const -> const std::string&;

    ///@}

    /// \name Planner Settings
    ///@{
    auto plannerName() const ->  std::string;
    auto plannerID() const -> std::string;
    int  numPlanningAttempts() const;
    auto allowedPlanningTime() const -> double;
    ///@}

    /// \name Goal Constraints Settings
    ///@{
    auto planningJointGroupName() const -> const std::string&;
    auto goalJointTolerance() const -> double;
    auto goalPositionTolerance() const -> double;
    auto goalOrientationTolerance() const -> double;
    auto workspace() const -> const moveit_msgs::WorkspaceParameters&;
    ///@}

    void load(const rviz::Config& config);
    void save(rviz::Config config) const;

    auto getRobotCommandModel() -> RobotCommandModel*
    { return &m_robot_command_model; }

public Q_SLOTS:

    void setPlannerName(const std::string& planner_name);
    void setPlannerID(const std::string& planner_id);
    void setNumPlanningAttempts(int num_planning_attempts);
    void setAllowedPlanningTime(double allowed_planning_time_s);
    void setPlanningJointGroup(const std::string& joint_group_name);
    void setJointVariable(int jidx, double value);
    void setJointVariable(const std::string& jv_name, double value);
    void setGoalJointTolerance(double tol_deg);
    void setGoalPositionTolerance(double tol_m);
    void setGoalOrientationTolerance(double tol_deg);
    void setWorkspace(const moveit_msgs::WorkspaceParameters& ws);
    void updatePlannerInterfaces();

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();

    /// \brief Signal that a configuration setting has been modified
    ///
    /// The following setting changes are signaled by this signal:
    /// * planner settings
    ///   * planner name
    ///   * planner id
    ///   * num planning attempts
    ///   * allowed planning time
    /// * active planning joint group
    /// * goal settings
    ///   * any goal constraint tolerance
    /// * path constraints
    ///   * workspace boundaries
    void configChanged();

    void availableFramesUpdated();

private Q_SLOTS:

    void updateRobotState();

private:

    std::string m_robot_description;
    std::unique_ptr<robot_model_loader::RobotModelLoader> m_loader;
    robot_model::RobotModelConstPtr m_robot_model;

    RobotCommandModel m_robot_command_model;

    // assertions:
    // * robot_loaded:
    //     m_scene_monitor ^ !robotDescription().empty() ^
    //     robotModel() ^ robotState()
    // * (robot loaded and model has at least one joint group) ^ active joint group is non-empty

    ros::NodeHandle m_nh;

    // publishes the current command state whenever it is updated
    ros::Publisher m_command_robot_state_pub;

    // NOTE: This replaces an old PlanningSceneMonitor. We lose the ability to
    // inspect all currently known frames...but then we don't have to deal with
    // PlanningSceneMonitor.
    std::unique_ptr<planning_scene_monitor::CurrentStateMonitor> m_state_monitor;

    boost::tribool m_validity = boost::indeterminate;
    Contacts m_contacts;

    /// \name move_group commands
    ///@{
    std::unique_ptr<ros::ServiceClient> m_check_state_validity_client;
    std::unique_ptr<ros::ServiceClient> m_query_planner_interface_client;

    using MoveGroupActionClient = actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>;
    std::unique_ptr<MoveGroupActionClient> m_move_group_client;
    ///@}

    // Set of available planning libraries and planning algorithms from each
    // library. If we have a set of valid planner interfaces, these indices
    // must reference a valid selection from this set. If we haven't yet
    // received a description of available planners, these indices may be
    // represent indices from a previous run.
    std::vector<moveit_msgs::PlannerInterfaceDescription> m_planner_interfaces;
    int m_curr_planner_idx = -1;
    int m_curr_planner_id_idx = -1;

    std::vector<std::string> m_available_frames;

    /// \name MotionPlanRequest settings
    ///@{
    double m_joint_tol_rad = 0.0;
    double m_pos_tol_m = 0.0;
    double m_rot_tol_rad = 0.0;

    moveit_msgs::WorkspaceParameters m_workspace;

    int m_num_planning_attempts = 0;
    double m_allowed_planning_time_s = 0.0;

    std::string m_curr_joint_group_name;
    ///@}

    void reinitCheckStateValidityService();

    void updateRobotStateValidity();

    bool fillWorkspaceParameters(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req);
    bool fillPoseGoalConstraints(
        const ros::Time& now,
        const std::string& group,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillConfigurationGoalConstraints(
        const ros::Time& now,
        const std::string& group,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillPathConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillTrajectoryConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;

    void logMotionPlanResponse(
        const moveit_msgs::MotionPlanResponse& res) const;
    void logMotionPlanResponse(
        const moveit_msgs::MoveGroupResult& res) const;

    bool sendMoveGroupPoseGoal(
        const std::string& group_name,
        const moveit_msgs::PlanningOptions& ops);
    bool sendMoveGroupConfigurationGoal(
        const std::string& group_name,
        const moveit_msgs::PlanningOptions& ops);

    void moveGroupResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result);

    bool plannerIndicesValid(int planner_idx, int planner_id_idx) const;

    bool hasVariable(
        const moveit::core::RobotModel& rm,
        const std::string& jv_name) const;

    bool updateAvailableFrames();

    bool computeInscribedRadius(
        const moveit::core::LinkModel& link,
        double& radius) const;

    void notifyCommandStateChanged();
};

} // namespace sbpl_interface

#endif
