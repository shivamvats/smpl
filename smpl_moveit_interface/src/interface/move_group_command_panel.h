#ifndef sbpl_interface_move_group_command_panel_h
#define sbpl_interface_move_group_command_panel_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#ifndef Q_MOC_RUN
#include <moveit/robot_model/robot_model.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <visualization_msgs/MarkerArray.h>
#endif

#include <smpl_moveit_interface/interface/ik_command_interactive_marker.h>
#include <smpl_moveit_interface/interface/teleop_command.h>

#include "move_group_command_model.h"

class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QString;
class QVBoxLayout;
class QWidget;

namespace sbpl_interface {

class JointVariableCommandWidget;
class IKCommandInteractiveMarker;

class MoveGroupCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    MoveGroupCommandPanel(QWidget* parent = 0);
    ~MoveGroupCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    /// \brief Load the robot using the ROS param in the Robot Description line
    ///     edit box
    void loadRobot();

    /// \brief Update the GUI and visualizations when a new robot is loaded
    void updateRobot();

    /// \brief Update the workspace frame selections when new transforms become
    ///     available.
    void updateTransforms();

    void syncRobot();
    void syncModelConfig();

    /// \brief Update the robot visualization to reflect the state of the robot
    void updateRobotVisualization();

    void planToGoalPose();
    void moveToGoalPose();
    void planToGoalConfiguration();
    void moveToGoalConfiguration();
    void copyCurrentState();

    void setGoalJointTolerance(double tol_deg);
    void setGoalPositionTolerance(double tol_m);
    void setGoalOrientationTolerance(double tol_deg);

    void setCurrentPlanner(const QString& name);
    void setCurrentPlannerID(const QString& id);

    void setWorkspaceFrame(const QString& frame);
    void setWorkspaceMinX(double value);
    void setWorkspaceMinY(double value);
    void setWorkspaceMinZ(double value);
    void setWorkspaceMaxX(double value);
    void setWorkspaceMaxY(double value);
    void setWorkspaceMaxZ(double value);

private:

    ros::NodeHandle m_nh;

    MoveGroupCommandModel m_model;

    /// \name General Settings Widgets
    ///@{
    QLineEdit* m_robot_description_line_edit    = nullptr;
    QPushButton* m_load_robot_button            = nullptr;
    ///@}

    /// \name Planner Settings Widgets
    ///@{
    QComboBox* m_planner_name_combo_box             = nullptr;
    QComboBox* m_planner_id_combo_box               = nullptr;
    QSpinBox* m_num_planning_attempts_spinbox       = nullptr;
    QDoubleSpinBox* m_allowed_planning_time_spinbox = nullptr;
    ///@}

    /// \name Goal Constraints Widgets - No Robot Required
    ///@{
    QGroupBox* m_goal_constraints_group = nullptr;

    QDoubleSpinBox* m_joint_tol_spinbox = nullptr;
    QDoubleSpinBox* m_pos_tol_spinbox   = nullptr;
    QDoubleSpinBox* m_rot_tol_spinbox   = nullptr;

    QComboBox* m_workspace_frame_combo_box      = nullptr;
    QDoubleSpinBox* m_workspace_min_x_spinbox   = nullptr;
    QDoubleSpinBox* m_workspace_min_y_spinbox   = nullptr;
    QDoubleSpinBox* m_workspace_min_z_spinbox   = nullptr;
    QDoubleSpinBox* m_workspace_max_x_spinbox   = nullptr;
    QDoubleSpinBox* m_workspace_max_y_spinbox   = nullptr;
    QDoubleSpinBox* m_workspace_max_z_spinbox   = nullptr;
    ///@}

    /// \name Goal Constraints Widgets - Robot Required
    ///@{
    JointVariableCommandWidget* m_var_cmd_widget    = nullptr;

    IKCommandInteractiveMarker  m_ik_cmd_marker;
    TeleopCommand               m_teleop_command;
    ///@}

    /// \name Command Widgets
    ///@{
    QPushButton* m_plan_to_position_button      = nullptr;
    QPushButton* m_move_to_position_button      = nullptr;
    QPushButton* m_plan_to_configuration_button = nullptr;
    QPushButton* m_move_to_configuration_button = nullptr;
    QPushButton* m_copy_current_state_button    = nullptr;
    ///@}

    ros::Publisher m_marker_pub;

    /// \brief Setup the baseline GUI for loading robots from URDF parameter
    void MakeGUI();
    void InitPostModel();

    auto setupGeneralSettingsGroup() -> QGroupBox*;
    auto setupPlannerSettingsGroup() -> QGroupBox*;
    auto setupGoalConstraintsGroup() -> QGroupBox*;
    auto setupCommandsGroup() -> QGroupBox*;

    void syncPlannerSelection();
    void syncNumPlanningAttemptsSpinBox();
    void syncAllowedPlanningTimeSpinBox();
    void syncGoalPositionToleranceSpinBox();
    void syncGoalOrientationToleranceSpinBox();
    void syncGoalJointToleranceSpinBox();
    void syncWorkspaceWidgets();

    auto getWorkspaceVisualization() const -> visualization_msgs::MarkerArray;
    void getRobotCollisionMarkers(
        visualization_msgs::MarkerArray& ma,
        const moveit::core::RobotState& state,
        const std::vector<std::string>& link_names,
        bool include_attached = false) const;

    auto mainLayout() -> QVBoxLayout*;
};

} // namespace sbpl_interface

#endif
