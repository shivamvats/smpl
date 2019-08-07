#ifndef SMPL_EUCLID_FULLBODY_HEURISTIC_H
#define SMPL_EUCLID_FULLBODY_HEURISTIC_H

// project includes
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/spatial.h>

namespace smpl {

class EuclidFullbodyHeuristic : public RobotHeuristic
{
public:

    bool init(RobotPlanningSpace* space);

    void updateGoal(const GoalConstraint&);
    void setWeightX(double wx);
    void setWeightY(double wy);
    void setWeightZ(double wz);
    void setWeightRot(double wr);

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricGoalDistance(double x, double y, double z) override;
    double getMetricStartDistance(double x, double y, double z) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

    std::vector<std::vector<double>> m_heuristic_base_poses;

private:

    static constexpr double FIXED_POINT_RATIO = 1000.0;

    PoseProjectionExtension* m_pose_ext = nullptr;
    PointProjectionExtension* m_point_ext = nullptr;
    ExtractRobotStateExtension* m_extract_ext = nullptr;

    double m_x_coeff = 1.0;
    double m_y_coeff = 1.0;
    double m_z_coeff = 1.0;
    double m_rot_coeff = 1.0;

    Affine3 createPose(const std::vector<double>& pose) const;
    Vector3 createPoint(const std::vector<double>& point) const;

    Affine3 createPose(
        double x, double y, double z,
        double Y, double P, double R) const;

    double computeDistance(const Affine3& a, const Affine3& b) const;

    double computeDistance(const Vector3& u, const Vector3& v) const;
};

} // namespace smpl

#endif
