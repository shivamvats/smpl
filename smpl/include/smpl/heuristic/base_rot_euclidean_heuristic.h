#ifndef SMPL_BASE_ROT_EUCLIDEAN_HEURISTIC_H
#define SMPL_BASE_ROT_EUCLIDEAN_HEURISTIC_H

// project includes
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/spatial.h>

namespace smpl {

// Rotate by ``theta`` and then compute Euclidean.
class BaseRotEuclideanHeuristic : public RobotHeuristic
{
public:

    bool init(RobotPlanningSpace* space, double angle);

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

private:

    double m_x_coeff = 1.0;
    double m_y_coeff = 1.0;
    double m_z_coeff = 1.0;
    double m_rot_coeff = 1.0;

    static constexpr double FIXED_POINT_RATIO = 1000.0;
    double m_rot_angle;

    PoseProjectionExtension* m_pose_ext = nullptr;
    PointProjectionExtension* m_point_ext = nullptr;
    ExtractRobotStateExtension* m_extract_ext = nullptr;

    double computeDistance(const Affine3& a, const Affine3& b) const;
};

} // namespace smpl

#endif
