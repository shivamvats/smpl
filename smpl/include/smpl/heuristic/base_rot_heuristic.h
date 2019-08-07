#ifndef SMPL_BASE_ROT_HEURISTIC_H
#define SMPL_BASE_ROT_HEURISTIC_H

// project includes
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/spatial.h>

namespace smpl {

class BaseRotHeuristic : public RobotHeuristic
{
public:

    bool init(RobotPlanningSpace* space, double angle);

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

    static constexpr double FIXED_POINT_RATIO = 1000.0;
    double m_rot_angle;

    PoseProjectionExtension* m_pose_ext = nullptr;
    PointProjectionExtension* m_point_ext = nullptr;
    ExtractRobotStateExtension* m_extract_ext = nullptr;

    double computeDistance(const Affine3& a, const Affine3& b) const;

    double computeDistance(const Vector3& u, const Vector3& v) const;
};

} // namespace smpl

#endif
