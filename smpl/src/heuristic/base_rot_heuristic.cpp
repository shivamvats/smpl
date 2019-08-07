#include <smpl/heuristic/base_rot_heuristic.h>

// standard includes
#include <cmath>
#include <chrono>
#include <thread>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>

namespace smpl {

static const char* LOG = "heuristic.base_rot";

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool BaseRotHeuristic::init(RobotPlanningSpace* space, double angle )
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_rot_angle = angle;

    m_point_ext = space->getExtension<PointProjectionExtension>();
    if (m_point_ext) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_pose_ext = space->getExtension<PoseProjectionExtension>();
    if (m_pose_ext) {
        SMPL_INFO_NAMED(LOG, "Got Pose Projection Extension!");
    }
    if (!m_pose_ext && !m_point_ext) {
        SMPL_WARN_NAMED(LOG, "BaseRotHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }

    m_extract_ext = planningSpace()->getExtension<ExtractRobotStateExtension>();
    if(m_extract_ext)
        SMPL_INFO_NAMED(LOG, "Got Extract State Extension!");
    else
        return false;

    return true;
}

double BaseRotHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

double BaseRotHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* BaseRotHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int BaseRotHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (m_extract_ext) {
        RobotState robot_state = m_extract_ext->extractState(state_id);
        int h = FIXED_POINT_RATIO*shortest_angle_diff( robot_state[2], m_rot_angle );
        return h;
    } else {
        return 0;
    }
}

int BaseRotHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int BaseRotHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace smpl
