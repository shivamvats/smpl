#include <smpl/heuristic/base_rot_euclidean_heuristic.h>

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

bool BaseRotEuclideanHeuristic::init(RobotPlanningSpace* space, double angle )
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
        SMPL_WARN_NAMED(LOG, "BaseRotEuclideanHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }

    m_extract_ext = planningSpace()->getExtension<ExtractRobotStateExtension>();
    if(m_extract_ext)
        SMPL_INFO_NAMED(LOG, "Got Extract State Extension!");
    else
        return false;

    return true;
}

void BaseRotEuclideanHeuristic::setWeightX(double wx)
{
    m_x_coeff = wx;
}

void BaseRotEuclideanHeuristic::setWeightY(double wy)
{
    m_y_coeff = wy;
}

void BaseRotEuclideanHeuristic::setWeightZ(double wz)
{
    m_z_coeff = wz;
}

void BaseRotEuclideanHeuristic::setWeightRot(double wr)
{
    m_rot_coeff = wr;
}

double BaseRotEuclideanHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

double BaseRotEuclideanHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* BaseRotEuclideanHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int BaseRotEuclideanHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (m_extract_ext) {
        RobotState robot_state = m_extract_ext->extractState(state_id);
        double new_orientation = normalize_angle(robot_state[2] + m_rot_angle);
        robot_state[2] = new_orientation;

        Affine3 p;
        m_pose_ext->projectToPose(robot_state, p);

        auto& goal_pose = planningSpace()->goal().pose;

        const double dist = computeDistance(p, goal_pose);

        const int h = FIXED_POINT_RATIO * dist;
        return h;
    } else {
        return 0;
    }
}

int BaseRotEuclideanHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int BaseRotEuclideanHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

double BaseRotEuclideanHeuristic::computeDistance(
    const Affine3& a,
    const Affine3& b) const
{
    auto sqrd = [](double d) { return d * d; };

    Vector3 diff = b.translation() - a.translation();

    double dp2 =
            m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());

    Quaternion qb(b.rotation());
    Quaternion qa(a.rotation());

    double dot = qa.dot(qb);
    if (dot < 0.0) {
        qb = Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
        dot = qa.dot(qb);
    }

    double dr2 = angles::normalize_angle(2.0 * std::acos(dot));
    dr2 *= (m_rot_coeff * dr2);

    SMPL_DEBUG_NAMED(LOG, "Compute Distance: sqrt(%f + %f)", dp2, dr2);

    return std::sqrt(dp2 + dr2);
}

} // namespace smpl
