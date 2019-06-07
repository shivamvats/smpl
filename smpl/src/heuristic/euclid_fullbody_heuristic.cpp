#include <smpl/heuristic/euclid_fullbody_heuristic.h>

// standard includes
#include <cmath>
#include <chrono>
#include <thread>
#include <math.h>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
//#include <smpl_urdf_robot_model/robot_state.h>
//#include <smpl_urdf_robot_model/urdf_robot_model.h>
//#include <sbpl_kdl_robot_model/kdl_robot_model.h>

namespace smpl {

static const char* LOG = "heuristic.euclid_fullbody";

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

bool EuclidFullbodyHeuristic::init(RobotPlanningSpace* space)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_point_ext = space->getExtension<PointProjectionExtension>();
    if (m_point_ext) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_pose_ext = space->getExtension<PoseProjectionExtension>();
    if (m_pose_ext) {
        SMPL_INFO_NAMED(LOG, "Got Pose Projection Extension!");
    }
    if (!m_pose_ext && !m_point_ext) {
        SMPL_WARN_NAMED(LOG, "EuclidFullbodyHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }

    return true;
}

void EuclidFullbodyHeuristic::updateGoal(const GoalConstraint& goal) {
    Affine3 goal_pose = goal.pose;
    double goal_x = goal_pose.translation()[0];
    double goal_y = goal_pose.translation()[1];
    double base_x=0, base_y=0;
    double arm_length = 0.60;
    double robot_yaw_delta = 60 * 3.14/180;

    bool found_base = false;

    for (int i=0; i<4; i++) {
        double theta = i*0.4;
        double possible_x = goal_x + arm_length*cos(theta);
        double possible_y = goal_y + arm_length*sin(theta);
        double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x) + robot_yaw_delta;
        RobotState possible_state(planningSpace()->robot()->jointCount(), 0);
        possible_state[0] = possible_x;
        possible_state[1] = possible_y;
        possible_state[2] = possible_yaw;

        std::vector<double> heuristic_base_pose;
        if (planningSpace()->collisionChecker()->isStateValid(possible_state)) {
            heuristic_base_pose.push_back(possible_x);
            heuristic_base_pose.push_back(possible_y);
            heuristic_base_pose.push_back(possible_yaw);
            m_heuristic_base_poses.push_back(heuristic_base_pose);
            found_base = true;
        }
    }
    if (!found_base) {
        ROS_ERROR("Could not find a valid base state.");
    }
    else {
        ROS_ERROR( "%d Valid base pose found.", m_heuristic_base_poses.size() );
    }

    //KDL::Chain chain;
    //KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    //bool valid = planningSpace()->m_tracik_solver_ptr->getKDLChain(chain);

    //if (!valid)
    //{
    //    ROS_ERROR("There was no valid KDL chain found");
    //    return;
    //}

    //valid = planningSpace()->m_tracik_solver_ptr->getKDLLimits(ll, ul);

    //if (!valid)
    //{
    //    ROS_ERROR("There were no valid KDL joint limits found");
    //    return;
    //}

}


void EuclidFullbodyHeuristic::setWeightX(double wx)
{
    m_x_coeff = wx;
}

void EuclidFullbodyHeuristic::setWeightY(double wy)
{
    m_y_coeff = wy;
}

void EuclidFullbodyHeuristic::setWeightZ(double wz)
{
    m_z_coeff = wz;
}

void EuclidFullbodyHeuristic::setWeightRot(double wr)
{
    m_rot_coeff = wr;
}

double EuclidFullbodyHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    auto& goal_pose = planningSpace()->goal().pose;
    return EuclideanDistance(
            x, y, z,
            goal_pose.translation()[0],
            goal_pose.translation()[1],
            goal_pose.translation()[2]);
}

double EuclidFullbodyHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* EuclidFullbodyHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

// XXX TODO
int EuclidFullbodyHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (m_pose_ext) {
        Affine3 p;
        if (!m_pose_ext->projectToPose(state_id, p)) {
            return 0;
        }

        auto& goal_pose = planningSpace()->goal().pose;

        // IK
        /*
        for( int i=0; i<m_heuristic_base_poses.size(); i++ ){
            // Set reference state in the robot planning model...
            //
            urdf::RobotState* reference_state = &dynamic_cast<urdf::URDFRobotModel*>(planningSpace()->robot())->robot_state;
            ROS_INFO("x, y of ref state: %d, %d\n", reference_state->positions[0], reference_state->positions[1]);
            reference_state->positions[0] = m_heuristic_base_poses[i][0];
            reference_state->positions[1] = m_heuristic_base_poses[i][1];
            reference_state->positions[2] = m_heuristic_base_poses[i][2];
            ROS_INFO("Updated x, y of ref state: %d, %d\n", reference_state->positions[0], reference_state->positions[1]);

            urdf::SetReferenceState(dynamic_cast<urdf::URDFRobotModel*>(planningSpace()->robot()), GetVariablePositions(reference_state));

            RobotState ik_soltn;
            dynamic_cast<KDLRobotModel*>( planningSpace()->robot() )->computeTracIKSearch( goal_pose, ik_soltn );
        }
        */

        auto goal_xyz = goal_pose.translation();
        auto p_xyz = p.translation();

        double Y, P, R;
        angles::get_euler_zyx(p.rotation(), Y, P, R);

        auto angle_p_goal = atan2(goal_xyz.y() - p_xyz.y(), goal_xyz.x() - p_xyz.x());
        // Rotate towards goal.
        auto dist_rot1 = fabs(shortest_angle_dist(Y, angle_p_goal));

        SMPL_DEBUG_NAMED(LOG, "p.Y: %f, angle_p_goal: %f, dist_rot1: %f", Y, angle_p_goal, dist_rot1);
        auto target_base = m_heuristic_base_poses[0];
        double base_dist = std::sqrt( (p_xyz.x() - target_base[0])*(p_xyz.x() - target_base[0]) + (p_xyz.y() - target_base[1])*(p_xyz.y() - target_base[1])) + fabs(shortest_angle_dist( target_base[2], Y));

        // Move to goal xyz.
        auto dist_xyz = computeDistance(p_xyz, goal_xyz);

        angles::get_euler_zyx(goal_pose.rotation(), Y, P, R);
        // Achieve goal yaw.
        auto dist_rot2 = fabs(shortest_angle_dist(angle_p_goal, Y));

        auto dist = std::sqrt(m_rot_coeff*dist_rot1*dist_rot1 + dist_xyz);// + m_rot_coeff*dist_rot2;
        //auto dist = std::sqrt(dist_xyz);

        const int h = FIXED_POINT_RATIO * (1.5*base_dist + dist);
        angles::get_euler_zyx(p.rotation(), Y, P, R);
        SMPL_DEBUG_NAMED(LOG, "h(%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) = coeff*%f + coeff*%f = %d", p.translation()[0], p.translation()[1], p.translation()[2], Y, P, R, dist_rot1, dist_xyz, h);

        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return h;
    } else if (m_point_ext) {
        Vector3 p;
        if (!m_point_ext->projectToPoint(state_id, p)) {
            return 0;
        }

        auto& goal_pose = planningSpace()->goal().pose;
        Vector3 gp(goal_pose.translation());

        double dist = computeDistance(p, gp);

        const int h = FIXED_POINT_RATIO * dist;
        SMPL_DEBUG_NAMED(LOG, "h(%d) = %d", state_id, h);
        return h;
    } else {
        return 0;
    }
}

int EuclidFullbodyHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int EuclidFullbodyHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

Affine3 EuclidFullbodyHeuristic::createPose(
    const std::vector<double> &pose) const
{
    return createPose(pose[0], pose[1], pose[2], pose[5], pose[4], pose[3]);
}

Vector3 EuclidFullbodyHeuristic::createPoint(
    const std::vector<double>& point) const
{
    return Vector3(point[0], point[1], point[2]);
}

Affine3 EuclidFullbodyHeuristic::createPose(
    double x, double y, double z,
    double Y, double P, double R) const
{
    return Affine3(
            Translation3(x, y, z) *
            AngleAxis(Y, Vector3::UnitZ()) *
            AngleAxis(P, Vector3::UnitY()) *
            AngleAxis(R, Vector3::UnitX()));
}

double EuclidFullbodyHeuristic::computeDistance(
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

double EuclidFullbodyHeuristic::computeDistance(
    const Vector3& u,
    const Vector3& v) const
{
    auto sqrd = [](double d) { return d * d; };
    Vector3 diff = v - u;
    return m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());
}

} // namespace smpl
