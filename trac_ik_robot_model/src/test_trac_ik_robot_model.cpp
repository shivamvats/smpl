#include <ros/ros.h>
#include <leatherman/print.h>

// project includes
#include <trac_ik_robot_model/trac_ik_robot_model.h>

void PrintTransform(char* buff, size_t n, const Eigen::Affine3d& T)
{
    snprintf(buff, n, "[ [%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f] ]",
            T(0, 0), T(0, 1), T(0, 2), T(0, 3),
            T(1, 0), T(1, 1), T(1, 2), T(1, 3),
            T(2, 0), T(2, 1), T(2, 2), T(2, 3),
            T(3, 0), T(3, 1), T(3, 2), T(3, 3));
}

/*int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_trac_ik_robot_model");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    smpl::KDLRobotModel rm;

    std::string urdf;
    nh.param<std::string>("robot_description", urdf, "");
    if (urdf.empty()) {
        return 1;
    }

    std::vector<std::string> planning_joints;
    planning_joints.push_back("r_shoulder_pan_joint");
    planning_joints.push_back("r_elbow_flex_joint");
    planning_joints.push_back("r_shoulder_lift_joint");
    planning_joints.push_back("r_wrist_flex_joint");
    planning_joints.push_back("r_upper_arm_roll_joint");
    planning_joints.push_back("r_forearm_roll_joint");
    planning_joints.push_back("r_wrist_roll_joint");

    std::string base_link = "torso_lift_link";
    std::string tip_link = "r_gripper_palm_link";

    if (!rm.init(urdf, base_link, tip_link)) {
        ROS_ERROR("Failed to initialize the robot model");
        return 0;
    }

    ROS_WARN("Robot Model Information");
    rm.printRobotModelInformation();

    smpl::RobotState fka(planning_joints.size(), 0.0);
    fka[0] = -0.5;
    fka[1] = -0.3;
    fka[2] =  0.0;
    fka[3] = -1.0;
    fka[4] = -0.5;
    fka[5] = -0.5;
    fka[6] =  0.0;

    ROS_WARN("IK-FK Test 3 (kinematics_frame == planning_frame)");

    Eigen::Affine3d pose =
            Eigen::Translation3d(0.766268, -0.188, 0.790675) *
            Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());

    smpl::RobotState seed(planning_joints.size(), 0.0);
    smpl::RobotState ika(planning_joints.size(), 0.0);
    if (!rm.computeIK(pose, seed, ika)) {
        ROS_ERROR("Failed to compute fK");
        return 0;
    }

    char buff[256];
    PrintTransform(buff, sizeof(buff), pose);
    ROS_INFO("[ik] output_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f pose: %s",
            ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], buff);

    auto posef = rm.computeFK(ika);

    PrintTransform(buff, sizeof(buff), posef);

    ROS_INFO("[fk]  input_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f pose: %s",
            ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], buff);

    ROS_INFO("done");
    return 1;
}*/
