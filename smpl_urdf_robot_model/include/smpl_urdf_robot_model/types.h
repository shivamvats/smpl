#ifndef SMPL_URDF_ROBOT_MODEL_TYPES_H
#define SMPL_URDF_ROBOT_MODEL_TYPES_H

// system includes
#include <Eigen/Dense>

namespace smpl {

using Affine3 = Eigen::Affine3d;
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;
using Translation3 = Eigen::Translation3d;

} // namespace smpl

#endif
