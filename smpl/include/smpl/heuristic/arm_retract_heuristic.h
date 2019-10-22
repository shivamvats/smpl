////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_ARM_RETRACT_HEURISTIC_H
#define SMPL_ARM_RETRACT_HEURISTIC_H

// project includes
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/spatial.h>

namespace smpl {

class ArmRetractHeuristic : public RobotHeuristic
{
public:

    bool init(RobotPlanningSpace* space);

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

    static constexpr double FIXED_POINT_RATIO = 1000.0;

    PoseProjectionExtension* m_pose_ext = nullptr;
    PointProjectionExtension* m_point_ext = nullptr;

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
