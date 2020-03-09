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

#ifndef sbpl_collision_collision_operations_h
#define sbpl_collision_collision_operations_h

// system includes
#include <ros/console.h>
#include <smpl/occupancy_grid.h>

// project includes
#include <sbpl_collision_checking/robot_collision_state.h>

namespace smpl {
namespace collision {

bool CheckSphereCollision(
    const OccupancyGrid& grid,
    const CollisionSphereState& s,
    double padding,
    double& dist);

double SphereCollisionDistance(
    const OccupancyGrid& grid,
    const CollisionSphereState& s,
    double padding);

template <typename StateType>
bool CheckVoxelsCollisions(
    StateType& state,
    std::vector<const CollisionSphereState*>& q,
    const OccupancyGrid* grid,
    double padding,
    double& dist);

static const char* COP_LOGGER = "collision_operations";

/// Check a single sphere against an occupancy grid
inline
bool CheckSphereCollision(
    const OccupancyGrid& grid,
    const CollisionSphereState& s,
    double padding,
    double& dist)
{
    const double effective_radius = s.model->radius + padding;
    dist = grid.getSquaredDist(s.pos.x(), s.pos.y(), s.pos.z());
    return dist >= effective_radius * effective_radius;
}

/// Compute the closest distance between a sphere and an occupied voxel
inline
double SphereCollisionDistance(
    const OccupancyGrid& grid,
    const CollisionSphereState& s,
    double padding)
{
    double dist = grid.getDistanceFromPoint(s.pos.x(), s.pos.y(), s.pos.z());
    const double effective_radius = s.model->radius + padding;
    return dist - effective_radius;
}

std::vector<SphereIndex> GatherSphereIndices(
    const RobotCollisionState& state, int gidx);

/// Check sphere hierarchies for collisions against an occupancy grid
///
/// \param state The aggregate state of the collision trees. Must have a method
///     updateSphereState(const SphereIndex&)
/// \param q A queue for maintaining the list of remaining spheres to check,
///     preseeded with the roots of all collision sphere trees to check
/// \param grid The distance map to check spheres against
/// \param padding Padding to be applied to each sphere
/// \param dist The distance to the occupancy grid that caused the check to
///     fail, if any
template <typename StateType>
bool CheckVoxelsCollisions(
    StateType& state,
    std::vector<const CollisionSphereState*>& q,
    const OccupancyGrid& grid,
    double padding,
    double& dist)
{
    while (!q.empty()) {
        const CollisionSphereState* s = q.back();
        q.pop_back();

        if (s->parent_state->index != -1) {
            state.updateSphereState(SphereIndex(s->parent_state->index, s->index()));
        }

        ROS_DEBUG_NAMED(COP_LOGGER, "Checking sphere '%s' with radius %0.3f at (%0.3f, %0.3f, %0.3f)", s->model->name.c_str(), s->model->radius, s->pos.x(), s->pos.y(), s->pos.z());

        double obs_dist;
        if (CheckSphereCollision(grid, *s, padding, obs_dist)) {
            ROS_DEBUG_NAMED(COP_LOGGER, " dist^2: %0.3f -> ok!", obs_dist);
            continue; // no collision -> ok!
        }
        //ROS_ERROR(" dist^2: %0.3f -> collision", obs_dist);

        if (s->isLeaf()) {
            if (s->parent_state->index == -1) { // meta-leaf
                const CollisionSphereState* sl = s->left->left;
                const CollisionSphereState* sr = s->right->right;

                if (sl && sr) {
                    if (sl->model->radius > sr->model->radius) {
                        q.push_back(sr);
                        q.push_back(sl);
                    } else {
                        q.push_back(sl);
                        q.push_back(sr);
                    }
                } else if (sl) {
                    q.push_back(sl);
                } else if (sr) {
                    q.push_back(sr);
                }
            } else { // normal leaf
                const CollisionSphereModel* sm = s->model;
                dist = obs_dist;
                ROS_DEBUG_NAMED(COP_LOGGER, "    *collision* name: %s, pos: (%0.3f, %0.3f, %0.3f), radius: %0.3fm, dist: %0.3fm", sm->name.c_str(), s->pos.x(), s->pos.y(), s->pos.z(), sm->radius, obs_dist);
                return false;
            }
        } else { // recurse on both children
            if (s->left->model->radius > s->right->model->radius) {
                q.push_back(s->right);
                q.push_back(s->left);
            } else {
                q.push_back(s->left);
                q.push_back(s->right);
            }
        }
    }

    ROS_DEBUG_NAMED(COP_LOGGER, "No voxels collisions");
    return true;
}

} // namespace collision
} // namespace smpl

#endif
