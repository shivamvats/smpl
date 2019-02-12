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

#include <sbpl_collision_checking/world_collision_model.h>

// standard includes
#include <algorithm>
#include <utility>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <ros/console.h>

// project includes
#include <sbpl_collision_checking/voxel_operations.h>
#include <sbpl_collision_checking/shape_visualization.h>

namespace smpl {
namespace collision {

static const char* LOG = "world";

/// \class WorldCollisionModel
///
/// This class manages the collision representations for a set of objects in a
/// scene. Objects are represented in the collision model as the voxels they
/// occupy and the associated distance map is automatically updated when objects
/// are inserted or removed.
///
/// Storage for the objects themselves is managed externally and must be stable
/// throughout the lifetime of the WorldCollisionModel. If the object should
/// change in any way, the WorldCollisionModel should be notified accordingly
/// to remain consistent.
///
/// The collision model disallows duplicates of the same object, with uniqueness
/// determined by the object's address.

WorldCollisionModel::WorldCollisionModel(OccupancyGrid* grid) :
    m_grid(grid),
    m_padding(0.0)
{
}

WorldCollisionModel::WorldCollisionModel(
    const WorldCollisionModel& o,
    OccupancyGrid* grid)
:
    m_grid(grid),
    m_object_models(o.m_object_models),
    m_padding(o.m_padding)
{
    // TODO: check for different voxel origin/resolution/etc here...if they
    // differ, need to do a deep copy + revoxelization of the objects over just
    // a simple deep copy
    *grid = *m_grid;
}

/// Add an object to the collision model. The object will be automatically
/// rasterized and the distance map updated.
bool WorldCollisionModel::insertObject(const CollisionObject* object)
{
    if (!checkObjectInsert(object)) {
        ROS_ERROR_NAMED(LOG, "Rejecting addition of collision object '%s'", object->id.c_str());
        return false;
    }

    const double res = m_grid->resolution();
    const Eigen::Vector3d origin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());

    const Eigen::Vector3d gmin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());

    const Eigen::Vector3d gmax(
            m_grid->originX() + m_grid->sizeX(),
            m_grid->originY() + m_grid->sizeY(),
            m_grid->originZ() + m_grid->sizeZ());

    std::vector<std::vector<Eigen::Vector3d>> all_voxels;
    if (!VoxelizeObject(*object, res, origin, gmin, gmax, all_voxels)) {
        ROS_ERROR_NAMED(LOG, "Failed to voxelize object '%s'", object->id.c_str());
        return false;
    }

    {
        ObjectCollisionModel model;
        model.object = object;
        model.cached_voxels = std::move(all_voxels);
        m_object_models.push_back(std::move(model));
    }

    for (auto& voxel_list : m_object_models.back().cached_voxels) {
        ROS_DEBUG_NAMED(LOG, "Adding %zu voxels from collision object '%s' to the distance transform", voxel_list.size(), object->id.c_str());
        m_grid->addPointsToField(voxel_list);
    }

    return true;
}

/// Remove an object from the collision model. The voxels occupied by this
/// object will be cleared and the distance map updated.
bool WorldCollisionModel::removeObject(const CollisionObject* object)
{
    if (!checkObjectRemove(object)) {
        ROS_ERROR_NAMED(LOG, "Rejecting removal of collision object '%s'", object->id.c_str());
        return false;
    }

    auto* model = getObjectCollisionModel(object);
    assert(model != NULL);

    for (auto& voxel_list : model->cached_voxels) {
        ROS_DEBUG_NAMED(LOG, "Removing %zu grid cells from the distance transform", voxel_list.size());
        m_grid->removePointsFromField(voxel_list);
    }

    auto rit = std::remove_if(begin(m_object_models), end(m_object_models),
            [&](const ObjectCollisionModel& model) {
                return model.object == object;
            });
    m_object_models.erase(rit, end(m_object_models));
    return true;
}

/// Update the collision model in response to a collision object moving or
/// shapes moving with a collision object.
bool WorldCollisionModel::moveShapes(const CollisionObject* object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

/// Update the collision model in response to shapes being added to a collision
/// object.
bool WorldCollisionModel::insertShapes(const CollisionObject* object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

/// Update the collision model in response to shapes being removed from a
/// collision object.
bool WorldCollisionModel::removeShapes(const CollisionObject* object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

/// Return true if the collision model contains the object.
bool WorldCollisionModel::hasObject(const CollisionObject* object) const
{
    return haveObject(object);
}

/// Return true if any object has id $id.
bool WorldCollisionModel::hasObjectWithName(const std::string& id) const
{
    for (auto& model : m_object_models) {
        if (model.object->id == id) {
            return true;
        }
    }
    return false;
}

/// Reset the occupancy grid and distance map by removing all occupied voxels,
/// reinserting all object occupied voxels, and updating the distance map
void WorldCollisionModel::reset()
{
    m_grid->reset();
    for (auto& model : m_object_models) {
        for (auto& voxel_list : model.cached_voxels) {
            m_grid->addPointsToField(voxel_list);
        }
    }
}

/// Return a visualization of the objects in the collision model.
auto WorldCollisionModel::getWorldVisualization() const
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;
    for (auto& model : m_object_models) {
        if (model.object->shapes.size() != model.object->shape_poses.size()) {
            ROS_ERROR_NAMED(LOG, "Mismatched sizes of shapes and shape poses");
            continue;
        }

        double r, g, b;
        leatherman::HSVtoRGB(&r, &g, &b, 200.0, 1.0, 1.0);

        for (size_t i = 0; i < model.object->shapes.size(); ++i) {
            auto& shape = model.object->shapes[i];
            auto& pose = model.object->shape_poses[i];

            // fill in type and scale
            visualization_msgs::Marker m;
            if (!MakeCollisionShapeMarker(*shape, m)) {
                ROS_WARN_NAMED(LOG, "Failed to construct marker from shape");
            }

            m.header.frame_id = m_grid->getReferenceFrame();
            m.ns = model.object->id;
            m.id = (int)i;
            // m.type filled in above
            m.action = visualization_msgs::Marker::ADD;
            tf::poseEigenToMsg(pose, m.pose);
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = 1.0f;

            ma.markers.push_back(m);
        }
    }

    return ma;
}

/// Return a visualization of the object occupied voxels.
auto WorldCollisionModel::getCollisionWorldVisualization() const
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;

    std::vector<geometry_msgs::Point> voxels;
    for (auto& model : m_object_models) {
        for (size_t sidx = 0; sidx < model.object->shapes.size(); ++sidx) {
            auto& shape = model.object->shapes[sidx];
            if (shape->type != ShapeType::OcTree) {
                auto& shape_voxels = model.cached_voxels[sidx];
                voxels.reserve(voxels.size() + shape_voxels.size());
                for (auto& voxel : shape_voxels) {
                    geometry_msgs::Point point;
                    point.x = voxel.x();
                    point.y = voxel.y();
                    point.z = voxel.z();
                    voxels.push_back(point);
                }
            }
        }
    }

    visualization_msgs::Marker marker;
    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = m_grid->getReferenceFrame();
    marker.ns = "collision_object_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.scale.x = m_grid->resolution();
    marker.scale.y = m_grid->resolution();
    marker.scale.z = m_grid->resolution();
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.points.resize(voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i) {
        marker.points[i].x = voxels[i].x;
        marker.points[i].y = voxels[i].y;
        marker.points[i].z = voxels[i].z;
    }
    ma.markers.push_back(marker);

    return ma;
}

bool WorldCollisionModel::haveObject(const CollisionObject* object) const
{
    auto it = std::find_if(begin(m_object_models), end(m_object_models),
            [object](const ObjectCollisionModel& model) {
                return model.object == object;
            });
    return it != end(m_object_models);
}

auto WorldCollisionModel::getObjectCollisionModel(
    const CollisionObject* object) const
    -> const ObjectCollisionModel*
{
    for (auto& model : m_object_models) {
        if (model.object == object) {
            return &model;
        }
    }
    return NULL;
}

// Return true if the model does not already contain this object and the object
// is not malformed.
bool WorldCollisionModel::checkObjectInsert(const CollisionObject* object) const
{
    if (haveObject(object)) {
        ROS_ERROR_NAMED(LOG, "Already have collision object '%s'", object->id.c_str());
        return false;
    }

    if (object->shapes.size() != object->shape_poses.size()) {
        ROS_ERROR_NAMED(LOG, "Mismatched sizes of shapes and shape poses");
        return false;
    }

//    assert(std::find_if(begin(m_object_models), end(m_object_models),
//        [object](const ObjectCollisionModel& model) {
//            return model.object == object;
//        }) == end(m_object_models));

    return true;
}

// Return true if the model contains this object, so that it may be removed.
bool WorldCollisionModel::checkObjectRemove(const CollisionObject* object) const
{
    return haveObject(object);
}

// Return true if the model contains this object, so that it may be moved
bool WorldCollisionModel::checkObjectMoveShape(const CollisionObject* object) const
{
    return haveObject(object);
}

// Return true if the model contains this object, so that it may be updated.
bool WorldCollisionModel::checkObjectInsertShape(const CollisionObject* object) const
{
    return haveObject(object);
}

// Return true if the model contains this object, so that it may be updated.
bool WorldCollisionModel::checkObjectRemoveShape(const CollisionObject* object) const
{
    return haveObject(object);
}

void WorldCollisionModel::removeAllObjects()
{
    // while loop since m_object_models is not stable here
    while (!m_object_models.empty()) {
        bool res = removeObject(m_object_models.front().object);
        assert(res);
    }
}

} // namespace collision
} // namespace smpl
