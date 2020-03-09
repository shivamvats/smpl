////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/occupancy_grid.h>
#include <math.h>
#include <ros/ros.h>

// standard includes
#include <memory>

// project includes
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/distance_map/euclid_distance_map.h>

namespace smpl {

/// \class OccupancyGrid
///
/// OccupancyGrid is a lightweight wrapper around DistanceMapInterface, with
/// some additional functionality.
///
/// The first additional feature is the presence of a reference frame id that is
/// attached to the distance map. The reference frame is used solely for
/// preparing visualizations and does there are no active transformations
/// managed by this class.
///
/// The second additional feature is cell reference counting. Each cell may
/// count the number of times it has been inserted into the distance map. Only
/// when a cell has been removed the same number of times it has been inserted,
/// will it be removed from the distance map. The cell is only actually inserted
/// once into the distance map, regardless of the number of consecutive
/// insertions into the occupancy grid.
///
/// If cell reference counting is enabled with a distance field passed to the
/// OccupancyGrid on construction, the caller must not directly modify the
/// distance map. This may corrupt the invariant that the obstacle exists in
/// the distance map if its reference count is non-zero.
///
/// An arbitrary distance map implementation may be used with this class. If
/// none is specified, by calling the verbose constructor, an instance of
/// smpl::EuclidDistanceMap is constructed.

OccupancyGrid::OccupancyGrid()
{
    m_ref_counted = false;
    m_x_stride = 0;
    m_y_stride = 0;
}

/// Construct an Occupancy Grid.
///
/// \param size_x Dimension of the grid along the X axis, in meters
/// \param size_y Dimension of the grid along the Y axis, in meters
/// \param size_z Dimension of the grid along the Z axis, in meters
/// \param resolution Resolution of the grid, in meters
/// \param origin_x X Coordinate of origin, in meters
/// \param origin_y Y Coordinate of origin, in meters
/// \param origin_z Z Coordinate of origin, in meters
/// \param max_dist The maximum distance away from obstacles to propagate
///     the distance field, in meters
/// \param propagate_negative_distances Whether to compute signed distances to
///     nearest obstacle boundaries
/// \param ref_counted Whether to reference count cells
OccupancyGrid::OccupancyGrid(
    double size_x, double size_y, double size_z,
    double resolution,
    double origin_x, double origin_y, double origin_z,
    double max_dist,
    bool ref_counted)
:
    m_grid(std::make_shared<EuclidDistanceMap>(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            resolution,
            max_dist)),
    reference_frame_(),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->numCellsY() * m_grid->numCellsZ()),
    m_y_stride(m_grid->numCellsZ()),
    m_counts()
{
    // distance field guaranteed to be empty -> faster initialization
    if (m_ref_counted) {
        m_counts.assign(getCellCount(), 0);
    }
}

/// Construct an OccupancyGrid from an existing distance map.
///
/// \param df The distance field
/// \param ref_counted Whether to reference count cells
OccupancyGrid::OccupancyGrid(
    const std::shared_ptr<DistanceMapInterface>& df,
    bool ref_counted)
:
    m_grid(df),
    reference_frame_(),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->numCellsY() * m_grid->numCellsZ()),
    m_y_stride(m_grid->numCellsZ()),
    m_counts()
{
    initRefCounts();
}

/// Copy constructor. Constructs the Occupancy Grid with a deep copy of the
/// contents of \p o.
OccupancyGrid::OccupancyGrid(const OccupancyGrid& o) :
    m_grid(o.m_grid->clone()),
    reference_frame_(o.reference_frame_),
    m_ref_counted(o.m_ref_counted),
    m_x_stride(o.m_x_stride),
    m_y_stride(o.m_y_stride),
    m_counts(o.m_counts)
{
}

OccupancyGrid& OccupancyGrid::operator=(const OccupancyGrid& rhs)
{
    if (this != &rhs) {
        m_grid.reset(rhs.m_grid->clone());
        reference_frame_ = rhs.reference_frame_;
        m_ref_counted = rhs.m_ref_counted;
        m_x_stride = rhs.m_x_stride;
        m_y_stride = rhs.m_y_stride;
        m_counts = rhs.m_counts;
    }
    return *this;
}

/// Reset the grid, removing all obstacles setting distances to their
/// uninitialized values.
void OccupancyGrid::reset()
{
    m_grid->reset();
    if (m_ref_counted) {
        m_counts.assign(getCellCount(), 0);
    }
}

void polarToCartesian(const double r, const double theta, const double phi, double& x, double& y, double& z)
{
    x = r * std::sin(theta) * std::cos(phi);
    y = r * std::sin(theta) * std::sin(phi);
    z = r * std::cos(theta);
}

void OccupancyGrid::getRayCast(const double _root[3], const double _goal[3],
        double _dtheta, double _dr,
        std::vector<double>& _ray_cast) const
{
    // auto start_time = smpl::clock::now();
    double dtheta = _dtheta*M_PI/180;
    //double delta_dist = 0.02;
    auto goal_x = _goal[0];
    auto goal_y = _goal[1];
    auto goal_z = _goal[2];

    auto xc = _root[0];
    auto yc = _root[1];
    auto zc = _root[2];

    double start_r = sqrt(pow(goal_x - xc, 2) + pow(goal_y - yc, 2) + pow(goal_z - zc, 2));

    std::cout<<"start r "<<start_r<<"\n";

    double start_theta = atan2(goal_y - yc, goal_x - xc);
    if (start_theta < 0 ){
        start_theta += M_2_PI;
    }

    double start_phi = acos((goal_z -zc)/start_r);

    if (start_phi < 0 ){
        start_phi += M_2_PI;
    }

     std::cout<<"theta "<<start_theta<<" phi "<<start_phi<<"\n";
     std::cout<<"origin "<<xc<<" "<<yc<<" "<<zc<<"\n";
     std::cout<<"goal "<<goal_x<<" "<<goal_y<<" "<<goal_z<<"\n";
    //

    //_ray_cast.push_back(1); //to account for bias term in logistic regression
    int ray_itr = 1;
    double theta_radius = M_PI / 3;
    double phi_radius = M_PI / 2;

    for(double theta = start_theta - theta_radius; theta < start_theta + theta_radius; theta += dtheta)
    {
        for(double phi = start_phi - phi_radius; phi < start_phi + phi_radius; phi += dtheta)
        {
            //ROS_ERROR("%f, %f", theta, phi);
            double r = 0.05;
            double x, y, z;
            polarToCartesian(r, theta, phi, x, y, z);

            x = x + xc;
            y = y + yc;
            z = z + zc;

            double ray_dist = sqrt(pow(xc - x, 2) + pow(yc - y, 2) + pow(zc - z, 2));
            while(getDistanceFromPoint(x, y, z) > 0.0001 && ray_dist <= start_r)
            {
                //std::cout << getDistanceFromPoint(x,y,z) << " ";
                r += _dr;
                polarToCartesian(r, theta, phi, x, y, z);
                x = x + xc;
                y = y + yc;
                z = z + zc;
                ray_dist = sqrt(pow(xc - x, 2) + pow(yc - y, 2) + pow(zc - z, 2));
                //ROS_ERROR("Ray dist: %f vs %f", ray_dist, start_r);
            }

            // std::cout<<"ray end "<<x<<" "<<y<<" "<<z<<"\n";
            // std::cout<<"goal_rel "<<goal_x-xc<<" "<<goal_y-yc<<" "<<goal_z-zc<<"\n";
            // std::cout<<"rel ray end "<<x-xc<<" "<<y-yc<<" "<<z-zc<<"\n";

            //ray_dist = sqrt(pow(xc - x, 2) + pow(yc - y, 2) + pow(zc - z, 2));

            _ray_cast.push_back(ray_dist);
            //if(ray_dist < start_r)
            //{
                ////_ray_cast[ray_itr] = 0;
                //_ray_cast.push_back(0.0);
            //} else
            //{
                ////_ray_cast[ray_itr] = 1;
                //_ray_cast.push_back(1.0);
            //}
            //ROS_WARN("ray cast val: %f, %f, %d", ray_dist, start_r, _ray_cast.back());

            ray_itr++;
        }
    }

    // clock::duration elapsed_secs = clock::now()-start_time;
    // std::cout<<"Ray Tracing Time "<<to_seconds(elapsed_secs)<<"\n";
}

/// Count the number of obstacles in the occupancy grid.
size_t OccupancyGrid::getOccupiedVoxelCount() const
{
    size_t count = 0;
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            ++count;
        }
    });
    return count;
}

/// Get all occupied voxels within an oriented cube region of the grid.
void OccupancyGrid::getOccupiedVoxels(
    const Affine3& pose,
    const std::vector<double>& dim,
    std::vector<Vector3>& voxels) const
{
    for (double x = -0.5 * dim[0]; x <= 0.5 * dim[0]; x += m_grid->resolution()) {
    for (double y = -0.5 * dim[1]; y <= 0.5 * dim[1]; y += m_grid->resolution()) {
    for (double z = -0.5 * dim[2]; z <= 0.5 * dim[2]; z += m_grid->resolution()) {
        Vector3 point = pose * Vector3(x, y, z);

        if (getDistanceFromPoint(point.x(), point.y(), point.z()) <= 0.0) {
            voxels.push_back(point);
        }
    }
    }
    }
}

/// Get all occupied voxels within an axis-aligned cube region of the grid.
void OccupancyGrid::getOccupiedVoxels(
    double x_center,
    double y_center,
    double z_center,
    double radius,
    std::vector<Vector3>& voxels) const
{
    int x_c, y_c, z_c;
    worldToGrid(x_center, y_center, z_center, x_c, y_c, z_c);
    int radius_c = radius / resolution() + 0.5;

    Vector3 v;

    iterateCells(
            x_c - radius_c, y_c - radius_c, z_c - radius_c,
            x_c + radius_c, y_c + radius_c, z_c + radius_c,
            [&](int x, int y, int z)
            {
                if (getDistanceFromPoint(x, y, z) == 0.0) {
                    gridToWorld(x, y, z, v.x(), v.y(), v.z());
                    voxels.push_back(v);
                }
            });
}

/// Gather all the obstacle points in the occupancy grid.
void OccupancyGrid::getOccupiedVoxels(
    std::vector<Vector3>& voxels) const
{
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            double wx, wy, wz;
            m_grid->gridToWorld(x, y, z, wx, wy, wz);
            voxels.emplace_back(wx, wy, wz);
        }
    });
}

/// Return a visualization of the bounding box of the distance map.
auto OccupancyGrid::getBoundingBoxVisualization() const -> visual::Marker
{
    std::vector<Vector3> points;
    points.reserve(
            4 * (m_grid->numCellsX() + 2) +
            4 * (m_grid->numCellsY()) +
            4 * (m_grid->numCellsZ()));

    Vector3 p;
    int x, y, z;

    for (x = -1; x <= m_grid->numCellsX(); ++x) {
        y = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = m_grid->numCellsY(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = -1, z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = m_grid->numCellsY(), z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);
    }

    for (y = 0; y < m_grid->numCellsY(); ++y) {
        x = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        x = m_grid->numCellsX(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        x = -1, z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        x = m_grid->numCellsX(), z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);
    }

    for (z = 0; z < m_grid->numCellsZ(); ++z) {
        y = -1, x = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = m_grid->numCellsY(), x = -1;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = -1, x = m_grid->numCellsX();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        y = m_grid->numCellsY(), x = m_grid->numCellsX();
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);
    }

    return MakeCubesMarker(
            std::move(points),
            resolution(),
            smpl::visual::MakeColorHSV(10.0f),
            getReferenceFrame(),
            "collision_space_bounds");
}

/// Return a visualization of the distance values stored in the distance map.
auto OccupancyGrid::getDistanceFieldVisualization(double max_dist) const
    -> visual::Marker
{
    const double min_value = m_grid->resolution();
    const double max_value = max_dist < 0.0 ?
            m_grid->getUninitializedDistance() : max_dist;

    std::vector<Vector3> points;
    std::vector<visual::Color> colors;
    iterateCells([&](int x, int y, int z)
    {
        const double d = m_grid->getCellDistance(x, y, z);
        if (d >= min_value && d <= max_value) {
            Vector3 p;
            m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            points.push_back(p);

            const double alpha = (d - min_value) / (max_value - min_value);
            colors.push_back(visual::MakeColorHSV(120.0 + alpha * (240.0)));
        }
    });

    return MakeCubesMarker(
            std::move(points),
            0.5 * m_grid->resolution(),
            std::move(colors),
            reference_frame_,
            "distance_field");
}

/// Return a visualization of the obstacle cells stored in the distance map.
auto OccupancyGrid::getOccupiedVoxelsVisualization() const -> visual::Marker
{
    std::vector<Vector3> voxels;
    getOccupiedVoxels(voxels);

    return MakeCubesMarker(
            std::move(voxels),
            m_grid->resolution(),
            //visual::Color{ 0.8f, 0.3f, 0.5f, 1.0f },
            visual::Color{ 1.0f, 1.0f, 1.0f, 1.0f },
            getReferenceFrame(),
            "occupied_voxels");
}

/// Add a set of obstacle cells to the occupancy grid.
void OccupancyGrid::addPointsToField(
    const std::vector<Vector3>& points)
{
    if (m_ref_counted) {
        std::vector<Vector3> pts;
        pts.reserve(points.size());
        int gx, gy, gz;
        for (const Vector3& v : points) {
            worldToGrid(v.x(), v.y(), v.z(), gx, gy, gz);

            if (isInBounds(gx, gy, gz)) {
                const int idx = coordToIndex(gx, gy, gz);

                if (m_counts[idx] == 0) {
                    pts.emplace_back(v.x(), v.y(), v.z());
                }

                ++m_counts[idx];
            }
        }
        m_grid->addPointsToMap(pts);
    }
    else {
        m_grid->addPointsToMap(points);
    }
}

/// Remove a set of obstacle cells from the occupancy grid.
void OccupancyGrid::removePointsFromField(
    const std::vector<Vector3>& points)
{
    if (m_ref_counted) {
        std::vector<Vector3> pts;
        pts.reserve(points.size());
        int gx, gy, gz;
        for (const Vector3& v : points) {
            worldToGrid(v.x(), v.y(), v.z(), gx, gy, gz);

            if (isInBounds(gx, gy, gz)) {
                int idx = coordToIndex(gx, gy, gz);

                if (m_counts[idx] > 0) {
                    --m_counts[idx];
                    if (m_counts[idx] == 0) {
                        pts.emplace_back(v.x(), v.y(), v.z());
                    }
                }
            }
        }
        m_grid->removePointsFromMap(pts);
    }
    else {
        m_grid->removePointsFromMap(points);
    }
}

/// Update the occupancy grid, removing obstacles that exist in the old obstacle
/// set, but not in the new obstacle set, and adding obstacles that exist in the
/// new obstacle set, but not in the old obstacle set.
void OccupancyGrid::updatePointsInField(
    const std::vector<Vector3>& old_points,
    const std::vector<Vector3>& new_points)
{
    // TODO: ref counting
    m_grid->updatePointsInMap(old_points, new_points);
}

void OccupancyGrid::initRefCounts()
{
    if (!m_ref_counted) {
        m_counts.clear();
        return;
    }

    int gidx = 0;
    m_counts.resize(getCellCount());
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            m_counts[gidx++] = 1;
        }
        else {
            m_counts[gidx++] = 0;
        }
    });
}

template <typename CellFunction>
void OccupancyGrid::iterateCells(CellFunction f) const
{
    for (int gx = 0; gx < m_grid->numCellsX(); ++gx) {
    for (int gy = 0; gy < m_grid->numCellsY(); ++gy) {
    for (int gz = 0; gz < m_grid->numCellsZ(); ++gz) {
        f(gx, gy, gz);
    }
    }
    }
}

template <typename CellFunction>
void OccupancyGrid::iterateCells(
    int fx, int fy, int fz,
    int tx, int ty, int tz,
    CellFunction f) const
{
    // NOTE: [x, y, z] = major -> minor indexes
    for (int x = fx; x < tx; ++x) {
    for (int y = fy; y < ty; ++y) {
    for (int z = fz; z < tz; ++z) {
        f(x, y, z);
    }
    }
    }
}

} // namespace smpl
