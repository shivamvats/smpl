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

#ifndef SMPL_OCCUPANCY_GRID_H
#define SMPL_OCCUPANCY_GRID_H

// standard includes
#include <algorithm>
#include <string>
#include <vector>

// system includes
#include <Eigen/StdVector>

// project includes
#include <smpl/forward.h>
#include <smpl/debug/marker.h>
#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/spatial.h>

namespace smpl {

class OccupancyGrid
{
public:

    OccupancyGrid();

    OccupancyGrid(
        double size_x, double size_y, double size_z,
        double resolution,
        double origin_x, double origin_y, double origin_z,
        double max_dist,
        bool ref_counted = false);

    OccupancyGrid(const std::shared_ptr<DistanceMapInterface>& df, bool ref_counted = false);

    OccupancyGrid(const OccupancyGrid& o);

    OccupancyGrid(OccupancyGrid&& o) = default;

    OccupancyGrid& operator=(const OccupancyGrid& rhs);
    OccupancyGrid& operator=(OccupancyGrid&& rhs) = default;

    void getRayCast(const double root[3], const double goal[3], double dtheta, double dr, std::vector<double>& ray_cast) const;
    auto getDistanceField() const -> const std::shared_ptr<DistanceMapInterface>&
    { return m_grid; }

    /// \name Modifiers
    ///@{
    void addPointsToField(const std::vector<Vector3>& points);
    void removePointsFromField(const std::vector<Vector3>& points);

    void updatePointsInField(
        const std::vector<Vector3>& old_points,
        const std::vector<Vector3>& new_points);

    void reset();
    ///@}

    /// \name Properties
    ///@{
    double originX() const { return m_grid->originX(); }
    double originY() const { return m_grid->originY(); }
    double originZ() const { return m_grid->originZ(); }
    double sizeX() const { return m_grid->sizeX(); }
    double sizeY() const { return m_grid->sizeY(); }
    double sizeZ() const { return m_grid->sizeZ(); }
    int numCellsX() const { return m_grid->numCellsX(); }
    int numCellsY() const { return m_grid->numCellsY(); }
    int numCellsZ() const { return m_grid->numCellsZ(); }

    double resolution() const { return m_grid->resolution(); }

    const std::string& getReferenceFrame() const;
    void setReferenceFrame(const std::string& frame);
    ///@}

    /// \name Obstacle Lookups
    ///@{
    size_t getOccupiedVoxelCount() const;

    void getOccupiedVoxels(std::vector<Vector3>& voxels) const;

    void getOccupiedVoxels(
        const Affine3& pose,
        const std::vector<double>& dim,
        std::vector<Vector3>& voxels) const;

    void getOccupiedVoxels(
        double x_center,
        double y_center,
        double z_center,
        double radius,
        std::vector<Vector3>& voxels) const;
    ///@}

    /// \name Distance Lookups
    ///@{
    double getDistance(int x, int y, int z) const;

    double getDistanceFromPoint(double x, double y, double z) const;
    double getSquaredDist(double x, double y, double z) const;

    double getDistanceToBorder(int x, int y, int z) const;

    double getDistanceToBorder(double x, double y, double z) const;
    ///@}

    /// \name Conversions Between Cell and Metric Coordinates
    ///@{
    void gridToWorld(
        int x, int y, int z,
        double &wx, double &wy, double &wz) const;

    void worldToGrid(
        double wx, double wy, double wz,
        int &x, int &y, int &z) const;

    bool isInBounds(double x, double y, double z) const;
    bool isInBounds(int x, int y, int z) const;
    ///@}

    /// \name Visualization
    ///@{
    auto getBoundingBoxVisualization() const -> visual::Marker;
    auto getDistanceFieldVisualization(double max_dist = -1.0) const -> visual::Marker;
    auto getOccupiedVoxelsVisualization() const -> visual::Marker;
    ///@}

private:

    std::shared_ptr<DistanceMapInterface> m_grid;
    std::string reference_frame_;

    bool m_ref_counted;
    int m_x_stride;
    int m_y_stride;
    std::vector<int> m_counts;

    void initRefCounts();

    int coordToIndex(int x, int y, int z) const;

    int getCellCount() const;

    template <typename CellFunction>
    void iterateCells(CellFunction f) const;

    template <typename CellFunction>
    void iterateCells(
        int fx, int fy, int fz,
        int tx, int ty, int tz,
        CellFunction f) const;
};

inline
void OccupancyGrid::gridToWorld(
    int x, int y, int z,
    double& wx, double& wy, double& wz) const
{
    m_grid->gridToWorld(x, y, z, wx, wy, wz);
}

inline
void OccupancyGrid::worldToGrid(
    double wx, double wy, double wz,
    int& x, int& y, int& z) const
{
    m_grid->worldToGrid(wx, wy, wz, x, y, z);
}

inline
const std::string& OccupancyGrid::getReferenceFrame() const
{
    return reference_frame_;
}

inline
void OccupancyGrid::setReferenceFrame(const std::string& frame)
{
    reference_frame_ = frame;
}

/// Get the distance, in meters, to the nearest occupied cell.
inline
double OccupancyGrid::getDistance(int x, int y, int z) const
{
    return m_grid->getCellDistance(x, y, z);
}

/// Get the distance, in meters, to the nearest occupied cell
inline
double OccupancyGrid::getDistanceFromPoint(double x, double y, double z) const
{
    return m_grid->getMetricDistance(x, y, z);
}

inline
double OccupancyGrid::getSquaredDist(double x, double y, double z) const
{
    return m_grid->getMetricSquaredDistance(x, y, z);
}

/// Get the distance to the, in meters, to the border.
inline
double OccupancyGrid::getDistanceToBorder(int x, int y, int z) const
{
    if (!isInBounds(x, y, z)) {
        return 0.0;
    }
    int dx = std::min(m_grid->numCellsX() - x, x);
    int dy = std::min(m_grid->numCellsY() - y, y);
    int dz = std::min(m_grid->numCellsZ() - z, z);
    return m_grid->resolution() * std::min(dx, std::min(dy, dz));
}

inline
double OccupancyGrid::getDistanceToBorder(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistanceToBorder(gx, gy, gz);
}

inline
bool OccupancyGrid::isInBounds(int x, int y, int z) const
{
    return m_grid->isCellValid(x, y, z);
}

inline
bool OccupancyGrid::isInBounds(double x, double y, double z) const
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    return isInBounds(gx, gy, gz);
}

inline
int OccupancyGrid::coordToIndex(int x, int y, int z) const
{
    return x * m_x_stride + y * m_y_stride + z;
}

inline
int OccupancyGrid::getCellCount() const
{
    return m_grid->numCellsX() * m_grid->numCellsY() * m_grid->numCellsZ();
}

} // namespace smpl

#endif
