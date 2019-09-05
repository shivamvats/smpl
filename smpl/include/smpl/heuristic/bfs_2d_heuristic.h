#ifndef SMPL_BFS_2D_HEURISTIC_H
#define SMPL_BFS_2D_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/occupancy_grid.h>
#include <smpl/bfs/bfs3d.h>
#include <smpl/bfs/bfs2d.h>
#include <smpl/debug/marker.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {

class Bfs2DHeuristic : public RobotHeuristic
{
public:

    virtual ~Bfs2DHeuristic();

    bool init(RobotPlanningSpace* space, const OccupancyGrid* grid);

    double inflationRadius() const { return m_inflation_radius; }
    void setInflationRadius(double radius);
    int costPerCell() const { return m_cost_per_cell; }
    void setCostPerCell(int cost);

    auto grid() const -> const OccupancyGrid* { return m_grid; }

    auto getWallsVisualization() const -> visual::Marker;
    auto getValuesVisualization() -> visual::Marker;

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    void updateGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}
    void syncGridAndBfs();

    inline void set2DProjectionThresh(double th){m_2d_projection_thresh = th;}

    std::vector<std::vector<double>> m_heuristic_base_poses;

private:

    const OccupancyGrid* m_grid = nullptr;

    std::unique_ptr<BFS_2D> m_bfs_2d;
    PointProjectionExtension* m_pp = nullptr;

    double m_inflation_radius = 0.0;
    int m_cost_per_cell = 1;
    double m_2d_projection_thresh = 1.5;

    int getBfsCostToGoal(int x, int y) const;
};

} // namespace smpl

#endif
