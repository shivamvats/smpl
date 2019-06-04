#ifndef SMPL_OMPL_INTERFACE_OMPL_INTERFACE_H
#define SMPL_OMPL_INTERFACE_OMPL_INTERFACE_H

// standard includes
#include <memory>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <ompl/base/Planner.h>

#if OMPL_VERSION_VALUE >= 1004000  // Version greater than 1.4.0
typedef Eigen::VectorXd OMPLProjection;
#else  // All other versions
typedef ompl::base::EuclideanProjection OMPLProjection;
#endif

namespace smpl {

class OccupancyGrid;

namespace visual {
struct Marker;
} // namespace visual

namespace detail {
struct PlannerImpl;
} // namespace detail

struct PoseGoal : public ompl::base::Goal
{
    Eigen::Affine3d pose;
    Eigen::Vector3d position_tolerance;
    Eigen::Vector3d orientation_tolerance;

    PoseGoal(
        const ompl::base::SpaceInformationPtr& si,
        const Eigen::Affine3d& pose = Eigen::Affine3d::Identity());

    bool isSatisfied(const ompl::base::State* state) const override;
};

struct OMPLPlanner : public ompl::base::Planner
{
    std::unique_ptr<detail::PlannerImpl> m_impl;

    /// \param planner_id
    OMPLPlanner(
        const ompl::base::SpaceInformationPtr& si,
        const std::string& planner_id = std::string(),
        OccupancyGrid* grid = NULL);

    ~OMPLPlanner();

    using VisualizerFun = std::function<
            std::vector<smpl::visual::Marker>(const std::vector<double>& state)>;
    void setStateVisualizer(const VisualizerFun& fun);

    void setOccupancyGrid(OccupancyGrid* grid);

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

    auto solve(const ompl::base::PlannerTerminationCondition& ptc)
        -> ompl::base::PlannerStatus override;

    void clear() override;

    void checkValidity() override;

    void setup() override;

    void getPlannerData(ompl::base::PlannerData& data) const override;

    friend struct detail::PlannerImpl;
};

auto MakeStateSMPL(
    const ompl::base::StateSpace* space,
    const ompl::base::State* state)
    -> std::vector<double>;

auto MakeStateOMPL(
    const ompl::base::StateSpace* space,
    const std::vector<double>& state)
    -> ompl::base::State*;

auto MakeStateOMPL(
    const ompl::base::StateSpacePtr& space,
    const std::vector<double>& state)
    -> ompl::base::ScopedState<>;

} // namespace smpl

#endif

