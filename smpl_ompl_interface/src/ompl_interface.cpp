#include <smpl_ompl_interface/ompl_interface.h>

// system includes
#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/console/console.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>
#include <smpl/stl/memory.h>

namespace smpl {
namespace detail {

// stolen defer from scdl
template <class Callable>
struct CallOnDestruct
{
    Callable c;
    CallOnDestruct(Callable c) : c(c) { }
    ~CallOnDestruct() { c(); }
};

template <class Callable>
CallOnDestruct<Callable> MakeCallOnDestruct(Callable c) {
    return CallOnDestruct<Callable>(c);
}

// preprocessor magic to get a prefix to concatenate with the __LINE__ macro
#define MAKE_LINE_IDENT_JOIN_(a, b) a##b
#define MAKE_LINE_IDENT_JOIN(a, b) MAKE_LINE_IDENT_JOIN_(a, b)
#define MAKE_LINE_IDENT(prefix) MAKE_LINE_IDENT_JOIN(prefix, __LINE__)

// create an obscurely named CallOnDestruct with an anonymous lambda that
// executes the given statement sequence
#define DEFER(fun) auto MAKE_LINE_IDENT(tmp_call_on_destruct_) = ::smpl::detail::MakeCallOnDestruct([&](){ fun; })

///////////////////////////////
// RobotModel Implementation //
///////////////////////////////

struct RobotModel :
    public virtual smpl::RobotModel,
    public smpl::ForwardKinematicsInterface
{
    struct VariableProperties
    {
        double min_position = 0.0;
        double max_position = 0.0;

        enum VariableFlags {
            CONTINUOUS = 1,
            BOUNDED = 2
        };
        uint8_t flags = 0;

        double max_velocity = 0.0;
        double max_acceleration = 0.0;
    };

    std::vector<VariableProperties> variables;
    ompl::base::SpaceInformation* si = NULL;
    ompl::base::ProjectionEvaluator* projection = NULL;

    double minPosLimit(int vidx) const override;
    double maxPosLimit(int vidx) const override;
    bool hasPosLimit(int vidx) const override;
    bool isContinuous(int vidx) const override;
    double velLimit(int vidx) const override;
    double accLimit(int vidx) const override;
    bool checkJointLimits(const smpl::RobotState& state, bool verbose = false) override;

    auto computeFK(const smpl::RobotState& state) -> Eigen::Affine3d override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;
};

double RobotModel::minPosLimit(int vidx) const
{
    return variables[vidx].min_position;
}

double RobotModel::maxPosLimit(int vidx) const
{
    return variables[vidx].max_position;
}

bool RobotModel::hasPosLimit(int vidx) const
{
    return (variables[vidx].flags & VariableProperties::BOUNDED) != 0;
}

bool RobotModel::isContinuous(int vidx) const
{
    return (variables[vidx].flags & VariableProperties::CONTINUOUS) != 0;
}

double RobotModel::velLimit(int vidx) const
{
    return variables[vidx].max_velocity;
}

double RobotModel::accLimit(int vidx) const
{
    return variables[vidx].max_acceleration;
}

bool RobotModel::checkJointLimits(const smpl::RobotState& state, bool verbose)
{
    // TODO: extract these from SpaceInformation
    return true;
}

auto RobotModel::computeFK(const smpl::RobotState& state) -> Eigen::Affine3d
{
    auto s = MakeStateOMPL(this->si->getStateSpace(), state);
    OMPLProjection projected;
    this->projection->project(s.get(), projected);
    return Eigen::Translation3d(projected[0], projected[1], projected[2]) *
            Eigen::AngleAxisd(projected[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(projected[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(projected[5], Eigen::Vector3d::UnitX());
}

auto RobotModel::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::RobotModel>()) {
        return this;
    }

    if (class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>() &&
        this->projection != NULL &&
        this->projection->getDimension() == 6)
    {
        return this;
    }

    return NULL;
}

/////////////////////////////////////
// CollisionChecker Implementation //
/////////////////////////////////////

struct CollisionChecker : public smpl::CollisionChecker
{
    ompl::base::StateSpace* space;
    ompl::base::StateValidityChecker* checker;
    ompl::base::MotionValidator* validator;
    OMPLPlanner::VisualizerFun visualizer;

    /// \name smpl::CollisionChecker Interface
    ///@{
    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose = false) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;

    auto getCollisionModelVisualization(const RobotState& state)
        -> std::vector<smpl::visual::Marker> override;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> smpl::Extension* override;
    ///@}
};

bool CollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    auto* s = MakeStateOMPL(this->space, state);
    DEFER(this->space->freeState(s));
    return this->checker->isValid(s);
}

bool CollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    auto* s = MakeStateOMPL(this->space, start);
    auto* f = MakeStateOMPL(this->space, finish);
    DEFER(this->space->freeState(s));
    DEFER(this->space->freeState(f));
    return this->validator->checkMotion(s, f);
}

bool CollisionChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    // TODO: only used practically for shortcutting...we'll let ompl handle
    // path simplification and not think too hard about that here
    path.push_back(start);
    path.push_back(finish);
    return true;
}

auto CollisionChecker::getCollisionModelVisualization(const RobotState& state)
    -> std::vector<smpl::visual::Marker>
{
    return this->visualizer(state);
}

auto CollisionChecker::getExtension(size_t class_code)
    -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
        return this;
    }
    return NULL;
}

enum struct ConcreteSpaceType
{
    C_FOREST_STATE_SPACE_WRAPPER,
    MORSE_STATE_SPACE_TYPE,
};

struct PlannerImpl
{
    // world model interface
    RobotModel model;
    CollisionChecker checker;

    // graph
    std::string mprim_filename;
    smpl::ManipLattice space;
    smpl::ManipLatticeActionSpace actions;

    // heuristic
    std::unique_ptr<smpl::RobotHeuristic> heuristic;

    // search
    std::unique_ptr<smpl::ARAStar> search;

    OccupancyGrid* grid = NULL;

    bool initialized = false;

    PlannerImpl(
        OMPLPlanner* planner,
        ompl::base::SpaceInformation* si,
        const std::string& planner_id,
        OccupancyGrid* grid);

    void setProblemDefinition(OMPLPlanner* planner, const ompl::base::ProblemDefinitionPtr& pdef);

    auto solve(OMPLPlanner* planner, const ompl::base::PlannerTerminationCondition& ptc)
        -> ompl::base::PlannerStatus;

    void clear(OMPLPlanner* planner);

    void checkValidity(OMPLPlanner* planner);

    void setup(OMPLPlanner* planner);

    void getPlannerData(const OMPLPlanner* planner, ompl::base::PlannerData& data) const;
};

static
bool MakeVariableProperties(
    const ompl::base::StateSpace* space,
    std::vector<std::string>& names,
    std::vector<RobotModel::VariableProperties>& props)
{
    const ompl::base::CompoundStateSpace* compound_space;
    const ompl::base::DiscreteStateSpace* discrete_space;
    const ompl::base::RealVectorStateSpace* real_space;
    const ompl::base::SO2StateSpace* so2_space;
    const ompl::base::SO3StateSpace* so3_space;
    const ompl::base::TimeStateSpace* time_space;
    if ((compound_space = dynamic_cast<const ompl::base::CompoundStateSpace*>(space)) != NULL) {
        for (auto& ss : compound_space->getSubspaces()) {
            if (!MakeVariableProperties(ss.get(), names, props)) {
                return false;
            }
        }
    } else if ((discrete_space = dynamic_cast<const ompl::base::DiscreteStateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = discrete_space->getLowerBound();
        prop.max_position = discrete_space->getUpperBound();
        prop.flags |= RobotModel::VariableProperties::BOUNDED;

        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();

        names.push_back("discrete");
        props.push_back(prop);
    } else if ((real_space = dynamic_cast<const ompl::base::RealVectorStateSpace*>(space)) != NULL) {
        for (auto i = 0; i < real_space->getDimension(); ++i) {
            RobotModel::VariableProperties prop;
            prop.min_position = real_space->getBounds().low[i];
            prop.max_position = real_space->getBounds().high[i];
            prop.flags |= RobotModel::VariableProperties::BOUNDED;
            prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
            prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();

            if (real_space->getDimensionName(i).empty()) {
                names.push_back("real" + std::to_string(i));
            } else {
                names.push_back(real_space->getDimensionName(i));
            }
            props.push_back(prop);
        }
    } else if ((so2_space = dynamic_cast<const ompl::base::SO2StateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = -std::numeric_limits<double>::infinity();
        prop.max_position = std::numeric_limits<double>::infinity();
        prop.flags |= RobotModel::VariableProperties::CONTINUOUS;
        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();
        names.push_back("so2");
        props.push_back(prop);
    } else if ((so3_space = dynamic_cast<const ompl::base::SO3StateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = -std::numeric_limits<double>::infinity();
        prop.max_position = std::numeric_limits<double>::infinity();
        prop.flags |= RobotModel::VariableProperties::CONTINUOUS;
        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();
        names.push_back("so3_r");
        props.push_back(prop);

        names.push_back("so3_p");
        props.push_back(prop);

        names.push_back("so3_y");
        props.push_back(prop);
    } else {
        return false; // unsupported state space type
    }

    return true;
}

PlannerImpl::PlannerImpl(
    OMPLPlanner* planner,
    ompl::base::SpaceInformation* si,
    const std::string& planner_id,
    OccupancyGrid* grid)
{
    SMPL_DEBUG("Construct Planner");

    this->grid = grid;

    planner->specs_.approximateSolutions = false;
    planner->specs_.canReportIntermediateSolutions = true; //false;
    planner->specs_.directed = true;
    planner->specs_.multithreaded = false;
    planner->specs_.optimizingPaths = true;
    planner->specs_.provingSolutionNonExistence = true;

    // hmm...
    planner->specs_.recognizedGoal = ompl::base::GoalType::GOAL_ANY;

    ////////////////////////////////
    // Log state space properties //
    ////////////////////////////////

    SMPL_DEBUG("dimensions = %u", si->getStateSpace()->getDimension());

    std::map<std::string, std::string> params;
    si->getStateSpace()->params().getParams(params);

    si->getStateSpace()->computeLocations();
    auto& locations = si->getStateSpace()->getValueLocations();
    SMPL_DEBUG("locations = %zu", locations.size());
    for (auto& location : locations) {
        SMPL_DEBUG("  index = %zu", location.index);
        SMPL_DEBUG("  chain size = %zu", location.stateLocation.chain.size());
        SMPL_DEBUG_STREAM("  chain: " << location.stateLocation.chain);
        SMPL_DEBUG("  dimension = %u", location.stateLocation.space->getDimension());
    }

    std::vector<int> signature;
    si->getStateSpace()->computeSignature(signature);
    SMPL_DEBUG_STREAM("signature = " << signature);

    SMPL_DEBUG("name = %s", si->getStateSpace()->getName().c_str());
    SMPL_DEBUG("measure = %f", si->getStateSpace()->getMeasure());
    SMPL_DEBUG("maximum extent = %f", si->getStateSpace()->getMaximumExtent());

    SMPL_DEBUG("Params:");
    for (auto& p : params) {
        SMPL_DEBUG("  %s: %s", p.first.c_str(), p.second.c_str());
    }

    SMPL_DEBUG("Projections:");
    for (auto& proj : si->getStateSpace()->getRegisteredProjections()) {
        SMPL_DEBUG("  %s", proj.first.c_str());
    }

    ///////////////////////////
    // Initialize RobotModel //
    ///////////////////////////

    this->model.si = si;

    {
        std::vector<std::string> names;
        std::vector<RobotModel::VariableProperties> props;
        if (!MakeVariableProperties(si->getStateSpace().get(), names, props)) {
            SMPL_WARN("Failed to construct Planner!");
            return;
        }

        SMPL_DEBUG_STREAM("variable names = " << names);

        model.setPlanningJoints(names);
        model.variables = std::move(props);

        if (si->getStateSpace()->hasProjection("fk")) {
            auto proj = si->getStateSpace()->getProjection("fk");
            model.projection = proj.get();
        }
    }

    ////////////////////////////////////////////
    // Initialize Collision Checker Interface //
    ////////////////////////////////////////////

    this->checker.space = planner->getSpaceInformation()->getStateSpace().get();
    this->checker.checker = planner->getSpaceInformation()->getStateValidityChecker().get();
    this->checker.validator = planner->getSpaceInformation()->getMotionValidator().get();

    //////////////////////////////
    // Initialize Manip Lattice //
    //////////////////////////////

    auto res = 0.05;

    std::vector<double> resolutions;
    resolutions.resize(this->model.getPlanningJoints().size(), res);
    if (!this->space.init(
            &this->model,
            &this->checker,
            resolutions,
            &this->actions))
    {
        SMPL_WARN("Failed to initialize manip lattice");
        return;
    }

    if (grid != NULL) {
        this->space.setVisualizationFrameId(grid->getReferenceFrame());
    }

    if (!this->actions.init(&this->space)) {
        SMPL_WARN("Failed to initialize Manip Lattice Action Space");
        return;
    }

    for (int i = 0; i < (int)this->model.getPlanningJoints().size(); ++i) {
        std::vector<double> mprim(this->model.getPlanningJoints().size(), 0.0);
        mprim[i] = res;
        this->actions.addMotionPrim(mprim, false);
    }

#if 0
    SMPL_DEBUG("Action Set:");
    for (auto ait = this->actions.begin(); ait != this->actions.end(); ++ait) {
        SMPL_DEBUG("  type: %s", to_cstring(ait->type));
        if (ait->type == smpl::MotionPrimitive::SNAP_TO_RPY) {
            SMPL_DEBUG("    enabled: %s", this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            SMPL_DEBUG("    thresh: %0.3f", this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == smpl::MotionPrimitive::SNAP_TO_XYZ) {
            SMPL_DEBUG("    enabled: %s", this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            SMPL_DEBUG("    thresh: %0.3f", this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == smpl::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            SMPL_DEBUG("    enabled: %s", this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            SMPL_DEBUG("    thresh: %0.3f", this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == smpl::MotionPrimitive::LONG_DISTANCE ||
            ait->type == smpl::MotionPrimitive::SHORT_DISTANCE)
        {
            SMPL_DEBUG_STREAM("    action: " << ait->action);
        }
    }
#endif

    //////////////////////////
    // Initialize Heuristic //
    //////////////////////////

    if (planner_id.empty()) {
        this->heuristic = make_unique<JointDistHeuristic>();
        if (!this->heuristic->init(&this->space)) {
            return;
        }
    } else if (planner_id == "arastar.joint_dist.manip") {
        this->heuristic = make_unique<JointDistHeuristic>();
        if (!this->heuristic->init(&this->space)) {
            return;
        }
    } else if (planner_id == "arastar.bfs.manip") {
        auto bfs_heuristic = make_unique<BfsHeuristic>();
        if (!bfs_heuristic->init(&this->space, grid)) {
            return;
        }
        this->heuristic = std::move(bfs_heuristic);
    } else {
        SMPL_ERROR("Unrecognized planner name");
        return;
    }

    ///////////////////////////
    // Initialize the Search //
    ///////////////////////////

    this->search = make_unique<ARAStar>(&this->space, this->heuristic.get());

    ////////////////////////
    // Declare Parameters //
    ////////////////////////

    // declare state space discretization parameters...
    for (auto i = 0; i < this->model.getPlanningJoints().size(); ++i) {
        auto& vname = this->model.getPlanningJoints()[i];
        auto set = [this](double discretization) { /* TODO */ };
        auto get = [this, i]() { return this->space.resolutions()[i]; };
        planner->params().declareParam<double>("discretization_" + vname, set, get);
    }

    // declare action space parameters...
    {
        auto set = [&](const std::string& val) {
            if (this->actions.load(val)) {
                this->mprim_filename = val;
            }
        };
        auto get = [&]() { return mprim_filename; };
        planner->params().declareParam<std::string>("mprim_filename", set, get);
    }

    {
        auto set = [&](bool val) { this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ, val); };
        auto get = [&]() { return this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ); };
        planner->params().declareParam<double>("use_xyz_snap_mprim", set, get);
    }

    {
        auto set = [&](bool val) { this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_RPY, val); };
        auto get = [&]() { return this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_RPY); };
        planner->params().declareParam<double>("use_rpy_snap_mprim", set, get);
    }

    {
        auto set = [&](bool val) { this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY, val); };
        auto get = [&]() { return this->actions.useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY); };
        planner->params().declareParam<double>("use_xyzrpy_snap_mprim", set, get);
    }

    {
        auto set = [&](bool val) { this->actions.useAmp(smpl::MotionPrimitive::SHORT_DISTANCE, val); };
        auto get = [&]() { return this->actions.useAmp(smpl::MotionPrimitive::SHORT_DISTANCE); };
        planner->params().declareParam<double>("use_short_dist_mprims", set, get);
    }

    {
        auto set = [&](double val) { this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ, val); };
        auto get = [&]() { return this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ); };
        planner->params().declareParam<double>("xyz_snap_dist_thresh", set, get);
    }

    {
        auto set = [&](double val) { this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY, val); };
        auto get = [&]() { return this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY); };
        planner->params().declareParam<double>("rpy_snap_dist_thresh", set, get);
    }

    {
        auto set = [&](double val) { this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY, val); };
        auto get = [&]() { return this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY); };
        planner->params().declareParam<double>("rpy_snap_dist_thresh", set, get);
    }

    {
        auto set = [&](double val) { this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY, val); };
        auto get = [&]() { return this->actions.ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY); };
        planner->params().declareParam<double>("xyzrpy_snap_dist_thresh", set, get);
    }

    {
        auto set = [&](double val) { this->actions.ampThresh(smpl::MotionPrimitive::SHORT_DISTANCE, val); };
        auto get = [&]() { return this->actions.ampThresh(smpl::MotionPrimitive::SHORT_DISTANCE); };
        planner->params().declareParam<double>("short_dist_mprims_thresh", set, get);
    }

    {
        auto set = [&](bool val) { this->actions.useMultipleIkSolutions(val); };
        auto get = [&]() { return this->actions.useMultipleIkSolutions(); };
        planner->params().declareParam<double>("short_dist_mprims_thresh", set, get);
    }

    // declare heuristic parameters...

    {
        auto* bfs_heuristic = dynamic_cast<BfsHeuristic*>(this->heuristic.get());
        if (bfs_heuristic != NULL) {
            {
                auto set = [bfs_heuristic](double val) { bfs_heuristic->setInflationRadius(val); };
                auto get = [bfs_heuristic]() { return bfs_heuristic->inflationRadius(); };
                planner->params().declareParam<double>("bfs_inflation_radius", set, get);
            }

            {
                auto set = [bfs_heuristic](int val) { bfs_heuristic->setCostPerCell(val); };
                auto get = [bfs_heuristic]() { return bfs_heuristic->costPerCell(); };
                planner->params().declareParam<int>("bfs_cost_per_cell", set, get);
            }
        }
    }

    // declare search parameters...

    {
        auto set = [&](double val) { this->search->set_initialsolution_eps(val); };
        auto get = [&]() { return this->search->get_initial_eps(); };
        planner->params().declareParam<double>("epsilon", set, get);
    }

    {
        auto set = [&](bool val) { this->search->set_search_mode(val); };
        auto get = [&]() { return false; /* TODO */ };
        planner->params().declareParam<bool>("search_mode", set, get);
    }

    {
        auto set = [&](bool val) { this->search->allowPartialSolutions(val); };
        auto get = [&]() { return this->search->allowPartialSolutions(); };
        planner->params().declareParam<bool>("allow_partial_solutions", set, get);
    }

    {
        auto set = [&](double val) { this->search->setTargetEpsilon(val); };
        auto get = [&]() { return this->search->targetEpsilon(); };
        planner->params().declareParam<double>("target_epsilon", set, get);
    }

    {
        auto set = [&](double val) { this->search->setDeltaEpsilon(val); };
        auto get = [&]() { return this->search->deltaEpsilon(); };
        planner->params().declareParam<double>("delta_epsilon", set, get);
    }

    {
        auto set = [&](bool val) { this->search->setImproveSolution(val); };
        auto get = [&]() { return this->search->improveSolution(); };
        planner->params().declareParam<double>("improve_solution", set, get);
    }

    {
        auto set = [&](bool val) { this->search->setBoundExpansions(val); };
        auto get = [&]() { return this->search->boundExpansions(); };
        planner->params().declareParam<bool>("bound_expansions", set, get);
    }

    {
        auto set = [&](double val) { this->search->setAllowedRepairTime(val); };
        auto get = [&]() { return this->search->allowedRepairTime(); };
        planner->params().declareParam<double>("repair_time", set, get);
    }

    this->initialized = true;
}

bool IsAnyGoal(void* user, const smpl::RobotState& state)
{
    auto* planner = static_cast<OMPLPlanner*>(user);
    auto* space = planner->getSpaceInformation()->getStateSpace().get();
    return planner->getProblemDefinition()->getGoal()->isSatisfied(
            MakeStateOMPL(space, state));
}

auto to_cstring(ompl::base::GoalType type) -> const char*
{
    switch (type) {
    case ompl::base::GoalType::GOAL_ANY:
        return "GOAL_ANY";
    case ompl::base::GoalType::GOAL_LAZY_SAMPLES:
        return "GOAL_LAZY_SAMPLES";
    case ompl::base::GoalType::GOAL_STATE:
        return "GOAL_STATE";
    case ompl::base::GoalType::GOAL_REGION:
        return "GOAL_REGION";
    case ompl::base::GoalType::GOAL_STATES:
        return "GOAL_STATES";
    case ompl::base::GoalType::GOAL_SAMPLEABLE_REGION:
        return "GOAL_SAMPLEABLE_REGION";
    default:
        return "<UNRECOGNIZED>";
    }
}

auto PlannerImpl::solve(
    OMPLPlanner* planner,
    const ompl::base::PlannerTerminationCondition& ptc)
    -> ompl::base::PlannerStatus
{
    SMPL_DEBUG("Planner::solve");

    auto* si = planner->getSpaceInformation().get();
    auto* ompl_space = si->getStateSpace().get();
    auto* pdef = planner->getProblemDefinition().get();

    if (pdef->getSpaceInformation().get() != si) {
        SMPL_ERROR("wtf");
    }

    /////////////////////////
    // Set the start state //
    /////////////////////////

    {
        if (pdef->getStartStateCount() > 1) {
            SMPL_WARN("Received multiple start states. Why?");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_START);
        }

        auto* start = pdef->getStartState(0);
        auto start_state = MakeStateSMPL(ompl_space, start);
        SMPL_DEBUG_STREAM("start state = " << start_state);
        if (!this->space.setStart(start_state)) {
            SMPL_WARN("Failed to set start state");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_START);
        }

        this->heuristic->updateStart(start_state);
    }

    ////////////////////////
    // Set the goal state //
    ////////////////////////

    {
        smpl::GoalConstraint goal_condition;

        // Inheritance hierarchy for goal types:
        // GoalAny
        //      GoalRegion
        //          GoalSampleableRegion
        //              GoalState
        //              GoalStates
        //                  GoalLazySamples

        auto& abstract_goal = pdef->getGoal();
        SMPL_DEBUG("Received goal of type %s", to_cstring(abstract_goal->getType()));

        switch (abstract_goal->getType()) {
        case ompl::base::GoalType::GOAL_ANY:
        {
            auto* pose_goal = dynamic_cast<PoseGoal*>(abstract_goal.get());
            if (pose_goal != NULL) {
                SMPL_INFO("Got ourselves a pose goal!");
                goal_condition.type = smpl::GoalType::XYZ_RPY_GOAL;
                goal_condition.pose = pose_goal->pose;
                goal_condition.xyz_tolerance[0] = pose_goal->position_tolerance[0];
                goal_condition.xyz_tolerance[1] = pose_goal->position_tolerance[1];
                goal_condition.xyz_tolerance[2] = pose_goal->position_tolerance[2];
                goal_condition.rpy_tolerance[0] = pose_goal->orientation_tolerance[0];
                goal_condition.rpy_tolerance[1] = pose_goal->orientation_tolerance[1];
                goal_condition.rpy_tolerance[2] = pose_goal->orientation_tolerance[2];
                break;
            }
        }
        case ompl::base::GoalType::GOAL_REGION:
        case ompl::base::GoalType::GOAL_SAMPLEABLE_REGION:
        case ompl::base::GoalType::GOAL_STATES:
        case ompl::base::GoalType::GOAL_LAZY_SAMPLES:
        {
            auto* goal = static_cast<ompl::base::Goal*>(abstract_goal.get());
            goal_condition.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;
            goal_condition.check_goal = IsAnyGoal;
            goal_condition.check_goal_user = planner;
            break;
        }
        case ompl::base::GoalType::GOAL_STATE:
        {
            auto* goal = static_cast<ompl::base::GoalState*>(abstract_goal.get());
            auto goal_state = MakeStateSMPL(ompl_space, goal->getState());
            SMPL_DEBUG_STREAM("goal state = " << goal_state);
            goal_condition.type = smpl::GoalType::JOINT_STATE_GOAL;
            goal_condition.angles = goal_state;
            // UGH
            goal_condition.angle_tolerances.resize(
                    goal_condition.angles.size(),
                    0.5 * this->space.resolutions().front());
            break;
        }
        default:
            SMPL_WARN("Unrecognized OMPL goal type");
            break;
        }

        if (!space.setGoal(goal_condition)) {
            SMPL_WARN("Failed to set goal");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_GOAL);
        }

        this->heuristic->updateGoal(goal_condition);
    }

    auto* bfs_heuristic = dynamic_cast<BfsHeuristic*>(this->heuristic.get());
    if (bfs_heuristic != NULL) {
        SV_SHOW_DEBUG_NAMED("bfs_walls", bfs_heuristic->getWallsVisualization());
        SV_SHOW_DEBUG_NAMED("bfs_values", bfs_heuristic->getValuesVisualization());
    }

    //////////////////
    // Do the thing //
    //////////////////

    // TODO: hmmm, is this needed? this should probably be part of clear()
    // and allow the state of the search to persist between calls
    this->search->force_planning_from_scratch();

    smpl::ARAStar::TimeParameters time_params;
    time_params.bounded = this->search->boundExpansions();
    time_params.improve = this->search->improveSolution();
    time_params.type = smpl::ARAStar::TimeParameters::USER;
    time_params.timed_out_fun = [&]() { return ptc.eval(); };

    auto start_id = space.getStartStateID();
    auto goal_id = space.getGoalStateID();
    this->search->set_start(start_id);
    this->search->set_goal(goal_id);

    std::vector<int> solution;
    int cost;
    auto res = this->search->replan(time_params, &solution, &cost);

    if (!res) {
        SMPL_WARN("Failed to find solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::TIMEOUT);
    }

    SMPL_DEBUG("Expands: %d", this->search->get_n_expands());
    SMPL_DEBUG("Expands (Init): %d", this->search->get_n_expands_init_solution());
    SMPL_DEBUG("Epsilon: %f", this->search->get_final_epsilon());
    SMPL_DEBUG("Epsilon (Init): %f", this->search->get_initial_eps());

#if 0
    // TODO: hidden ARA*-specific return codes
    switch (res) {
    case 0: // SUCCESS
        SMPL_DEBUG("Planner found optimal solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
        break;
    case 4: // TIMED_OUT
        SMPL_DEBUG("Planner timed out");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::TIMEOUT);
        break;
    case 5: // EXHAUSTED_OPEN_LIST
        SMPL_DEBUG("Planner returned with an exact solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
        break;
    }
#endif

    //////////////////////////////////////////////////////////
    // Convert discrete state path to continuous state path //
    //////////////////////////////////////////////////////////

    std::vector<smpl::RobotState> path;
    if (!space.extractPath(solution, path)) {
        return ompl::base::PlannerStatus::CRASH;
    }

    auto* p_path = new ompl::geometric::PathGeometric(planner->getSpaceInformation());

    for (auto& p : path) {
        auto* ompl_state = MakeStateOMPL(ompl_space, p);
        p_path->append(ompl_state);
    }

    auto ompl_path = ompl::base::PathPtr(p_path);
    planner->getProblemDefinition()->addSolutionPath(ompl_path);
    return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
}

void PlannerImpl::setProblemDefinition(
    OMPLPlanner* planner,
    const ompl::base::ProblemDefinitionPtr& pdef)
{
    SMPL_DEBUG("Planner::setProblemDefinition");
    planner->ompl::base::Planner::setProblemDefinition(pdef);
}

void PlannerImpl::clear(OMPLPlanner* planner)
{
    SMPL_DEBUG("TODO: Planner::clear");
    planner->ompl::base::Planner::clear();

    // NOTE: can set this->setup_ to false to communicate that setup should
    // be called again
}

void PlannerImpl::setup(OMPLPlanner* planner)
{
    SMPL_DEBUG("Planner::setup");
    planner->ompl::base::Planner::setup();
}

void PlannerImpl::checkValidity(OMPLPlanner* planner)
{
    SMPL_DEBUG("Planner::checkValidity");
    planner->ompl::base::Planner::checkValidity(); // lol, throws exceptions
}

void PlannerImpl::getPlannerData(
    const OMPLPlanner* planner,
    ompl::base::PlannerData& data) const
{
    SMPL_DEBUG("TODO: Planner::getPlannerData");
    planner->ompl::base::Planner::getPlannerData(data);
}

void SetStateVisualizer(PlannerImpl* planner, const OMPLPlanner::VisualizerFun& fun)
{
    planner->checker.visualizer = fun;
}

void SetOccupancyGrid(PlannerImpl* planner, OccupancyGrid* grid)
{
    planner->grid = grid;
}

} // namespace detail

////////////////////////////////
// OMPLPlanner Implementation //
////////////////////////////////

PoseGoal::PoseGoal(
    const ompl::base::SpaceInformationPtr& si,
    const Eigen::Affine3d& pose)
:
    ompl::base::Goal(si),
    pose(pose)
{
}

bool PoseGoal::isSatisfied(const ompl::base::State* state) const
{
    // TODO: is this useful generally to anyone? This class gets converted to
    // smpl's internal goal representation anyway
    return false;
}

OMPLPlanner::OMPLPlanner(
    const ompl::base::SpaceInformationPtr& si,
    const std::string& planner_id,
    OccupancyGrid* grid)
:
    Planner(si, "smpl_planner"),
    m_impl(make_unique<smpl::detail::PlannerImpl>(
            this, si.get(), planner_id, grid))
{
}

OMPLPlanner::~OMPLPlanner()
{
}

void OMPLPlanner::setStateVisualizer(const VisualizerFun& fun)
{
    SetStateVisualizer(this->m_impl.get(), fun);
}

void OMPLPlanner::setOccupancyGrid(OccupancyGrid* grid)
{
    SetOccupancyGrid(this->m_impl.get(), grid);
}

void OMPLPlanner::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef)
{
    return m_impl->setProblemDefinition(this, pdef);
}

auto OMPLPlanner::solve(const ompl::base::PlannerTerminationCondition& ptc)
    -> ompl::base::PlannerStatus
{
    return m_impl->solve(this, ptc);
}

void OMPLPlanner::clear()
{
    return m_impl->clear(this);
}

void OMPLPlanner::checkValidity()
{
    return m_impl->checkValidity(this);
}

void OMPLPlanner::setup()
{
    return m_impl->setup(this);
}

void OMPLPlanner::getPlannerData(ompl::base::PlannerData& data) const
{
    return m_impl->getPlannerData(this, data);
}

auto MakeStateSMPL(
    const ompl::base::StateSpace* space,
    const ompl::base::State* state)
    -> smpl::RobotState
{
    smpl::RobotState s;
    space->copyToReals(s, state);
    return s;
};

auto MakeStateOMPL(
    const ompl::base::StateSpace* space,
    const smpl::RobotState& state)
    -> ompl::base::State*
{
    auto* s = space->allocState();
    space->copyFromReals(s, state);
    return s;
}

auto MakeStateOMPL(
    const ompl::base::StateSpacePtr& space,
    const smpl::RobotState& state)
    -> ompl::base::ScopedState<>
{
    auto s = ompl::base::ScopedState<>(space);
    space->copyFromReals(s.get(), state);
    return std::move(s);
}

} // namespace smpl

