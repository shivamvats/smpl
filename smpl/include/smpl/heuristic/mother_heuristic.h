#ifndef SMPL_MOTHER_HEURISTIC_H
#define SMPL_MOTHER_HEURISTIC_H

#include "robot_heuristic.h"
#include "bfs_2d_heuristic.h"
#include "bfs_3d_heuristic.h"
#include "bfs_3d_base_heuristic.h"

//namespace smpl {
struct MotherHeuristic : public smpl::RobotHeuristic {

    inline double getMetricStartDistance( double, double, double ){return 0.0;}
    inline double getMetricGoalDistance( double, double, double ){return 0.0;}
    inline Extension* getExtension(size_t class_code){
        if (class_code == smpl::GetClassCode<smpl::RobotHeuristic>()) {
            return this;
        }
        return nullptr;
    }

    virtual int GetGoalHeuristic(int) = 0;
    int GetStartHeuristic(int state_id){return 0;}
    int GetFromToHeuristic(int from_id, int to_id){return 0;}
};

struct BfsHeuristic : public MotherHeuristic {

    inline bool init( std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
            std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d ){
        bfs_3d_base = _bfs_3d_base;
        bfs_3d = _bfs_3d;
        return true;
    }

    inline void updateGoal(const smpl::GoalConstraint& _goal) override {
        if(areClose(_goal.pose, bfs_3d_base->getGoal().pose) &&
                areClose(_goal.pose, bfs_3d->getGoal().pose))
            return;
        goal = _goal;
        bfs_3d_base->updateGoal(_goal);
        bfs_3d->updateGoal(_goal);
    }

    std::shared_ptr<smpl::Bfs3DBaseHeuristic> bfs_3d_base;
    std::shared_ptr<smpl::Bfs3DHeuristic> bfs_3d;
    smpl::GoalConstraint goal;

    protected:

    template <typename T>
    inline double euclidDist( const T* a, const T* b, unsigned int n ){
        double dist_sqrd = 0.0;
        for(int i=0; i<n; i++)
            dist_sqrd += (a[i] - b[i])*(a[i] - b[i]);
        return sqrt(dist_sqrd);
    }

    inline bool areClose( const smpl::Affine3& a, const smpl::Affine3& b ){
        auto trans_a = a.translation(), trans_b = b.translation();
        auto rot_a = a.rotation(), rot_b = b.rotation();
        if(euclidDist(trans_a.data(), trans_b.data(), 3) > 0.01)
            return false;
        if(euclidDist(rot_a.data(), rot_b.data(), 4) > 0.01)
            return false;
        return true;
    }
};

//} //namespace smpl

#endif
