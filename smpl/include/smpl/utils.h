#ifndef SMPL_COMMON_UTILS_H
#define SMPL_COMMON_UTILS_H

#include <vector>
#include <iostream>
#include <smpl/console/console.h>

template <typename T>
double euclideanDistance( std::vector<T>& s1, std::vector<T>& s2, std::vector<double> weights ){
    double dist = 0;
    for( int i=0; i<s1.size(); i++ )
        dist += ( weights[i]*(s1[i] - s2[i])*(s1[i] - s2[i]) );
    return dist;
}


#endif
