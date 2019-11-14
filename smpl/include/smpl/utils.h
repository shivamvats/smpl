#ifndef SMPL_COMMON_UTILS_H
#define SMPL_COMMON_UTILS_H

#include <vector>
#include <iostream>
#include <smpl/console/console.h>
#include <std_msgs/ColorRGBA.h>
#include <smpl/debug/marker.h>

template <typename T>
double euclideanDistance( std::vector<T>& s1, std::vector<T>& s2, std::vector<double> weights ){
    double dist = 0;
    for( int i=0; i<s1.size(); i++ )
        dist += ( weights[i]*(s1[i] - s2[i])*(s1[i] - s2[i]) );
    return dist;
}

inline smpl::visual::Color StdMsgColorToSmpl( const std_msgs::ColorRGBA _color )
{
    smpl::visual::Color color;
    color.r = _color.r;
    color.g = _color.g;
    color.b = _color.b;
    color.a = _color.a;
    return color;
}

#endif
