#ifndef SLAM_STUDY_DUMMY_HPP
#define SLAM_STUDY_DUMMY_HPP

#include "Eigen/Geometry"
#include "pangolin/pangolin.h"

class Dummy
{
public:
    Dummy() = default;

private:
    Eigen::Matrix3d eigen_mat;
};

#endif //SLAM_STUDY_DUMMY_HPP
