#ifndef EUL2QUAT_H
#define EUL2QUAT_H

#include <eigen3/Eigen/Dense>
inline Eigen::Quaterniond ToEigen(float *eulur)
{
        Eigen::Matrix3d matrix_tmp;
        matrix_tmp = (Eigen::AngleAxisd(eulur[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulur[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eulur[2], Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Quaterniond q(matrix_tmp);
        return q;
}

inline Eigen::Vector3d ToEulur(const Eigen::Quaterniond &q) // q(w,x,y,z)
{
        Eigen::Vector3d eul = q.matrix().eulerAngles(2, 1, 0); // yawl pitch roll
        return eul;
}

#endif