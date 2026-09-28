#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace krlsim
{
    using Scalar = double;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
    using Color = Eigen::Matrix<Scalar, 4, 1>;

    using JacobianMatrix = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;
}