#pragma once

#include <chrono>

#include <eigen3/Eigen/Core>

typedef std::chrono::duration<double>       Duration;

typedef Eigen::VectorXd                     VecXd;
typedef Eigen::Vector2d                     Vec2d;
typedef Eigen::Vector3d                     Vec3d;
typedef Eigen::Matrix<double, 6, 1>         Vec6d;
typedef Eigen::Matrix<double, 7, 1>         Vec7d;
template <int ROWS> using                   VecRd =
                                            Eigen::Matrix<double, ROWS, 1>;
typedef Eigen::MatrixXd                     MatXd;
typedef Eigen::Matrix2d                     Mat2d;
typedef Eigen::Matrix<double, 6, -1>        Mat6Xd;
typedef Eigen::Matrix<double, 6, 7>         Mat67d;
typedef Eigen::DiagonalMatrix<double, -1>   DiagXd;
template <int ROWS, int COLS> using         MatRCd =
                                            Eigen::Matrix<double, ROWS, COLS>;

typedef Eigen::Ref<VecXd>                   VecXdRef;
typedef Eigen::Ref<Vec3d>                   Vec3dRef;
typedef Eigen::Ref<Vec6d>                   Vec6dRef;
template <int R> using                      VecRdRef =
                                            Eigen::Ref<Eigen::Matrix<double, R, 1>>;
typedef Eigen::Ref<MatXd>                   MatXdRef;
typedef Eigen::Ref<Mat6Xd>                  Mat6XdRef;
template <int R, int C> using               MatRCdRef =
                                            Eigen::Ref<Eigen::Matrix<double, R, C>>;
typedef Eigen::Ref<const VecXd>             CVecXdRef;
typedef Eigen::Ref<const Vec3d>             CVec3dRef;
typedef Eigen::Ref<const Vec6d>             CVec6dRef;
template <int R> using                      CVecRdRef =
                                            Eigen::Ref<const Eigen::Matrix<double, R, 1>>;
typedef Eigen::Ref<const MatXd>             CMatXdRef;
typedef Eigen::Ref<const Mat6Xd>            CMat6XdRef;
typedef Eigen::Ref<const Mat67d>            CMat67dRef;
typedef Eigen::Ref<const Mat67d>            CMat67dRef;

typedef Eigen::Map<VecXd>                   VecXdMap;
typedef Eigen::Map<Vec3d>                   Vec3dMap;
typedef Eigen::Map<Vec6d>                   Vec6dMap;
typedef Eigen::Map<MatXd>                   MatXdMap;
typedef Eigen::Map<const VecXd>             CVecXdMap;
typedef Eigen::Map<const Vec3d>             CVec3dMap;
typedef Eigen::Map<const Vec6d>             CVec6dMap;
typedef Eigen::Map<const Mat6Xd>            CMat6XdMap;
typedef Eigen::Map<const Mat67d>            CMat67dMap;
