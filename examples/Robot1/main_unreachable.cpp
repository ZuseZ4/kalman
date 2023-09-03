// #define _USE_MATH_DEFINES
// #include <cmath>

// #define EIGEN_USE_BLAS

// #include <iostream>
// #include <random>
// #include <chrono>

// #include <Eigen/Core>

// extern double __enzyme_autodiff(void *, double);

// typedef float T;

// typedef Eigen::Matrix<T, 3, 1> State;
// typedef Eigen::Matrix<T, 2, 1> Control;

// typedef Eigen::Matrix<T, 1, 1> OrientationMeasurement;

// struct ExtendedKalmanFilter {
//   Eigen::Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime> P;
//   State x;
// };

// class KalmanSystemModel
// {
//     public:
//         virtual State f(const State& x, const Control& u) const = 0;
//     protected:
//         KalmanSystemModel() {}
//         virtual ~KalmanSystemModel() {}
// };

// class LinearizedSystemModel : public KalmanSystemModel
// {
//     public:
//         Eigen::Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime> W;
// };

// class SystemModel : public LinearizedSystemModel
// {
// public:
//     State f(const State& x, const Control& u) const {
//         State x_;
//         auto newOrientation = x[2] + u[1];
//         x_[2] = newOrientation;
//         x_[0] = x[0] + std::cos( newOrientation ) * u[0];
//         x_[1] = x[1] + std::sin( newOrientation ) * u[0];
//         return x_;
//     }
// };

// void predict(ExtendedKalmanFilter& ekf, SystemModel& sys, const Control& u ) {
//     ekf.x = sys.f(ekf.x, u);
//     ekf.P  *= ( sys.W * ekf.P * sys.W.transpose() ) + ( sys.W * sys.W * sys.W.transpose() );
// }

// double simulate(double input) {
//   State x;
//   x.setZero();

//   Control u;
//   SystemModel sys;

//   ExtendedKalmanFilter predictor;
//   ExtendedKalmanFilter ekf;

//   predictor.x = x;
//   ekf.x = x;

//   u[0] = input;
//   u[1] = 2.0;

//   predict(predictor, sys, u);
//   predict(ekf, sys, u);

//   OrientationMeasurement orientation;
//   Eigen::Matrix<T, OrientationMeasurement::RowsAtCompileTime, State::RowsAtCompileTime> H;
//   Eigen::Matrix<T, State::RowsAtCompileTime, OrientationMeasurement::RowsAtCompileTime> K = ekf.P * H.transpose();
//   ekf.x += K * orientation;
//   ekf.P -= K * H * ekf.P;

//   return 0.0;
// }

// int main(int argc, char **argv) {
//     double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
//     return 0;
// }
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>
int enzyme_dup;
int enzyme_dupnoneed;
int enzyme_out;
int enzyme_const;

extern double __enzyme_autodiff(void *, double);

using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;

double simulate(double input) {
  State x;
  x.setZero();

  Control u;
  SystemModel sys;

  PositionModel pm(-10, -10, 30, 75);
  OrientationModel om;

  Kalman::ExtendedKalmanFilter<State> predictor;
  Kalman::ExtendedKalmanFilter<State> ekf;

  predictor.init(x);
  ekf.init(x);

  T systemNoise = 0.1;
  T orientationNoise = 0.025;
  T distanceNoise = 0.25;

  double ekfy_sum = 0.0;
  const size_t N = 100;
  const double dN = N;

  u.v() = input;
  u.dtheta() = 2.0;

  x = sys.f(x, u);

  x.x() += systemNoise * 0.05; // 0.05;
  x.y() += systemNoise * 0.05;
  x.theta() += systemNoise * 0.05;

  auto x_pred = predictor.predict(sys, u);
  auto x_ekf = ekf.predict(sys, u);

  {
    OrientationMeasurement orientation = om.h(x);

    orientation.theta() += orientationNoise * 0.05;

    x_ekf = ekf.update(om, orientation);
  }

  {
    PositionMeasurement position = pm.h(x);

    position.d1() += distanceNoise * 0.05;
    position.d2() += distanceNoise * 0.05;

    x_ekf = ekf.update(pm, position);
  }

    ekfy_sum += x_ekf.y();

  return ekfy_sum / (double)N;
}

int main(int argc, char **argv) {
    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    return 0;
}
