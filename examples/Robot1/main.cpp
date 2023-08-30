
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
// template <typename return_type, typename... T>
// return_type __enzyme_fwddiff(void *, T...);

// template <typename return_type, typename... T>
// return_type __enzyme_autodiff(void *, T...);

extern double __enzyme_autodiff(void *, double);
// double foo(double x) { return x * x; }

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

// using Eigen::MatrixXd;
// using Eigen::VectorXd;
//
// void __enzyme_autodiff2(void *, ...);
// void bar(MatrixXd *m, VectorXd *v) { *v = *m * *v; }

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
  for (size_t i = 1; i <= N; i++) {
    u.v() = input;
    auto x_pred = predictor.predict(sys, u);
    ekfy_sum += u.v();
  }
  return ekfy_sum;
}

int main(int argc, char **argv) {
    double x1 = simulate(1.0);
    double x2 = simulate(1.1);
    std::cout << "x1: " << x1 << ", x2: " << x2 << std::endl;

    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    double df_dx2 = __enzyme_autodiff((void *)simulate, 1.1);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.0, x1, df_dx1);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.1, x2, df_dx2);

    return 0;
}
