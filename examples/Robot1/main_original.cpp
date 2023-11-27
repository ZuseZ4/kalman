
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
  for (size_t i = 1; i <= N; i++) {
    u.v() = input;
    u.dtheta() = 1.0;

    x = sys.f(x, u);

    x.x() += systemNoise * 0.5;
    x.y() += systemNoise * 0.5;
    x.theta() += systemNoise * 0.5;

    auto x_pred = predictor.predict(sys, u);
    auto x_ekf = ekf.predict(sys, u);

    {
      OrientationMeasurement orientation = om.h(x);
      orientation.theta() = 0.1;

      orientation.theta() += orientationNoise * 0.5;

      x_ekf = ekf.update(om, orientation);
    }

    {
      PositionMeasurement position = pm.h(x);

      position.d1() += distanceNoise * 0.5;
      position.d2() += distanceNoise * 0.5;

      x_ekf = ekf.update(pm, position);
    }

    ekfy_sum += x_ekf.y();

    std::cout << x.x() << "," << x.y() << "," << x.theta() << "," << x_pred.x()
              << "," << x_pred.y() << "," << x_pred.theta() << "," << x_ekf.x()
              << "," << x_ekf.y() << "," << x_ekf.theta() << std::endl;
  }
  return ekfy_sum / (double)N;
}

int main(int argc, char **argv) {
    double fx1 = simulate(1.0);
    double fx2 = simulate(1.1);
    std::cout << "fx1: " << fx1 << ", fx2: " << fx2 << std::endl;

    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    double df_dx2 = __enzyme_autodiff((void *)simulate, 1.1);
    printf("x = %f, f(x) = %f, f'(x) = %f\n", 1.0, fx1, df_dx1);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.1, fx2, df_dx2);

    return 0;
}