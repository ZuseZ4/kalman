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

  for (int i = 0; i < 1; i++) {
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
  }

  return ekfy_sum / (double)N;
}

int main(int argc, char **argv) {
    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    return 0;
}