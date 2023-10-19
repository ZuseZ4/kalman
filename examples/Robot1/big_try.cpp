// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include "big.hpp"

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

typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

// typedef Robot1::PositionMeasurement<T> PositionMeasurement;
// typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
// typedef Robot1::PositionMeasurementModel<T> PositionModel;
// typedef Robot1::OrientationMeasurementModel<T> OrientationModel;

double simulate(double input) {
  // init state
  State x;
  x[0] = 1;
  x[1] = 1;
  
  // init control
  Control u;
  u[0] = input;

  // init system
  SystemModel sys;

  // init ekf
  Kalman::ExtendedKalmanFilter<State> ekf;
  ekf.init(x);

  double error_sum = 0.0;
  const size_t N = 1;

  for (size_t i = 1; i <= N; i++) {

    // propagate hidden state
    x = sys.f(x, u);

    // propagate state estimate, read out mean
    State x_ekf = ekf.predict(sys, u);

    // no measurements :(

    error_sum += (x_ekf[0] - x[0])^2;

    std::cout << x[0] << "," << x[1] << "," << x_ekf[0]
              << "," << x_ekf[1] << std::endl;
  }
  return error_sum / (double)N;
}

int main(int argc, char **argv) {

    double delta = 0.0001;
    double fx1 = simulate(1.0);
    double fx2 = simulate(1.0 + delta);
    std::cout << "fx1: " << fx1 << ", fx2: " << fx2 << std::endl;

    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    double df_dx2 = __enzyme_autodiff((void *)simulate, 1.0 + delta);
    printf("x = %f, f(x) = %f, f'(x) = %f, f'(x) fd = %f\n", 1.0, fx1, df_dx1, (fx2 - fx1) / delta);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.0 + delta, fx2, df_dx2);

    return 0;
}