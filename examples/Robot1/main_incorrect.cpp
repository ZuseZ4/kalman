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

void predict(Kalman::ExtendedKalmanFilter<State>& ekf, SystemModel& sys, const Control& u ) {
    ekf.x = sys.f(ekf.x, u);
    
    ekf.P  = ( sys.F * ekf.P * sys.F.transpose() ) + ( sys.W * sys.getCovariance() * sys.W.transpose() );
}

double simulate(double input) {
  State x;
  x.setZero();

  Control u;
  SystemModel sys;

  Kalman::ExtendedKalmanFilter<State> ekf;

  ekf.init(x);

  double ekfy_sum = 0.0;
  for (size_t i = 1; i < 3; i++) {
    u.v() = input;
    u.dtheta() = 2.0;

    // predict(ekf, sys, u);
    ekf.x = sys.f(ekf.x, u);
    
    ekf.P  = ( sys.F * ekf.P * sys.F.transpose() ) + ( sys.W * sys.getCovariance() * sys.W.transpose() );

    ekfy_sum += ekf.x.y();
  }

  return ekfy_sum;
}

int main(int argc, char **argv) {
    double fx1 = simulate(1.0);
    double fx2 = simulate(1.1);
    printf("x = %f, f(x) = %f\n", 1.0, fx1);
    printf("x = %f, f(x) = %f\n", 1.1, fx2);

    // double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    // double df_dx2 = __enzyme_autodiff((void *)simulate, 1.1);
    // printf("x = %f, f(x) = %f, f'(x) = %f\n", 1.0, fx1, df_dx1);
    // printf("x = %f, f(x) = %f, f'(x) = %f\n", 1.1, fx2, df_dx2);

    return 0;
}