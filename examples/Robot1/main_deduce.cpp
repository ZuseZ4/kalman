// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include "SystemModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>

extern double __enzyme_autodiff(void *, double);

using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

double simulate(double input) {
  State x;
  x.setZero();

  Control u;
  SystemModel sys;

  Kalman::ExtendedKalmanFilter<State> ekf;
  ekf.init(x);

  double ekfy_sum = 0.0;
  auto x_ekf = ekf.predict(sys, u);

  return input; 
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