
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
  SystemModel sys;
  Kalman::ExtendedKalmanFilter<State> predictor;
  predictor.P  = ( sys.F * predictor.P * sys.F.transpose() ) + ( sys.W * sys.P * sys.W.transpose() );

  return 0.0;
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
