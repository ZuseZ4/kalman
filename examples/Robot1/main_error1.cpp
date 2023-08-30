
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

typedef Eigen::Matrix<T, 3, 1> State;

class LinearizedSystemModel
{
public:
    Eigen::Matrix<typename State::Scalar,State::RowsAtCompileTime, State::RowsAtCompileTime> F;
    
    /**
     * Callback function for state-dependent update of Jacobi-matrices F and W before each update step
     */
    virtual void updateJacobians( const State& x)
    {
        // No update by default
        (void)x;
    }
};

double simulate(double input) {
  LinearizedSystemModel sys;
  sys.F.setIdentity();
  sys.F  = ( sys.F * sys.F * sys.F.transpose() ) + ( sys.F * sys.F * sys.F.transpose() );

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
