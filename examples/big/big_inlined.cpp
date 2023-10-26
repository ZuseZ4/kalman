// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include "big.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>
extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

void __enzyme_autodiff(...);

template<typename RT, typename... Args>
RT __enzyme_autodiff(void*, Args...);

using namespace KalmanExamples;

typedef double T;

typedef Big::State<T> State;
typedef Big::Control<T> Control;
typedef Big::SystemModel<T> SystemModel;

const size_t n = Big::n;

// typedef Big::PositionMeasurement<T> PositionMeasurement;
// typedef Big::OrientationMeasurement<T> OrientationMeasurement;
// typedef Big::PositionMeasurementModel<T> PositionModel;

typedef Big::MeasurementModel<T> MeasurementModel;
typedef Big::Measurement<T> Measurement;

double simulate(double* A) {//Kalman::Jacobian<State, State> A) {
  // init state
  State x;
  for (int i = 0; i < n; i++) {
    x[i] = 1;
  }
  
  // init control
  Control u;
  u[0] = 0.0;

  // init measurement
  MeasurementModel mm;

  // init system
  SystemModel sys;

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      sys.F(i, j) = A[n * i + j];
    }
  }

  // init ekf
  Kalman::ExtendedKalmanFilter<State> ekf;
  ekf.init(x);
  ekf.P.setIdentity(); // explicitly set initial covariance, although identity is secretly already the default

  double error_sum = 0.0;
  const size_t N = 5; // if N = 1 then segfault

  for (size_t i = 1; i <= N; i++) {

    // propagate hidden state
    x = sys.f(x, u);

    // propagate state estimate, read out mean
    State x_ekf = ekf.predict(sys, u);

    // measurement
    Measurement m = mm.h(x);
    ekf.update(mm, m);

    // error_sum += std::pow(x[0], 2); 
    // error_sum += std::pow(x_ekf[0] - x[0], 2); 
    // add a funky P-dependent term to test differentiation
    error_sum += std::pow(ekf.P(0,0), 2); 

    // std::cout << x[0] << "," << x[1] << "," << x_ekf[0]
    //           << "," << x_ekf[1] << std::endl;
  }
  return error_sum / (double)N;
}

int main(int argc, char **argv) {

    // Kalman::Jacobian<State, State> A;
    // Kalman::Jacobian<State, State> Adup;

    double A[n * n];
    double Adup[n * n];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            A[n*i + j] = j == i ? 1.5 : 0.0;
            Adup[n*i + j] = 0.0;
        }
    }
   

    double delta = 0.001;
    double delta2 = delta * delta;
    // double fx1 = simulate(1.0, A);
    // double fx2 = simulate(1.0 + delta, A);
    // std::cout << "fx1: " << fx1 << ", fx2: " << fx2 << std::endl;

    // double df_dx1 = __enzyme_autodiff<double>((void *)simulate, enzyme_out, 1.0, enzyme_const, A);
    // double df_dx2 = __enzyme_autodiff<double>((void *)simulate, enzyme_out, 1.0 + delta, enzyme_const, A);
    // printf("x = %f, f(x) = %f, f'(x) = %f, f'(x) fd = %f\n", 1.0, fx1, df_dx1, (fx2 - fx1) / delta);
    // printf("x = %f, f(x) = %f, f'(x) = %f", 1.0 + delta, fx2, df_dx2);

    double fx1 = simulate(A);
    A[0] += delta;
    double fx2 = simulate(A);
    A[0] -= delta;
    printf("f(A) = %f, f(A + delta) = %f, f'(A)[0] fd = %f\n", fx1,  fx2,(fx2 - fx1) / delta);

    fx1 = simulate(A);
    A[0] += delta2;
    fx2 = simulate(A);
    A[0] -= delta2;
    printf("f(A) = %f, f(A + delta2) = %f, f'(A)[0] fd = %f\n", fx1,  fx2,(fx2 - fx1) / delta2);

    fx1 = simulate(A);
    A[1] += delta;
    fx2 = simulate(A);
    A[1] -= delta;
    printf("f(A) = %f, f(A + delta) = %f, f'(A)[1] fd = %f\n", fx1,  fx2,(fx2 - fx1) / delta);

    fx1 = simulate(A);
    A[2] += delta;
    fx2 = simulate(A);
    A[2] -= delta;
    printf("f(A) = %f, f(A + delta) = %f, f'(A)[2] fd = %f\n", fx1,  fx2,(fx2 - fx1) / delta);

    fx1 = simulate(A);
    A[10] += delta;
    fx2 = simulate(A);
    A[10] -= delta;
    printf("f(A) = %f, f(A + delta) = %f, f'(A)[10] fd = %f\n", fx1,  fx2,(fx2 - fx1) / delta);

    __enzyme_autodiff<double>((void *)simulate, enzyme_dup, A, Adup);
    printf("Adup[0] = %f, Adup[1] = %f, Adup[2] = %f, Adup[10] = %f", Adup[0], Adup[1], Adup[2], Adup[10]);

    return 0;
}
