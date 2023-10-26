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

typedef Big::MeasurementModel<T> MeasurementModel;
typedef Big::Measurement<T> Measurement;

typedef Eigen::Matrix<State::Scalar, State::RowsAtCompileTime, State::RowsAtCompileTime> EigenSquare;

double simulate(double* A) {
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
  EigenSquare W;
  EigenSquare F;
  EigenSquare P_sys;
  W.setIdentity();
  F.setIdentity();
  P_sys.setIdentity();

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      F(i, j) = A[n * i + j];
    }
  }

  T noiseLevel = 0.1;
  for (int i = 0; i < n; i++) {
    P_sys(i, i) = std::pow(noiseLevel, 2);
  }

  // init ekf
  State x_ekf;
  EigenSquare P;
  x_ekf.setZero();
  P.setIdentity();

  double error_sum = 0.0;
  const size_t N = 5; // if N = 1 then segfault

  for (size_t i = 1; i <= N; i++) {

    // propagate hidden state
    x = F * x; 
    for (int j = 0; j < n; j++) {
        x[i] += noiseLevel;// * noise(generator)
    }

    // ekf predict
    x_ekf = F * x_ekf; 
    P  = ( F * P * F.transpose() ) + ( W * P_sys * W.transpose() );

    // measurement
    Measurement m = mm.h(x);

    // ekf update
    EigenSquare S = ( mm.H * P * mm.H.transpose() ) + ( mm.V * mm.getCovariance() * mm.V.transpose() );
    EigenSquare Sinv = S;
    EigenSquare K = P * mm.H.transpose() * Sinv;//.inverse();
    x_ekf += K * ( m - mm.h( x_ekf ) );
    P -= K * mm.H * P;
            
    error_sum += std::pow(P(0,0), 2); 

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

    // __enzyme_autodiff<double>((void *)simulate, enzyme_dup, A, Adup);
    // printf("Adup[0] = %f, Adup[1] = %f, Adup[2] = %f, Adup[10] = %f", Adup[0], Adup[1], Adup[2], Adup[10]);

    return 0;
}
