// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

void __enzyme_autodiff(...);

template<typename RT, typename... Args>
RT __enzyme_autodiff(void*, Args...);

const size_t n = 10;

typedef double T;
typedef Eigen::Matrix<T, n, 1> State;
typedef Eigen::Matrix<T, 1, 1> Control;
typedef Eigen::Matrix<T, n, 1> Measurement;

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
  EigenSquare H;
  EigenSquare V;
  EigenSquare P_meas;
  H.setIdentity();
  V.setIdentity();
  P_meas.setIdentity();

  T noiseLevel_meas = 0.1;
  P_meas *= noiseLevel_meas;

  // init system
  EigenSquare W;
  EigenSquare F;
  EigenSquare P_sys;
  W.setZero();
  F.setZero();
  P_sys.setZero();

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      F(i, j) = A[n * i + j];
    }
  }

  T noiseLevel_sys = 0.1;
  for (int i = 0; i < n; i++) {
    P_sys(i, i) = std::pow(noiseLevel_sys, 2);
  }

  // init ekf
  State x_ekf;
  EigenSquare P;
  x_ekf.setZero();
  P.setIdentity();

  double error_sum = 0.0;
  const size_t N = 2;

  for (size_t i = 1; i <= N; i++) {

    P  = ( F * P * F.transpose() ) + ( W * P_sys * W.transpose() );
    error_sum += std::pow(P(0,0), 2); 
  }

  return error_sum / (double)N;
}

int main(int argc, char **argv) {

    double A[n * n];
    double Adup[n * n];
    double Adup_fd[n * n];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            A[n*i + j] = j == i ? 0.3 : 0.1;
            Adup[n*i + j] = 0.0;
            Adup_fd[n*i + j] = 0.0;
        }
    }

    double delta = 0.001;
    delta = delta * delta;

    double fx = simulate(A);
    printf("f(A) = %f\n", fx);

    for (int i = 0; i < n * n; i++) {
        A[i] += delta/2;
        double fx2 = simulate(A);
        A[i] -= delta;
        double fx3 = simulate(A);
        A[i] += delta/2;
        Adup_fd[i] = (fx2 - fx3) / delta;
    }

    printf("Adup_fd[0] = %f, Adup_fd[1] = %f, Adup_fd[2] = %f, Adup_fd[3] = %f\n", Adup_fd[0], Adup_fd[1], Adup_fd[2], Adup_fd[3]);

    __enzyme_autodiff<double>((void *)simulate, enzyme_dup, A, Adup);
    printf("Adup[0] = %f, Adup[1] = %f, Adup[2] = %f, Adup[3] = %f\n", Adup[0], Adup[1], Adup[2], Adup[3]);

    return 0;
}
