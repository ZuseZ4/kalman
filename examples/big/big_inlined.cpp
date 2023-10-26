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

    // propagate hidden state
    x = F * x; 
    for (int j = 0; j < n; j++) {
        x[i] += noiseLevel_sys;// * noise(generator)
    }

    // ekf predict
    x_ekf = F * x_ekf; 
    P  = ( F * P * F.transpose() ) + ( W * P_sys * W.transpose() );

    // measurement
    Measurement m = x;
    for (int j = 0; j < n; j++) {
        m[i] += noiseLevel_meas;// * noise(generator)
    }

    // ekf update
    EigenSquare S = ( H * P * H.transpose() ) + ( V * P_meas * V.transpose() );
    EigenSquare Sinv = S;//.inverse();
    EigenSquare K = P * H.transpose() * Sinv;
    x_ekf += K * ( m - x_ekf );
    P -= K * H * P;
            
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
            A[n*i + j] = j == i ? 1.5 : 0.0;
            Adup[n*i + j] = 0.0;
            Adup_fd[n*i + j] = 0.0;
        }
    }

    double delta = 0.001;
    double delta2 = delta * delta;

    double fx = simulate(A);
    printf("f(A) = %f\n", fx);

    for (int i = 0; i < n * n; i++) {
        A[0] += delta;
        double fx2 = simulate(A);
        A[0] -= delta;
        Adup_fd[i] = (fx2 - fx) / delta;
    }

    printf("Adup_fd[0] = %f, Adup_fd[1] = %f, Adup_fd[2] = %f, Adup_fd[10] = %f\n", Adup_fd[0], Adup_fd[1], Adup_fd[2], Adup_fd[10]);

    __enzyme_autodiff<double>((void *)simulate, enzyme_dup, A, Adup);
    printf("Adup[0] = %f, Adup[1] = %f, Adup[2] = %f, Adup[10] = %f", Adup[0], Adup[1], Adup[2], Adup[10]);

    return 0;
}
