
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
// template <typename return_type, typename... T>
// return_type __enzyme_fwddiff(void *, T...);

// template <typename return_type, typename... T>
// return_type __enzyme_autodiff(void *, T...);

extern double __enzyme_autodiff(void *, double);
// double foo(double x) { return x * x; }

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

// using Eigen::MatrixXd;
// using Eigen::VectorXd;
//
// void __enzyme_autodiff2(void *, ...);
// void bar(MatrixXd *m, VectorXd *v) { *v = *m * *v; }

double simulate(double input) {
  // Simulated (true) system state
  State x;
  x.setZero();

  // Control input
  Control u;
  // System
  SystemModel sys;

  // Measurement models
  // Set position landmarks at (-10, -10) and (30, 75)
  PositionModel pm(-10, -10, 30, 75);
  OrientationModel om;

  // Random number generation (for noise simulation)
  // std::default_random_engine generator;
  // generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  // std::normal_distribution<T> noise(0, 1);

  // Some filters for estimation
  // Pure predictor without measurement updates
  Kalman::ExtendedKalmanFilter<State> predictor;
  // Extended Kalman Filter
  Kalman::ExtendedKalmanFilter<State> ekf;

  // Init filters with true system state
  predictor.init(x);
  ekf.init(x);

  // Standard-Deviation of noise added to all state vector components during
  // state transition
  T systemNoise = 0.1;
  // Standard-Deviation of noise added to all measurement vector components in
  // orientation measurements
  T orientationNoise = 0.025;
  // Standard-Deviation of noise added to all measurement vector components in
  // distance measurements
  T distanceNoise = 0.25;

  double ekfy_sum = 0.0;
  // Simulate for 100 steps
  const size_t N = 100;
  const double dN = N;
  for (size_t i = 1; i <= N; i++) {
    // Generate some control input
    u.v() = input;// + std::sin(T(2.0) * T(M_PI) / T(dN));
    // u.dtheta() = 2.0;// std::sin(T(2.0) * T(M_PI) / T(dN)) * (1.0);// - 2.0 * (i > 50));

    // // // // Simulate system
    // x = sys.f(x, u);

    // // // // Add noise: Our robot move is affected by noise (due to actuator failures)
    // x.x() += systemNoise * 0.05; // 0.05;
    // x.y() += systemNoise * 0.05;
    // x.theta() += systemNoise * 0.05;

    // // Predict state for current time-step using the filters
    auto x_pred = predictor.predict(sys, u);
    // u.setZero();
    // auto x_ekf = ekf.predict(sys, u);
    // // auto x_ukf = ukf.predict(sys, u);

    // // Orientation measurement
    // {
    //   // We can measure the orientation every 5th step
    //   OrientationMeasurement orientation = om.h(x);

    //   // Measurement is affected by noise as well
    //   orientation.theta() += orientationNoise * 0.05;

    //   // Update EKF
    //   x_ekf = ekf.update(om, orientation);

    //   // Update UKF
    //   // x_ukf = ukf.update(om, orientation);
    // }

    // // Position measurement
    // {
    //   // We can measure the position every 10th step
    //   PositionMeasurement position = pm.h(x);

    //   // Measurement is affected by noise as well
    //   position.d1() += distanceNoise * 0.05;
    //   position.d2() += distanceNoise * 0.05;

    //   // Update EKF
    //   x_ekf = ekf.update(pm, position);

    //   // Update UKF
    //   // x_ukf = ukf.update(pm, position);
    // }

    ekfy_sum += u.v();//x_ekf.y();

  //   // Print to stdout as csv format
  //   std::cout << x.x() << "," << x.y() << "," << x.theta() << "," << x_pred.x()
  //             << "," << x_pred.y() << "," << x_pred.theta() << "," << x_ekf.x()
  //             << "," << x_ekf.y() << "," << x_ekf.theta() //<< "," << x_ukf.x()
  //             << std::endl;
  //             //<< "," << x_ukf.y() << "," << x_ukf.theta() << std::endl;
  }
  return ekfy_sum;// / (double)N;
}

int main(int argc, char **argv) {
    // MatrixXd m = MatrixXd::Random(30, 30);
    // MatrixXd dm = MatrixXd::Random(30, 30);
    // m = (m + MatrixXd::Constant(30, 30, 1.2)) * 50;
    // std::cout << "m =" << std::endl << m << std::endl;
    // VectorXd v = VectorXd::Random(30);
    // VectorXd dv = VectorXd::Random(30);
    //__enzyme_autodiff2((void *)bar, &m, &dm, &v, &dv);
    // std::cout << "dm, dv: =" << std::endl << dm << std::endl << dv <<
    // std::endl;
    // }

    double x1 = simulate(1.0);
    double x2 = simulate(1.1);
    std::cout << "x1: " << x1 << ", x2: " << x2 << std::endl;

    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    double df_dx2 = __enzyme_autodiff((void *)simulate, 1.1);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.0, x1, df_dx1);
    printf("x = %f, f(x) = %f, f'(x) = %f", 1.1, x2, df_dx2);

    return 0 + 1;
}
