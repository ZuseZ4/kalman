#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>

extern double __enzyme_autodiff(void *, double);

typedef float T;

typedef Eigen::Matrix<T, 3, 1> State;
typedef Eigen::Matrix<T, 2, 1> Control;

typedef Eigen::Matrix<T, 1, 1> OrientationMeasurement;

struct ExtendedKalmanFilter {
  Eigen::Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime> P;
  State x;
};

class KalmanSystemModel
{
    public:
        virtual State f(const State& x, const Control& u) const = 0;
    protected:
        KalmanSystemModel() {}
        virtual ~KalmanSystemModel() {}
};

class LinearizedSystemModel : public KalmanSystemModel
{
    public:
        Eigen::Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime> W;
};

class SystemModel : public LinearizedSystemModel
{
public:
    State f(const State& x, const Control& u) const {
        State x_;
        auto newOrientation = x[2] + u[1];
        x_[2] = newOrientation;
        x_[0] = x[0] + std::cos( newOrientation ) * u[0];
        x_[1] = x[1] + std::sin( newOrientation ) * u[0];
        return x_;
    }
};

void predict(ExtendedKalmanFilter& ekf, SystemModel& sys, const Control& u ) {
    ekf.x = sys.f(ekf.x, u);
    ekf.P  *= ( sys.W * ekf.P * sys.W.transpose() ) + ( sys.W * sys.W * sys.W.transpose() );
}

double simulate(double input) {
  State x;
  x.setZero();

  Control u;
  SystemModel sys;

  ExtendedKalmanFilter predictor;
  ExtendedKalmanFilter ekf;

  predictor.x = x;
  ekf.x = x;

  u[0] = input;
  u[1] = 2.0;

  predict(predictor, sys, u);
  predict(ekf, sys, u);

  OrientationMeasurement orientation;
  Eigen::Matrix<T, OrientationMeasurement::RowsAtCompileTime, State::RowsAtCompileTime> H;
  Eigen::Matrix<T, State::RowsAtCompileTime, OrientationMeasurement::RowsAtCompileTime> K = ekf.P * H.transpose();
  ekf.x += K * orientation;
  ekf.P -= K * H * ekf.P;

  return 0.0;
}

int main(int argc, char **argv) {
    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    return 0;
}