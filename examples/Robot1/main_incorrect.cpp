#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include <iostream>
#include <Eigen/Core>

extern double __enzyme_autodiff(void *, double);

typedef float T;

typedef Eigen::Matrix<T, 3, 1> State;
typedef Eigen::Matrix<T, 2, 1> Control;

struct MyStruct {
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

void predict(MyStruct& ekf, SystemModel& sys, const Control& u ) {
    ekf.x = sys.f(ekf.x, u);
    sys.W  = ( sys.W * sys.W * sys.W.transpose() ) + ( sys.W * sys.W * sys.W.transpose() );
}

double simulate(double input) {
  State x;
  x[0] = 0; x[1] = 0; x[2] = 0;

  Control u;
  SystemModel sys;

  MyStruct ekf;
  ekf.x = x;

  double ekfy_sum = 0.0;
  for (size_t i = 1; i < 3; i++) {
    u[0] = input;
    u[1] = 2.0;

    predict(ekf, sys, u);

    ekfy_sum += ekf.x.y();
  }

  return ekfy_sum;
}

int main(int argc, char **argv) {
    double fx1 = simulate(1.0);
    double fx2 = simulate(1.1);
    printf("x = %f, f(x) = %f\n", 1.0, fx1);
    printf("x = %f, f(x) = %f\n", 1.1, fx2);

    double df_dx1 = __enzyme_autodiff((void *)simulate, 1.0);
    double df_dx2 = __enzyme_autodiff((void *)simulate, 1.1);
    printf("x = %f, f(x) = %f, f'(x) = %f\n", 1.0, fx1, df_dx1);
    printf("x = %f, f(x) = %f, f'(x) = %f\n", 1.1, fx2, df_dx2);

    return 0;
}