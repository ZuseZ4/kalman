#define _USE_MATH_DEFINES
#include <cmath>

#define EIGEN_USE_BLAS

#include <Eigen/Dense>

template<typename RT, typename... Args>
RT __enzyme_autodiff(void*, Args...);

double simulate() {
  Eigen::Matrix<float, 5, 5> H;
  H.setIdentity();
  auto K = H.inverse();
  return K(0,0);
}

int main(int argc, char **argv) {
    __enzyme_autodiff<double>((void *)simulate);

    return 0;
}
