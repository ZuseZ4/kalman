#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>

extern double __enzyme_autodiff(void *, double);
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
    double df_dx = __enzyme_autodiff((void *)simulate, 1.0);

    return 0;
}
