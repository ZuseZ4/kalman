// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>

extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

#include <assert.h>

void __enzyme_autodiff(...);

const size_t n = 10;

typedef double T;

#include "/home/wmoses/git/Enzyme/enzyme/test/Integration/blas_inline.h"

#define EIGEN_USE_BLAS 1

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
typedef Eigen::Matrix<double, n, n> EigenSquare;
static char N = 'N';
static int ten = 10;
static double one = 1.0;
static double zero = 0.0;
double simulate(EigenSquare &P) {
  auto P1  = P * P * P;

  // double *out = (double*)malloc(sizeof(double)*n*n);
  // dgemm_(&N, &N, &ten, &ten, &ten, &one, P1.data(), &ten, P.data(), &ten, &zero, &out[0], &ten);
  return P1(0, 0);
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

    double fx = simulate(*(EigenSquare*)(&A[0]));
    printf("f(A) = %f\n", fx);
   
    if (argc == 2) {
    __enzyme_autodiff((void *)simulate, enzyme_dup, &A[0], &Adup[0]);
    printf("dP(0,0) = %f, dP(0,1) = %f, dP(0,2) = %f\n", Adup[0], Adup[1], Adup[2]);
    }

    for (int i = 0; i < n; i++) {
        A[i] += delta / 2;
        double fx2 = simulate(*(EigenSquare*)(&A[0]));
        A[i] -= delta;
        double fx3 = simulate(*(EigenSquare*)(&A[0]));
        A[i] += delta/2;
        Adup_fd[i] = (fx2 - fx3) / delta;
    }

    printf("dP(0,0) = %f, dP(0,1) = %f, dP(0,2) = %f\n", Adup_fd[0], Adup_fd[1], Adup_fd[2]);

    return 0;
}
