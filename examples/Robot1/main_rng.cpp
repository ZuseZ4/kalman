#include <iostream>
#include <random>

extern double __enzyme_autodiff(void *, double);

double simulate(double input) {
  std::default_random_engine generator;
  std::normal_distribution<float> noise(0.0, 1.0);
  
  return input * noise(generator);
}

int main(int argc, char **argv) {
    double df_dx = __enzyme_autodiff((void *)simulate, 1.0);
    printf("df_dx = %f", df_dx);

    double x = simulate(1.0);

    return 0;
}
