#ifndef KALMAN_EXAMPLES1_BIG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_BIG_SYSTEMMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <iostream>
#include <random>

namespace KalmanExamples
{
namespace Big
{

const size_t n = 5;


/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<Eigen::Matrix<T, n, 1>, Eigen::Matrix<T, n, 1>, CovarianceBase>
{
public:
    //! State type shortcut definition
    
    //! Measurement type shortcut definition
    // mutable std::default_random_engine generator;
    // mutable std::normal_distribution<T> noise;
    
    MeasurementModel()//: noise(0, 1)
    {
        // generator.seed(1);
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
       // this->H.setIdentity();
       // this->H *= 0.1;
       // this->V.setIdentity(); // TODO: what is V?
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    Eigen::Matrix<T,n, 1> h(const Eigen::Matrix<T, n, 1>& x) const
    {
        Eigen::Matrix<T,n, 1> measurement;
        
        measurement = x;

        for (int i = 0; i < n; i++) {
            measurement[i] += 0.1;// * noise(generator);
        }
        
        return measurement;
    }
};

}
}

#endif
