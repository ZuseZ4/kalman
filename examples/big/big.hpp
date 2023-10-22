#ifndef KALMAN_EXAMPLES1_BIG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_BIG_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>
#include <iostream>
#include <random>

namespace KalmanExamples
{
namespace Big
{

const size_t n = 100;

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, n>
{
public:
    KALMAN_VECTOR(State, T, n)
};

template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Big::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Big::Control<T> C;

    T noiseLevel = 0.1;

    // SystemModel(const Kalman::Jacobian<State<T>, State<T>> A): noise(0, 1)
    SystemModel()//: noise(0, 1)
    {
        // generator.seed(1);
        auto P = this->getCovariance();
        for (int i = 0; i < n; i++) {
            P(i, i) = std::pow(noiseLevel, 2);
        }

        // in current interface, let F be set by user.

        // for (int i = 0; i < n; i++) {
        //     for (int j = 0; j < n; j++) {
        //         this->F(i, j) = A(i, j);
        //     }
        // }
    }
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        S x_;

        // TODO: avoid the copy? not so important, since we can at least update jacobians..
        // Since we make a purely linear model, F plays a double role as A. 
        x_ = this->F * x;

        // for (int i = 0; i < n; i++) {
        //     // write a manual matrix multiply
        //     x_[i] = 0;
        //     for (int j = 0; j < n; j++) {
        //         x_[i] += this->F(i, j) * x[j];
        //     }
        // }

        for (int i = 0; i < n; i++) {
            x_[i] += noiseLevel;// * noise(generator);
        }

        return x_;
    }

    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)

        // in current interface, let F be set by user.

        // TODO: reconsider unitary, which makes F' F = I in ekf.predict.
        // this->F(0, 0) = std::cos(u[0]); 
        // this->F(0, 1) = -std::sin(u[0]);
        // this->F(1, 0) = std::sin(u[0]);
        // this->F(1, 1) = std::cos(u[0]);

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
    }
};

template<typename T>
class Measurement : public Kalman::Vector<T, n>
{
public:
    KALMAN_VECTOR(Measurement, T, n)
};

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
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Big::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Big::Measurement<T> M;

    // mutable std::default_random_engine generator;
    // mutable std::normal_distribution<T> noise;
    
    MeasurementModel()//: noise(0, 1)
    {
        // generator.seed(1);
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->H *= 0.1;
        this->V.setIdentity(); // TODO: what is V?
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
    M h(const S& x) const
    {
        M measurement;
        
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