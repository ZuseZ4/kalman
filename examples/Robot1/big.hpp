#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <iostream>
#include <random>

namespace KalmanExamples
{
namespace Robot1
{

const size_t n = 2;

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
	typedef KalmanExamples::Robot1::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Robot1::Control<T> C;

    mutable std::default_random_engine generator;
    mutable std::normal_distribution<T> noise;

    T noiseLevel = 0.1;

    SystemModel(const Kalman::Jacobian<State<T>, State<T>> A): noise(0, 1)
    {
        generator.seed(1);
        auto P = this->getCovariance();
        for (int i = 0; i < n; i++) {
            P(i, i) = std::pow(noiseLevel, 2);
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                this->F(i, j) = A(i, j);
            }
        }
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
        x_[0] = std::cos(u[0]) * x[0] - std::sin(u[0]) * x[1];
        x_[1] = std::sin(u[0]) * x[0] + std::cos(u[0]) * x[1]; 

        x_[0] += noiseLevel * noise(generator);
        x_[1] += noiseLevel;

        // Since we make a purely linear model, F plays a double role as A. 
        return x_;
    }

    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        // TODO: reconsider unitary, which makes F' F = I in ekf.predict.
        this->F(0, 0) = std::cos(u[0]); 
        this->F(0, 1) = -std::sin(u[0]);
        this->F(1, 0) = std::sin(u[0]);
        this->F(1, 1) = std::cos(u[0]);

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif