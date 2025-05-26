#pragma once
#include <cstdint>
#include "Data.hpp"
#include <stdexcept>
#include <memory>

namespace filter{
template<int N>
// Normal Standard Kalman Filter Class with Kalman predict , update steps, initialization check and getter functions
class KalmanFilter
{
public:
KalmanFilter();
explicit KalmanFilter(data::Pose2D_t<N> X,data::Covar2D_t<N> covin,uint32_t tim);
KalmanFilter(KalmanFilter&) = default;
KalmanFilter(KalmanFilter&&) noexcept= default;
KalmanFilter& operator=(KalmanFilter&) = default;
KalmanFilter& operator=(KalmanFilter&&) noexcept = default;

//@brief - Fetches the initialization status of the Kalman Filter
//@returns - [bool] - true or false status of initialization
bool getStatus() const;

//@brief - Prints out the current state of the Kalman Filter, pose and time 
void showStateAndCovariance() const;

//@brief - Outputs the current pose, time state of the Kalman Filter
//@returns - [pair<data::Pose2D_t,uint32_t>] - pose and time of the current Kalman Filter state as a pair
std::pair<data::Pose2D_t<N>,uint32_t> getState() const;

//@brief - Outputs the Covariance of the current Pose state in the Kalman Filter
//@returns - [data::Covar2D_t] - Covariance of the current Pose of the Kalman Filter
data::Covar2D_t<N> getCovariance() const;

//@brief - Initializes the Kalman Filter using a given initialization state, covariance and time
//@params - [data::Pose2D_t] - Initial Pose
//@params - [data::Covar2D_t] - Initial Covariance
//@params - [uint32_t] - initial state time
void initialize(const data::Pose2D_t<N> &X, const data::Covar2D_t<N> &Cov, const uint32_t &tim);

//@brief - Kalman Filter Predict step, assumed to be x(k) = x(k-1)+vdeltat,y(k) = y(k-1)+vdeltat, phi(k) = phi(k-1)+omegadeltat 
//@brief - predict runs the Kalman Filter predict step with speed, omega inputs propogating the state and covariance
//@params - [float] - vel, speed input
//@params - [float] - omega, angular velocity input
//@params - [uint32t] - t,time of speed and omega measurements
virtual void predict(const float &vel, const float &omega, const uint32_t &t,const data::Covar2D_t<N> &sens_noise);

//@brief - Kalman Filter update step, directly measures pose and a certain time
//@params - [data::Pose_t] - measurement of pose
//@params - [uint32_t] - time of measurement in (s)
virtual void update(data::Pose2D_t<N> &z,const uint32_t &mtime,const data::Covar2D_t<N> &sens_noise);

protected:
data::Pose2D_t<N> X_m;
data::Covar2D_t<N> Cov_m;
uint32_t t_m;
bool status_m;

};

//RobustKalman implements a more Robust version of the Kalman Filter
template<int N>
class RobustKalman:public KalmanFilter<N>
{
public:

//Using the same constructors and assignment operators as the KalmanFilter Class
using KalmanFilter<N>::KalmanFilter;

RobustKalman (std::shared_ptr<KalmanFilter<N>> other) //Can keep as just a normal pointer but in that case will need
//to convert the input shared_pointer to the raw pointer via .get()
{
    this->Cov_m = other->getCovariance();
    this->X_m = other->getState().first;
    this->t_m = other->getState().second;
    this->status_m = other->getStatus();
}

//@brief - Robust Kalman Filter Predict step, assumed to be x(k) = x(k-1)+vdeltat,y(k) = y(k-1)+2*vdeltat, phi(k) = phi(k-1)+omegadeltat 
//@brief - Adds some checks and applies the prediction step of a Kalman Filter more Robustly
//@params - [float] - vel, speed input
//@params - [float] - omega, angular velocity input
//@params - [uint32t] - t,time of speed and omega measurements
void predict (const float &vel,const float &omega,const uint32_t &t,const data::Covar2D_t<N> &sens_noise) override;

//@brief - Robust Kalman Filter update/Measurement step, measurement directly measures pose. z(k) = X(k)+w. Updates the state and covariance
//@brief - The Robust Kalman Filter measurement update performs some checks and the Joseph Form of The Covariance update for robustness 
//@params - [data::Pose_t] - measurement of pose in (m), (m), (rad)
//@params - [uint32_t] - time of measurement in (s)
void update(data::Pose2D_t<N> &z,const uint32_t &mtime,const data::Covar2D_t<N> &meas_noise) override;
};

template<int N>
//@brief - Operator Overloading for the * operator to multiply to covariance sized 2D arrays
//@params - [data::Covar2D_t] - other1 - Input Covariance Matrix 2D array
//@params - [data::Covar2D_t] - other2 - Input Covariance Matrix 2D array
//@returns - [data::Covar2D_t] - Output Matrix which is the multiplication of the two input matrices
data::Covar2D_t<N> operator*(const data::Covar2D_t<N> &other1 , const data::Covar2D_t<N> &other2);

template<int N>
//@brief - Operator Overloading for the * operator to multiply covariance and state vector 1D array
//@params - [data::Covae2D_t] - Covariance matrix input
//@params - [data::Pose2D_t] - State Vector 1D array input
//@returns - [data::Pose2D_t] - Output 1D array which is the Multiplication of the input 2D array and the input 1D array
data::Pose2D_t<N> operator*(const data::Covar2D_t<N> &mat, const data::Pose2D_t<N> &stat);

//@brief - Operator Overloading for the + operator to add two covariances (2 2D arrays)
//@params - [data::Covae2D_t] - Covariance matrix input
//@params - [data::Covae2D_t] - Covariance matrix input
//@returns - [data::Covar2D_t] - Output Covariance 2D array which is the addition of the two input 2D arrays
template<int N> 
data::Covar2D_t<N> operator+(const data::Covar2D_t<N> &mt1, const data::Covar2D_t<N> &mt2);

//@brief - Operator Overloading for the + operator to add two 1D struct arrays
//@params - [data::Pose2D_t] - Pose struct input 1
//@params - [data::Pose2D_t] - Pose struct input 2
//@returns - [data::Pose2D_t] - Output 1D array struct which is the addition of the two input struct 1D arrays
template<int N>
data::Pose2D_t<N> operator+(const data::Pose2D_t<N> &p1,const data::Pose2D_t<N> &p2);

//@brief - Operator Overloading for the - operator to subtract two covariance structs(2 2D arrays)
//@params - [data::Covar2D_t] - Covariance struct 2D array input
//@params - [data::Covar2D_t] - Covariance struct 2D array input
//@returns - [data::Covar2D_t] - Output Covariance 2D array which is the subytraction of the two input 2D struct arrays
template<int N>
data::Covar2D_t<N> operator-(const data::Covar2D_t<N> m1, const data::Covar2D_t<N> m2);

//@brief - Takes the transpose of an input 2D array struct
//@params - [data::Covar2D_t] - Covariance struct 2D  input
//@returns - [data::Covar2D_t] - Output Covariance 2D array struct which is the transpose of the input 2D array
template<int N>
data::Covar2D_t<N> trans(const data::Covar2D_t<N> &mtin);

//@brief - Takes the inverse of a diagonal 2D struct array input
//@params - [data::Covar2D_t] - Covariance struct 2D array input
//@returns - [data::Covar2D_t] - Output Covariance 2D array struct which is the inverse of the input 2D array
template<int N>
data::Covar2D_t<N> diagInverse(const data::Covar2D_t<N> &m1);
}