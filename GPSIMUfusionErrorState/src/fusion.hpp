#pragma once
#include<Eigen/Dense>
#include<cstdint>
#include <tuple>

namespace fusion{
//Implementing an Error State Kalman Filter (Loosely coupled) for GPS IMU fusion
// State is the following [misalignment angles (3), error in velocity (3), error in position(3), error in gyro bias (3), error in accelerometer bias (3)]
// The Kalman Filter is used in the NED frame
class Kalman
{

//Defining the default constructors below
public:
Kalman(Eigen::Vector<float,15> x_i,Eigen::Matrix<float,15,15> cov_i,float t_i);
Kalman(Kalman &other)=default;
Kalman(Kalman &&other) noexcept = default;
Kalman& operator=(Kalman &other)=default;
Kalman& operator=(Kalman &&other) noexcept =default;

//@brief - initializes the Kalman Filter
void initialize(Eigen::Vector<float,15> x_i,Eigen::Matrix<float,15,15> cov_i,float t_i);

//@brief - Runs the prediction step of the Error State Kalman Filter for the INS mechanization
//Cnb = (I-Y)-1 * Cnbest;
//Matrix3f Cnb;
//Y = [0,-delgam,-delBet,
//      delgam,0,-delalpha,
//     -delbeta,delalpha,0]
// del xk+1 = delxk(I+AT);
// Q(k) = (GT)Q(k-1)(GT.transpose())
//A =
//[0,           0,0, -Cnb, 0,
//acc_ned_skew, 0,0,  0,  Cnb,
//0            ,I,0,  0,   0,
//0            ,0,0,  0,   0,
//0            ,0,0,  0,   0]
//Note that 0 in the above most of the times indicates 0(3x3) matrix
//A = I+AT

//G = [−Cnb 0 0 0 
//     0 Cnb 0 0 
//     0 0 0 0 
//     0 0 I 0 ]
//Here as well most of the o indicate o(3x3) matrix
void predict(Eigen::Vector3f acc,Eigen::Vector3f omegas, float t_p);

//brief - runs the update step of the Error State Kalman and also calls the corrector to correct the total state using the new errors
// Updates the propogated INS mechanization with the GPS measurement
//GPS measures - [Vn, Ve, Pn, Pe, Pd] - 5 quantities
//z = Hdelx + R_meas
//zIMU =[vNIMU vEIMU pNIMU pEIMU pDIMU]
//delz= [vNGPS - vNIMU,
//      vEGPS - vEIMU,
//      pNGPS - pNIMU,
//      pEGPS - pEIMU,
//      pDGPS - pDIMU]
void update(Eigen::Vector<float,5> z_meas,Eigen::Matrix<float,5,5> R_meas,float t_meas);

//Propogates the inertial sensor measurement to align that time with the measurement timestamp , assumes a constant previous velocity, angular velocity
void propogate(float t_curr);

//@brief - checks whether the Error State Kalman Filter is Initialized or not
bool isInitialized();

//@brief - gets the error state, covariance and time
std::tuple<Eigen::Vector<float,15>,Eigen::Matrix<float,15,15>,float> getErrorStateAndCovar();

//@brief - gets the total stored state, covariance and time
std::tuple<Eigen::Vector<float,15>,Eigen::Matrix<float,15,15>,float> getStateAndCovar();

protected:
Eigen::Vector<float,15> errx_m;
Eigen::Matrix<float,15,15> errcov_m;
float t_m;
bool is_initialized_m;
Eigen::Vector<float,15> x_m;
Eigen::Vector3f prev_omegas_m;
Eigen::Matrix3f R_stor_m;

};    

}

