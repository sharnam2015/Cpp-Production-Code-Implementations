#include <fusion.hpp>
#include <cstdint>
#include <iostream>

using namespace Eigen;

int main()
{
    Eigen::Matrix<float,12,1> v{0.0014f,0.0014f,0.0014f,0.004f,0.004f,0.004f,0.0005f,0.0005f,0.0005f,0.02f,0.02f,0.02f};

  MatrixXf R(5,5);
   
   R.setZero();
   float gps_sigma_v = 0.1f;
   float gps_sigma_ure = 0.5f;

  // Ignoring the Non diagnonal terms on the DOP matrix just for now
   R(0,0) = pow(gps_sigma_ure,2);
   R(1,1) = pow(gps_sigma_ure,2);
   R(2,2) = pow(gps_sigma_ure,2);
   R(3,3) = pow(gps_sigma_ure,2);
   R(4,4) = pow(gps_sigma_ure,2);   

   //Setting the initial State and Covariance
   Eigen::Vector<float,15> errx_i = Eigen::Vector<float,15>::Zero();
   Eigen::Matrix<float,15,15> errcov_i = 0.1*Eigen::Matrix<float,15,15>::Identity();

  fusion::Kalman kf(errx_i,errcov_i,0.0);

  std::cout << "CHECKING STATUS" <<  kf.isInitialized() <<std::endl;

  Eigen::Vector3f acc{0.1f,0.1f,0.1f};
  Eigen::Vector3f omg{0.2f,0.2f,0.2f};
 std::cout << "CHECKING PRINT LINE 50\n" << std::endl;


  kf.predict( acc,omg,0.1f);

  
  Eigen::Vector<float,5> z1{0.1,0.1,0.01,0.01,0.01};
  kf.update(z1,R,0.12f);

  auto stat = std::get<0>(kf.getStateAndCovar());

 std::cout << "STATE\n" << stat << std::endl;
  
    return 0;
}
