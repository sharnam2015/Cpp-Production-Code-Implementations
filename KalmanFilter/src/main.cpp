#include "KalmanFilter.hpp"
#include "KalmanFilter.cpp"

int main() {
     std::cout << "Main Program " << std::endl;
     data::Covar2D_t<3> sens_noise{};
     sens_noise.cov = {{{1.0f,0.0f,0.0f},{0.0,1.0f,0.0},{0.0f,0.0f,1.0f}}};

     filter::KalmanFilter<3> kf(data::Pose2D_t<3>{1.0f,0.0f,0.0f},sens_noise,1U);

     kf.showStateAndCovariance();
    //kf.predict(1.0f,1.0f,2U,data::Covar2D_t<3>{{1.0f}});

     filter::RobustKalman<3> rk(data::Pose2D_t<3>{1.0f,0.0f,0.0f},data::Covar2D_t<3>{{}},1U);
     
     /*
     try {
     rk.predict(2.0f, 1.0f,2U, sens_noise);
     rk.showStateAndCovariance();
     } catch (std::exception &e) {
     std::cout << e.what() << std::endl;
     }
     */

    kf.predict(1.0f,1.0f,2U,sens_noise);
    kf.showStateAndCovariance();

    return 0;
}