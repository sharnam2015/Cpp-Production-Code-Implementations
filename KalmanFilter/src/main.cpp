#include "KalmanFilter.hpp"
#include "KalmanFilter.cpp"
#include <functional>

int main() {
     std::cout << "Main Program " << std::endl;
     data::Covar2D_t<3> sens_noise{};
     sens_noise.cov = {{{1.0f,0.0f,0.0f},{0.0,1.0f,0.0},{0.0f,0.0f,1.0f}}};

     filter::KalmanFilter<3> kf(data::Pose2D_t<3>{1.0f,0.0f,0.0f},sens_noise,1U);

     
    kf.predict(1.0f,1.0f,2U,sens_noise);
    kf.showStateAndCovariance();
    std::cout << "KF PREDICT DONE\n" ;

     filter::RobustKalman<3> rk(data::Pose2D_t<3>{1.0f,0.0f,0.0f},sens_noise,1U);

     try {
     rk.predict(2.0f, 1.0f,2U, sens_noise);
     rk.showStateAndCovariance();
     } catch (std::exception &e) {
     std::cout << e.what() << std::endl;
     }
    

    //kf.predict(1.0f,1.0f,2U,sens_noise);
    //kf.showStateAndCovariance();
    
    auto kfptr1 = std::make_shared<filter::KalmanFilter<3>>(kf);
    kfptr1->showStateAndCovariance();
    std::cout << "KF POINTER COVAR ABOVE\n";
    kfptr1->predict(1.0f,1.0f,3U,sens_noise);
    kfptr1->showStateAndCovariance();

    filter::RobustKalman<3> rkf(kfptr1);

    auto rkfptr1 = std::make_shared<filter::RobustKalman<3>>(rkf);
    rkfptr1->predict(1.0f,1.0f,4U,sens_noise);
    rkfptr1->showStateAndCovariance();
    
    std::cout << "FINAL KF STATE AND COVAR\n";
    kf.showStateAndCovariance();

    std::cout << "FINAL RKF STATE AND COVAR\n";
    rkfptr1->predict(1.0f,1.0f,5U,sens_noise);
    rkf.showStateAndCovariance();

    auto rkfptr2=std::make_shared<filter::RobustKalman<3>>(rkf);
    rkfptr2->predict(1.0f,1.0f,10U,sens_noise);
    rkfptr1->showStateAndCovariance();
    rkfptr2->showStateAndCovariance();

    auto rkfptr3 = rkfptr1;
    rkfptr3->predict(1.0f,1.0f,10U,sens_noise);
    rkfptr1->showStateAndCovariance();
    
    auto rkfptr4=std::make_unique<filter::RobustKalman<3>>(rkf);
    rkfptr4->showStateAndCovariance();
    rkfptr4->predict(1.0f,1.0f,5U,sens_noise);
    auto rkfptr5 = std::move(rkfptr4);
    
    if(rkfptr4==nullptr)
    {
        std::cout << "\nNULL CHECK SUCCESSFUL\n";
    }
    
    rkfptr5->predict(1.0,1.0,6U,sens_noise);
    rkfptr5->showStateAndCovariance();
    auto rkfptr6 = std::make_unique<filter::RobustKalman<3>>(rkf);
    //rkfptr5=nullptr;

    filter::RobustKalman<3>*p = &rkf;
    p->predict(1.0,1.0,10U,sens_noise);
    p->showStateAndCovariance();
    //auto rkfptr7 = std::unique_ptr<filter::RobustKalman<3>>(&rkf)// Need to delete the delete default function first
    rkf.showStateAndCovariance();

    std::cout << "\n POINTER COUNTS\n" << "rk1ptr1: " << rkfptr1.use_count() << "\nrkfptr2:" << rkfptr2.use_count() << "\nrkfptr3:" << rkfptr3.use_count();
    
    using State = decltype(rkf.getState());
    
    //auto could also be used for type erasure here below
    std::function<State(float)> statmul = [&](auto k) noexcept -> State{
        auto stat = rkf.getState();
        for(size_t i=0;i<stat.first.X.size();++i)
        {
            stat.first.X[i]=k*stat.first.X[i];
        }

        return stat;
    };


    State scaled_stat = statmul(2.0f);
    //auto stat_curr = rkf.getState().first.X;
    auto stat_curr = scaled_stat.first.X;
    for(size_t i=0;i<stat_curr.size();++i)
    {
        std::cout<< "State Element " << stat_curr.at(i) << std::endl;
    }

    return 0;
}