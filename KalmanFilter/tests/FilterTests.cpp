#include "../src/KalmanFilter.hpp"
#include "../src/KalmanFilter.cpp"
#include <gtest/gtest.h>

//filter::KalmanFilter<3> kf(data::Pose2D_t<3>{0.1f,0.2f,1.0f},data::Covar2D_t<3>{{}},4U);

class FilterTest : public ::testing::Test {
    protected:

    data::Covar2D_t<3> P{
                            {{
                                {1.0f,0.0f,0.0f},
                                {0.0,1.0f,0.0},
                                {0.0f,0.0f,1.0f}
                            }}
                        }; 
    filter::KalmanFilter<3> kf{data::Pose2D_t<3>{0.1f,0.2f,1.0f},P,1U};
    filter::RobustKalman<3> rkf{data::Pose2D_t<3>{1.0f,0.0f,1.0f},P,1U};
    };

TEST_F(FilterTest, status) {
    EXPECT_TRUE(kf.getStatus());
}

TEST_F(FilterTest,correct__initial_state)
{
  auto Stat  =kf.getState();
  auto arr = Stat.first.X;

  
  EXPECT_FLOAT_EQ(arr.at(0), 0.1f); 
  EXPECT_FLOAT_EQ(arr.at(1),0.2f);
  EXPECT_FLOAT_EQ(arr.at(2),1.0f);
  EXPECT_EQ(Stat.second,1U);//4U);
}

TEST_F(FilterTest,correct_prediction)
{   
    data::Covar2D_t<3> sens_noise{};
    
    for(size_t i=0;i<3U;i++)
    {
    sens_noise.cov[i][i]=1.0f;
    }

    kf.predict(1.0f,1.0f,2U,sens_noise);
    auto state = kf.getState();
    EXPECT_EQ(state.second,2U);
    EXPECT_FLOAT_EQ(state.first.X[0],1.1f);
    EXPECT_FLOAT_EQ(state.first.X[1],1.2f);
    EXPECT_FLOAT_EQ(state.first.X[2],2.0f);

    auto covar = kf.getCovariance();
    EXPECT_FLOAT_EQ(covar.cov[0][0],2.0f);
    EXPECT_FLOAT_EQ(covar.cov[1][1],2.0f);
    EXPECT_FLOAT_EQ(covar.cov[2][2],2.0f);
    EXPECT_FLOAT_EQ(covar.cov[2][0],0.0f);

    EXPECT_THROW(rkf.predict(10.0f,20.0f,3U,sens_noise),std::invalid_argument);
}


