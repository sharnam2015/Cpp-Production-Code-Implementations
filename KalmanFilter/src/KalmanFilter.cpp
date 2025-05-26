#include "KalmanFilter.hpp"
#include<iostream>

namespace filter{

template<int N>
KalmanFilter<N>::KalmanFilter()
{
  t_m = 0LU;
  status_m = false;
  Cov_m.cov={{0.0f}};
}


template<int N>
KalmanFilter<N>::KalmanFilter(data::Pose2D_t<N> X,data::Covar2D_t<N> covin,uint32_t tim): X_m(X),Cov_m(covin),t_m(tim),status_m(true){
  
}

template<int N>
void KalmanFilter<N>::showStateAndCovariance() const
{
  if(status_m)
  {
  for(size_t i=0;i<X_m.X.size();++i)
  {
    std::cout << "STATE ELEMENT" << i<< X_m.X.at(i) << std:: endl;
  }
  std::cout << "\n Covariance \n" ;

  for(size_t i=0;i<Cov_m.cov.size();++i)
  {
    for(size_t j=0;j<Cov_m.cov.at(0).size();++j)
    {
        std::cout << Cov_m.cov.at(i).at(j) << "," ;
    }
    std::cout << std::endl;
  }

  std::cout << "\nTime" << t_m << std::endl;
  }
  else
  {
    std::cout << "INITIALIZE FILTER FIRST" << std::endl;
  }
}

template<int N>
std::pair<data::Pose2D_t<N>,uint32_t> KalmanFilter<N>::getState() const
{
  return std::make_pair(X_m,t_m);
}

template<int N>
data::Covar2D_t<N> operator*(const data::Covar2D_t<N> &other1, const data::Covar2D_t<N> &other2)
{
    data::Covar2D_t<N> op{};
    op.cov = {{0.0f}};
    for(size_t i=0;i<other1.cov.size();++i)
    {
        for(size_t k=0;k<other2.cov.size();++k)
        {
        for(size_t j=0;j<other1.cov.at(0).size();++j)
        {
            op.cov[i][j]+=(other1.cov[i][j]*other2.cov[j][k]);
        }
        }
    }
    return op;
}

template<int N> 
data::Pose2D_t<N> operator*(const data::Covar2D_t<N> &mat, const data::Pose2D_t<N> &stat)
{
data::Pose2D_t<N> op{};
op.X = {0.0};

for(size_t i=0;i<mat.cov.size();++i)
{
for(size_t j=0;j<mat.cov.at(0).size();++j)
{
    op.X[i]+=mat.cov[i][j]*stat.X[j];
}
}
return op;
}

template<int N> 
data::Covar2D_t<N> operator+(const data::Covar2D_t<N> &mt1, const data::Covar2D_t<N> &mt2)
{
    data::Covar2D_t<N> op{};
    op.cov= {{0.0f}};

    for(size_t i=0;i<mt1.cov.size();i++)
    {
        for(size_t j=0;j<N;j++)
        {
            op.cov[i][j]=mt1.cov[i][j]+mt2.cov[i][j];
        }
    }
    return op;
}

template<int N>
data::Covar2D_t<N> operator-(const data::Covar2D_t<N> m1, const data::Covar2D_t<N> m2)
{
    data::Covar2D_t<N> op{};
    op = {{0.0f}};
    for(size_t i=0;i<m1.cov.size();++i)
    {
        for(size_t j=0;j<m1.cov.at(0).size();++j)
        {
            op.cov[i][j]=m1.cov[i][j]-m2.cov[i][j];
        }
    }
    return op;
}


template<int N>
data::Pose2D_t<N> operator+(const data::Pose2D_t<N> &p1,const data::Pose2D_t<N> &p2)
{
data::Pose2D_t<N> op;
op.X = {0.0};
for(size_t i=0;i<p1.X.size();++i)
{
op.X[i] = p1.X[i]+p2.X[i]; 
}    
return op;
}

template<int N>
data::Covar2D_t<N> trans(const data::Covar2D_t<N> &mtin)
{
    data::Covar2D_t<N> op{{0}};
    for(size_t i=0;i<mtin.cov.size();++i)
    {
        for(size_t j=0;j<mtin.cov.at(0).size();++j)
        {
            op.cov[j][i] = mtin.cov[i][j];
        }
    }
    return op;
}

template<int N>
data::Covar2D_t<N> diagInverse(const data::Covar2D_t<N> &m1)
{
    data::Covar2D_t<N> op{};
    op.cov={{0.0f}};
    for(size_t i=0;i<m1.cov.size();++i)
    {
        op.cov[i][i] = 1/(m1.cov[i][i]);
    }
    return op;
}

template<int N>
void KalmanFilter<N>::initialize(const data::Pose2D_t<N> &X, const data::Covar2D_t<N> &cov,const uint32_t &tim)
{
  status_m = true;
  X_m = X;
  Cov_m = cov;
  t_m = tim;
}

template<int N>
data::Covar2D_t<N> KalmanFilter<N>::getCovariance() const
{
return Cov_m;
}

template<int N>
bool KalmanFilter<N>::getStatus() const
{
  return status_m;
}

template<int N>
void KalmanFilter<N>::predict(const float &vel, const float &omega,const uint32_t &t, const data::Covar2D_t<N> &sens_noise)
{
  for(size_t i=0;i<X_m.X.size()-1;++i)
  {
    X_m.X.at(i)+=vel*static_cast<float>(t-t_m);
  }
  X_m.X.at(X_m.X.size()-1)+=omega*static_cast<float>(t-t_m);
  
  data::Covar2D_t<N> proc_noise{};
  proc_noise.cov = {{0.0f}};
  for(size_t i=0;i<sens_noise.cov.size();i++)
  {
    proc_noise.cov[i][i] = (sens_noise.cov[i][i])*((t-t_m)*(t-t_m));
  }
  Cov_m = Cov_m + proc_noise;
  t_m = t;
}

template<int N>
void RobustKalman<N>::predict(const float &vel,const float &omega,const uint32_t &t,const data::Covar2D_t<N> &sens_noise)
{
if(vel >=4.0f||omega>=3.0f)
{
  throw std::invalid_argument("Invalid Velocity or Time");
}

else 
{
for(size_t i=0;i<N;++i)
  {
    this->X_m.X.at(i)+=vel*static_cast<float>(t-this->t_m);
  }
  this->X_m.X.at(N-1)+=0.4*vel*static_cast<float>(t-this->t_m);
  data::Covar2D_t<N> proc_noise{};
  proc_noise.cov = {{0.0f}};
  for(size_t i=0;i<sens_noise.cov.size();++i)
  {
    proc_noise.cov[i][i] = sens_noise.cov[i][i]*(t-this->t_m)*(t-this->t_m);
  }
  this->Cov_m = this->Cov_m + proc_noise;
  this->t_m = t;
}

}

template<int N> 
void KalmanFilter<N>::update(data::Pose2D_t<N> &z,const uint32_t &mtime,const data::Covar2D_t<N> &meas_noise)
{
    data::Covar2D_t<N> K = Cov_m*(diagInverse((Cov_m+meas_noise)));
    t_m = mtime;
    for(size_t i=0;i<z.X.size();++i)
    {
        z.X[i]-=X_m.X[i];
    }
    X_m = X_m + (K*(z));

    data::Covar2D_t<N> I{};
    I.cov = {{0.0}};
    for(size_t i=0;i<I.cov.size();i++)
    {
        I.cov[i][i]=1.0f;
    }

    Cov_m = (I-K)*Cov_m;// Normal Covariance update
}

template<int N> 
void RobustKalman<N>::update(data::Pose2D_t<N> &z,const uint32_t &mtime,const data::Covar2D_t<N> &meas_noise)
{
    data::Covar2D_t<N> K = Cov_m*(diagInverse((Cov_m+meas_noise)));
    t_m = mtime;
    for(size_t i=0;i<z.X.size();++i)
    {
        z.X[i]-=X_m.X[i];
    }
    X_m = X_m + (K*(z));

    data::Covar2D_t<N> I{};
    I.cov = {{0.0f}};

    for(size_t i=0;i<I.cov.size();i++)
    {
        I.cov[i][i]=1.0f;
    }

    Cov_m = ((I-K)*((Cov_m)*(trans((I-K))))+(K*meas_noise*trans(K)));
}

}