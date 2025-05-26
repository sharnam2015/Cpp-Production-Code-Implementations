#pragma once
#include<cstdint>
#include<array>

namespace data{

  template<int N>
  struct Pose2D_t
  {
    std::array<float,N> X{0.0f};
  };

  template<int N>
  struct Covar2D_t
  {
    std::array<std::array<float,N>,N> cov{{0.0f}};
  };

}

