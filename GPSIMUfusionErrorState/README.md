# GPS IMU Fusion 
### [WIP - Only some more comments and tests are to be added]
- A Loosely Coupled GPS IMU fusion has been implemented
- Error State Kalman Filter has been used
- Eigen has been used as well
- PLEASE NOTE that gtests need to still be added and some more comments , checks are yet to be added [WIP]

  ## Note on States and Measurements
  - Error State has been used - delx = (misaligment angles (alpha, beta, gamma), velocity N, velocity E, velocity D, PosN, PosE, PosD, bf (3x1), bw (3x1))
  - delz = ZGPS-ZIMU
  - ZGPS = (velN, velE, posN, posE,posD)
  - ZIMU = (velN (INS), velE (INS), posN (INS), posE (INS), posD (INS))
    
