#include<fusion.hpp>
#include<stdexcept>

namespace fusion{

Kalman::Kalman(Eigen::Vector<float,15> x_i,Eigen::Matrix<float,15,15> cov_i,float t_i)
{
    this->errx_m.fill(0.0f);
    this->errcov_m = cov_i;
    this->t_m = t_i;
    is_initialized_m=true;
    this->x_m=x_i;
    prev_omegas_m.fill(0.0f);
    this->R_stor_m = Eigen::Matrix3f::Identity();
}

void Kalman::initialize(Eigen::Vector<float,15> x_i,Eigen::Matrix<float,15,15> cov_i,float t_i)
{
    this->errx_m.fill(0.0f);
    this->errcov_m = cov_i;
    this->t_m = t_i;
    is_initialized_m=true;
    this->x_m=x_i;
    prev_omegas_m.fill(0.0f);
}

bool Kalman::isInitialized()
{
    return this->is_initialized_m;
}


void Kalman::predict(Eigen::Vector3f acc,Eigen::Vector3f omegas, float t_p)
{
    Eigen::MatrixXf A(15,15);
    A.setZero();
    Eigen::Matrix3f R_curr;
    Eigen::Matrix3f Y;
    float delt = t_p-t_m;
    Y(0,1) = -1*omegas(2)*delt;
    Y(0,2) = omegas(1)*delt;
    Y(1,0) = omegas(2)*delt;
    Y(1,2) = -omegas(0)*delt;
    Y(2,0) = -1*Y(0,2);
    Y(2,1) = -1*Y(1,2);

    
    R_curr = (Eigen::Matrix3f::Identity()-Y)*R_stor_m;
    Eigen::Vector3f acc_ned = R_curr*acc;
    Eigen::Matrix3f acc_ned_skew;
    acc_ned_skew << 0.0f,-acc_ned[2], acc_ned[1],
                                    acc_ned[2],0.0f,-acc_ned[0],
                                    -acc_ned[1],acc_ned[0],0.0f;


    A.block<3,3>(0,9) =-R_curr;
    A.block<3,3>(3,12) = R_curr;
    A.block<3,3>(6,3) = Eigen::Matrix3f::Identity();
    A.block<3,3>(3,0) = acc_ned_skew;

    A = Eigen::Matrix<float,15,15>::Identity()+(A*delt);

    Eigen::MatrixXf G(15,12);
    G = Eigen::Matrix<float,15,12>::Zero();
    G.block<3,3>(0,0) = -R_curr;
    G.block<3,3>(3,3) = R_curr;
    G.block<3,3>(9,6) = Eigen::Matrix3f::Identity();
    G.block<3,3>(12,9) = Eigen::Matrix3f::Identity();

   Eigen::Vector<float,12> v{0.0014f,0.0014f,0.0014f,0.004f,0.004f,0.004f,0.0005f,0.0005f,0.0005f,0.02f,0.02f,0.02f};
    Eigen::Matrix<float,12,12> Q;
    Q = v.asDiagonal();
    Eigen::Matrix<float,15,15> Q_proc;
    Q_proc =  (1/delt)*(G*Q*(G.transpose()))*delt*delt;

    errcov_m = A*errcov_m*(A.transpose())+ Q_proc;
    errx_m = A*errx_m;
    prev_omegas_m = omegas;
    t_m = t_p;
    R_stor_m = R_curr;
}

void Kalman::propogate(float t_curr)
{
    Eigen::Vector3f aprop{0.0f,0.0f,0.0f};
    predict(aprop,prev_omegas_m,t_curr);
}

std::tuple<Eigen::Vector<float,15>,Eigen::Matrix<float,15,15>,float> Kalman::getErrorStateAndCovar()
{
return{errx_m,errcov_m,t_m};
}

std::tuple<Eigen::Vector<float,15>,Eigen::Matrix<float,15,15>,float> Kalman::getStateAndCovar()
{
return std::make_tuple(x_m,errcov_m,t_m);
}

void Kalman::update(Eigen::Vector<float,5> z_meas,Eigen::Matrix<float,5,5> R_meas,float t_meas)
{
propogate(t_meas);
auto tup = getStateAndCovar();
auto x_tot = std::get<0>(tup);
//To update the INS state properly with the latest propogated error addition
auto err_ins = std::get<0>(getErrorStateAndCovar());

Eigen::Vector<float,5>x_pred = {x_tot[3],x_tot[4],x_tot[6],x_tot[7],x_tot[8]};
Eigen::Vector<float,5> err_x_pred = {err_ins[3],err_ins[4],err_ins[6],err_ins[7],err_ins[8]};
Eigen::Vector<float,5> z_pred = x_pred+err_x_pred;
Eigen::Vector<float,5> z = z_meas - z_pred;

Eigen::Matrix<float,5,15> H;
H = Eigen::Matrix<float,5,15>::Zero();
H.block<2,2>(0,3) = -1*Eigen::Matrix2f::Identity();
H.block<3,3>(2,6) = -1* Eigen::Matrix3f::Identity();

Eigen::MatrixXf Pp = std::get<1>(tup);

Eigen::Matrix<float,15,5> K;
K = Pp*(H.transpose())*(((H*Pp*(H.transpose()))+(R_meas)).inverse());

errx_m = errx_m + K*(z-(H*errx_m));
errcov_m = (Eigen::Matrix<float,15,15>::Identity()-(K*H))*Pp;

//correction of total state
x_m = x_m + errx_m;
errx_m = Eigen::Vector<float,15>::Zero();
}

}
