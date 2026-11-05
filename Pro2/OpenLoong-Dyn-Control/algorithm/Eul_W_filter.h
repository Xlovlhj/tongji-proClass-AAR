/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#ifndef BIPED_STATEEST_EUL_W_FILTER_H
#define BIPED_STATEEST_EUL_W_FILTER_H

#include <Eigen/Dense>

class Eul_W_filter {
private:
    double dt;

public:
    bool isIni=false;

    Eigen::Matrix<double,6,1> kal_X, kal_Y, kal_Z; // eul, wL
    Eigen::Matrix<double,6,6> F; // state transition matrix
    Eigen::Matrix<double,6,6> H; // output matrix
    Eigen::Matrix<double,6,6> P; // covirance matrix
    Eigen::Matrix<double,6,6> Q; // process noise matrix
    Eigen::Matrix<double,6,6> R; // output noise matrix

    double Eul_filtered[3], wL_filtered[3];

    Eul_W_filter(double dtIn);
    void run(double *eulIn, double *wLIn);
    void getData(double *eulOut,double *wLOut);
};


#endif //BIPED_STATEEST_EUL_W_FILTER_H
