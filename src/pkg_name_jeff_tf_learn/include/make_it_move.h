#ifndef PROJECT_MAKE_IT_MOVE_H
#define PROJECT_MAKE_IT_MOVE_H
#include <ros/ros.h>
#include "car_model.h"
//ppt stands for function prototype

namespace Make_Move
{



    class Move_solver
    {

    public:
        //PID part
        YAML::Node node = YAML::LoadFile("/home/jeff/ws/simulation_pid_ws/src/pkg_name_jeff_tf_learn/config/PID_para.yaml");
        double kp =node["P"].as<double> ();
        double ki =node["I"].as<double> ();
        double kd =node["D"].as<double> ();
        double phi_e_prev=0;
        double int_e=0;
        double dif_e=0;
        double x_ref;
        double y_ref;

        //jialin pid debugging part
        //qPose : 0.3
        //qPred : 0.15
        //qThroll : 6.5
        // double qPred = 0.15;
        // double qPose = 0.3;
        // double qThroll = 6.5;
        // double phi,phiPose;
        // double error, error_last, i_error;
        // double x_;
        // double y_;
        // double phi_;
// double theta,x_,y_,phi_,phiPose; //???phipose???

// double qPose, qPred, qThroll;
// double P, I, D, throll;

// double error, error_last;
// double i_error;

        Make_Move::Car_model car_model;  //So this class can use car's attributes??
        

    //constructor
    //Move_solver();
    
    //void just_move();   //member func's ppt   --- test to see whether the car can move in rviz
    inline void PID_solver();   //pid controller  ppt
    //void the_great_verlet_method();  //numerical simulation method to simu the next state  ppt


    };







}
  
#endif   //PROJECT_MAKE_IT_MOVE_H