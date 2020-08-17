#ifndef PROJECT_CAR_MODEL_H
#define PROJECT_CAR_MODEL_H


//#include <Eigen/Core> //A matrix lib used to store states
//https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html  read for eigen use
//#include <unsupported/Eigen/MatrixFunctions> //A wired one
#include<yaml-cpp/yaml.h>          //Used to load yaml para files


namespace Make_Move
{


    //static constexpr unsigned long freq = 64;  //ns wide attribute??  64 since is 2^n, easier for pc to cal
    //static constexpr double ts = 1.0/freq;  //timestep for each simulation/cal


    //class car_model
    class Car_model
    {
        public:
        //static constexpr int num_state = 10;  //How many states
        //static constexpr int num_input = 3;   //size of u vector, input


        // state = [x,y,phi, vx,vy,omega, theta]T
        //Eigen::VectorXd state = Eigen::MatrixXd::Zero(num_state-num_input,1);  //init the state matrix
        // input = [D,delta,vtheta]T
        //Eigen::VectorXd input = Eigen::MatrixXd::Zero(num_input,1);

        


        //YAML::Node node = YAML::LoadFile("/home/jeff_2/ws/visu2/src/pkg_name_jeff_tf_learn/config/PID_para.yaml");
        //double kp =node["P"].as<double> ();
        //model para
        YAML::Node node = YAML::LoadFile("/home/jeff/ws/simulation_pid_ws/src/pkg_name_jeff_tf_learn/config/car_model_para.yaml");
        double Cm1, Cm2, Cr0, Cr2, Br, Cr, Dr, Bf, Cf, Df, m, Iz, lf, lr;

        double x;
        double y;
        double phi;
        double vx;
        double vy;
        double omega;
        double theta = 0;  //get from map function ,useless for now
        //double dif_theta=0;  //useless for now
        double d = node["throll"].as<double>();
        double delta;
        double freq = 64;  //64
        double ts = 1.0/freq;

        //Constructor 
        Car_model();  //ppt

        //Class methods ppt
        void update_model_state(double current_x, double current_y, double phi, 
                                  double vx, double vy, double omega);
        void update_model_input(double d, double delta);
        void cal_next_model_state();
        



    };



}

#endif  //PROJECT_MAKE_IT_MOVE_H