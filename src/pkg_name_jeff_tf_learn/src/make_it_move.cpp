//#include "pkg_name_jeff_tf_learn/make_it_move.h"
#include "make_it_move.h"
//#include ""
#include "map.h" //so that we can use mpcc namespace and map lib
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

#include <fstream>
#include <vector>
namespace Make_Move
{

    // Move_solver::Move_solver()
    // {
    //     x=0;
    //     y=0;
    //     vx=0;
    //     vy=0;
    // }



    // void Move_solver::just_move()
    // {
    //     x +=0.0001;
    //     y +=0.0005;
    // }



    void Move_solver::PID_solver()
    {   

//**************************************yj's algo**********************************************************
/*  
        double l_phi;
        double x_diff = x_ref - car_model.x;
        double y_diff = y_ref - car_model.y;
        double arctan_temp = atan2(y_diff,x_diff);

        l_phi = atan2(y_diff,x_diff) - car_model.phi;
        std::cout <<"I am in PID" << std::endl;
        std::cout <<"l_phi " << l_phi << std::endl;
        std::cout <<"phi " << car_model.phi << std::endl;
        std::cout <<"arctan" << atan2(y_diff,x_diff) << std::endl;
        std::cout <<"x_ref and x " << x_ref <<" "<<car_model.x << std::endl;
        std::cout <<"y_ref and y " << y_ref <<" "<<car_model.y << std::endl;
        if (l_phi > 2*M_PI)
            l_phi = l_phi - 2*M_PI;
        else
            l_phi = l_phi;
        if (l_phi < -2*M_PI)
            l_phi = l_phi +2*M_PI;
        else 
            l_phi = l_phi;

        double L_car  =0.15;
        double L_distance =pow((x_ref - car_model.x),2) + pow((y_ref - car_model.y),2);
        car_model.delta = 0.9* atan2( 2.0*L_car*sin(l_phi) / (L_distance + 0.15), 1.0 );
        car_model.delta = car_model.delta > 2*M_PI ? car_model.delta - 2*M_PI : car_model.delta;
        car_model.delta = car_model.delta < -2*M_PI ? car_model.delta - 2*M_PI : car_model.delta;
        car_model.delta = car_model.delta / (2*M_PI/9) * 0.8;
        car_model.delta = car_model.delta > 0.3 ? 0.3 : car_model.delta;
        car_model.delta = car_model.delta < -0.3 ? -0.3 : car_model.delta;
        
        std::cout <<"delta" << car_model.delta << std::endl;
        car_model.d = 0.2;
*/
//************************************yj's algo, worked*********************************************************






 /*

        //FInd out phi, change its reference line to x line
        double l_phi;
        double x_diff = x_ref - car_model.x;
        double y_diff = y_ref - car_model.y;
        double arctan_temp = atan2(y_diff,x_diff);

        l_phi = atan2(y_diff,x_diff) - car_model.phi;
        std::cout <<"I am in PID" << std::endl;
        std::cout <<"l_phi " << l_phi << std::endl;
        std::cout <<"phi " << car_model.phi << std::endl;
        std::cout <<"arctan" << atan2(y_diff,x_diff) << std::endl;
        std::cout <<"x_ref and x " << x_ref <<" "<<car_model.x << std::endl;
        std::cout <<"y_ref and y " << y_ref <<" "<<car_model.y << std::endl;


        // if (l_phi > M_PI)
        //     l_phi = -2*M_PI + l_phi;
        // else if(l_phi < -M_PI)
        //     l_phi = 2*M_PI + l_phi;

        // if (l_phi > 2*M_PI)
        //     l_phi = l_phi - 2*M_PI;
        // else
        //     l_phi = l_phi;
        // if (l_phi < -2*M_PI)
        //     l_phi = l_phi +2*M_PI;
        // else 
        //     l_phi = l_phi;

        if (x_diff>=0 && y_diff>=0)
            l_phi = arctan_temp;
        else if (x_diff>0 && y_diff<0)
            //l_phi = 2* M_PI +  arctan_temp;
            l_phi = arctan_temp;
        else if (x_diff<=0 && y_diff<0)
            //l_phi = 2* M_PI + arctan_temp;
            l_phi = -M_PI - arctan_temp;
        else if (x_diff<0 && y_diff>=0)
            //l_phi = M_PI - arctan_temp;
            l_phi = M_PI + arctan_temp;


        double phi_e = -(l_phi - car_model.phi);  

        int_e  += phi_e;
        dif_e  = (phi_e - phi_e_prev) / car_model.ts; 
        phi_e_prev = phi_e;
        //std::cout <<"i am in PID controller, the diff distance in x is " << x_diff <<std::endl; 
        //std::cout <<"i am in PID controller, the diff distance in y is " << y_diff <<std::endl;
        //std::cout <<"i am in PID controller, x y reference is: " << x_ref << " " << y_ref <<std::endl;
        std::cout <<"i am in PID controller, current l_phi and phi  are" << l_phi << " " << car_model.phi <<std::endl;   
        //car_model.delta = -kp*phi_e + ki*int_e + kd*dif_e;  //delta is negative for wheels pointing left
        car_model.delta = -kp*phi_e; 
        //confine the max turning angle of the wheel
        if (car_model.delta > 0.4)
            car_model.delta = 0.4;
        else if (car_model.delta <-0.4)
            car_model.delta = -0.4;
        else 
            car_model.delta = car_model.delta;


        //in the future, devise an algo of controlling d
        car_model.d = 0.2;
*/


/////////transform the map point, target point to car's frame of reference 
        // double yaw = car_model.phi;
        // Eigen::Vector3d ea0(yaw,0,0);

        // Eigen::Matrix3d R;
        // R = Eigen::AngleAxisd (ea0[0], Eigen::Vector3d::UnitZ())
        //   * Eigen::AngleAxisd (ea0[1], Eigen::Vector3d::UnitY())
        //   * Eigen::AngleAxisd (ea0[2], Eigen::Vector3d::UnitZ());
        
        // Eigen::Vector3d Pointi (x_ref,y_ref,0);
        // Eigen::Vector3d t (car_model.x,car_model.y,0);
        // Eigen::Vector3d Pointj;
        // Pointj = R*Pointi - t;

        // double proj_y = Pointj[1];
        // double leng_2 = pow(Pointj[0],2) + pow(Pointj[1],2);
        // double curv = 2* proj_y / leng_2;
        // std::cout <<"I am in PID" << std::endl;
        // std::cout <<"converted x: " << Pointj[0]<< std::endl;
        // std::cout <<"converted y: " << Pointj[1]<< std::endl;
        // std::cout <<"converted z: " << Pointj[2]<< std::endl;
        // //negative delta turn right
        // car_model.delta = kp * (-curv);   //if y is positive, delta should be negative
        // std::cout <<"curv" << curv << std::endl;
        // car_model.delta = car_model.delta > 0.4 ? 0.4 : car_model.delta;
        // car_model.delta = car_model.delta < -0.4 ? -0.4 : car_model.delta;
        // std::cout <<"delta" << car_model.delta << std::endl;
        // car_model.d = 0.2;


        Eigen::Matrix2d R;
        Eigen::Vector2d V((x_ref - car_model.x),(y_ref - car_model.y));   //[x3-x1; y3-y1]
        Eigen::Vector2d V_goal;
        R<< cos(car_model.phi), -sin(car_model.phi),
            sin(car_model.phi),  cos(car_model.phi);
        V_goal = R.inverse() * V;

        //double x_ref_c = x_ref * cos(car_model.phi) - y_ref * sin(car_model.phi) - car_model.x;
        //double y_ref_c = x_ref * sin(car_model.phi) + y_ref * cos(car_model.phi) - car_model.y; 
        double x_ref_c = V_goal(0);
        double y_ref_c = V_goal(1);
        double proj_y = y_ref_c;
        double leng_2 = pow(y_ref_c,2) + pow(x_ref_c,2);
        double curv = 2* proj_y / leng_2;

        std::cout << " " << std::endl;
        std::cout <<"I am in PID" << std::endl;
        std::cout <<"converted x: " << x_ref_c << std::endl;
        std::cout <<"converted y: " << y_ref_c<< std::endl;
        std::cout <<"i am in ros ok loop, x_ref x is :" <<x_ref << " "<< car_model.x<<std::endl;
        std::cout <<"i am in ros ok loop, y_ref y is :" <<y_ref <<" " <<car_model.y<<std::endl;
        //std::cout <<"converted z: " << Pointj[2]<< std::endl;
        //negative delta turn right
        car_model.delta = kp * (curv);   //if y is positive, delta should be negative
        //car_model.delta = (curv); 
        std::cout <<"curv" << curv << std::endl;
        car_model.delta = car_model.delta > 0.4 ? 0.4 : car_model.delta;
        car_model.delta = car_model.delta < -0.4 ? -0.4 : car_model.delta;
        std::cout <<"delta" << car_model.delta << std::endl;
        car_model.d = 0.2;

    }
















        //Jialin's pid algo, borrow here for debugging


        /*

        MPCC::Map map;s
        double x = car_model.x;
        double y = car_model.y;
        double theta = car_model.theta;
        
        car_model.theta = map.calculateTheta(x,y); //return deltaTheta + traj.theta[realPreviousId];
        x_= map.traj.xcenter(theta); //TRIPLE OBEJCTS!!! NB
        y_= map.traj.ycenter(theta);  //This return an interpolation of cubic spline line
        double distance = sqrt(pow(x-x_,2)+pow(y-y_,2));

        theta += qPred;
        theta = fmod(theta, map.traj.theta.back());
        x_ = map.traj.xcenter(theta);
        y_ = map.traj.ycenter(theta);

        double dx = map.traj.xcenter(theta,1);
        double dy = map.traj.ycenter(theta,1);

        double d2x = map.traj.xcenter(theta,2);
        double d2y = map.traj.ycenter(theta,2);

        double dphi__dtheta = (dx*d2y - dy*d2x)/(pow(dx,2)+pow(dy,2));
        
        phi_ = atan2(dy,dx);
        phiPose = atan2(y_-y,x_-x);
        
        double error_pose = phiPose - phi;
        double error_phi  = phi_ - phi;
        double c_dphi = 1;
        if (abs(dphi__dtheta) > 2)
            c_dphi = 2;

        if (error_pose > M_PI)
        {
            error_pose -= 2*M_PI;
        }
        if (error_pose < -M_PI){
            error_pose += 2*M_PI;
        }
        if (error_phi > M_PI){
            error_phi -= 2*M_PI;
        }
        if (error_phi < -M_PI){
            error_phi += 2*M_PI;
        }

        error_phi = error_phi / M_PI;
        error_pose = error_pose / M_PI;
        
        error_pose = error_pose/5 * (pow(2,10*distance)-1);
        // error_pose = error_pose/0.25 * distance;
        
        error_last = error;
        error = qPose * error_pose + (1-qPose) * error_phi;
        double d_error = (error-error_last)*64;
        i_error = i_error + error/64;


        double c_throll = pow(2,-qThroll*fabs(error));
        car_model.delta = kp * error + ki * i_error + kd * d_error;

        car_model.d = 2.5 * c_throll;
    if (car_model.delta > 0.35)
        car_model.delta = 0.35;
    if (car_model.delta < -0.35)
        car_model.delta = -0.35;
        


*/




}