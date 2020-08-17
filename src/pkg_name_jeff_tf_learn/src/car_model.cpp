#include "car_model.h"
#include <ros/console.h>
//http://wiki.ros.org/roscpp/Overview/Logging
namespace Make_Move
{
    Car_model::Car_model ()
    {

    Cm1 = 0.287;
    Cm2 = 0.0545;
    Cr0  = 0.0518;
    Cr2 =  0.00035;

    Br = 3.3852;
    Cr = 1.2691;
    Dr = 0.1737;
    Bf = 2.579;
    Cf = 1.2;
    Df = 0.192;

    m = 0.041;
    Iz = 0.0000278;
    lf = 0.029;
    lr = 0.033;
    }

    void Car_model::update_model_state(double current_x, double current_y, double phi, 
                                  double vx, double vy, double omega)
    {
        // state << current_x, current_y, phi, vx, vy, omega, theta;
        // this->x = state[0];
        // this->y = state[1];
        // this->phi  = state[2];                 
        // this->vx = state[3];
        // this->vy = state[4];
        // this->omega = state[5];
        // this->theta = state[6];   //state matrix is useless for now

        this->x = current_x;
        this->y = current_y;
        this->phi = phi;
        this->vx = vx;
        this->vy = vy;
        this->omega = omega;
                                  //theta is useless for now 
    }

    void Car_model::update_model_input(double d, double delta)
    {   

        //input <<  d, delta, dif_theta;  //theta is uselss for now, so is dif_theta
        // this->d = input[0];
        // this->delta = input[1];
        // this->dif_theta = input[2];
        this->d = d;
        this->delta = delta;
    }

    void Car_model::cal_next_model_state()
    {
        /*
        double alpha_f = -atan2( (omega*lf + vy), vx ) + delta;
        double Ffy = Df*sin(Cf*atan2(Bf*alpha_f, 1));
        double alpha_r = atan2( (omega*lr - vy), vx );
        double Fry = Dr*sin(Cr*atan2(Br*alpha_r,1));
        double Frx = (Cm1 - Cm2*vx)*d - Cr0 - Cr2*vx*vx;
        if(Frx <= 0.0)
            Frx = 0;

        double dx = vx*cos(phi) - vy*sin(phi);
        double dy = vx*sin(phi) + vy*cos(phi);
        double dphi = omega;
        double dvx = 1/m*(Frx - Ffy*sin(delta) + m*vy*omega);
        double dvy = 1/m*(Fry + Ffy*cos(delta) -m*vx*omega );
        double domega = 1/Iz* (Ffy*lf*cos(delta) - Fry*lr);
        //ROS_INFO(dx,"dx");
        std::cout <<"updating state, dx is" <<dx << std::endl;
        std::cout <<"updating state, phi is" <<phi << std::endl;
        std::cout <<"updating state, vx,vy is" <<vx<<" "<<vy << std::endl;
        std::cout <<"updating state, Frear is" <<Frx<< std::endl;
        phi = phi + dphi*ts;
        x = x+ dx*ts;   //linear intepolation
        y = y+ dy*ts;
        // vx = cos(phi)*dx + sin(phi)*dy;
        // vy = -sin(phi)*dx + cos(phi) * dy; ////////////////check this latter
        vx = vx + dvx*ts;
        vy = vy + dvy*ts;
        omega = omega + domega*ts;
        */

        //jialin's code , for debugging
        double alpha_f = -atan2(lf * omega + vy,vx)+delta;  //paper Page5 (2a)
        double alpha_r =  atan2(lr*omega - vy,vx);

        double F_fy = Df*sin(Cf*atan(Bf*alpha_f));
        double F_ry = Dr*sin(Cr*atan(Br*alpha_r));
        
        double F_rx = (Cm1*d- Cm2*d*vx- Cr0- Cr2*vx*vx);
        if (F_rx < 0 && vx < 1e-5){         //to avoid infeasible negative rear wheel force
            F_rx = 0;
        }

        double dx   = vx*cos(phi) - vy*sin(phi);  //(1a) page4
        double dy   = vx*sin(phi) + vy*cos(phi);
        double dphi = omega;
        double dvx  = 1/m*(F_rx-F_fy*sin(delta)+m*vy*omega);
        double dvy  = 1/m*(F_ry+F_fy*cos(delta)-m*vx*omega);
        double domega = 1/Iz *(F_fy*lf*cos(delta)-F_ry*lr);


        x   = x+dx*ts;          //linear approach? Use RK4 or Verlet method may be better?
        y   = y+dy*ts;
        phi = phi+dphi*ts;
        if (phi > M_PI)
            phi = -2*M_PI + phi;
        else if(phi < -M_PI)
            phi = 2*M_PI + phi;
        vx  = vx+dvx*ts;
        vy  = vy+dvy*ts;
        omega = omega+domega*ts;
        //state << x, y, phi, vx, vy, omega;
    }
}
