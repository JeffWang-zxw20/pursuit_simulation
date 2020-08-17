#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H

#include <eigen3/Eigen/Core>  //https://github.com/opencv/opencv/issues/14868
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "my_spline.h"

namespace MPCC {
//    class Map
    class Map {
    public:
        std::vector<double> xcenter;
        std::vector<double> ycenter;
        std::vector<double> xouter;
        std::vector<double> youter;
        std::vector<double> xinner;
        std::vector<double> yinner;
    public:
        struct Traj {
            double trackWidth = 0;
            std::vector<double> theta;
            tk::spline xcenter;   //Namespace of spline is tk, spline is a class in this namespace
            tk::spline ycenter;   //So here we create an obj of the class spline 
            tk::spline xouter;
            tk::spline youter;
            tk::spline xinner;
            tk::spline yinner;
        };
        Traj traj;   ///ASK: Traj is a struct inside the class and traj is an object of Traj??              
                    //so we can use it by  map.traj.xcenter where map is an object of class Map
                    //and traj is an object of a struct inside the class Map ?


        Map();

        void splineMap();

        double calculateTheta(double currentX, double currentY); //ppt, where is the implementation?

    };
}

#endif //PROJECT_MAP_H
