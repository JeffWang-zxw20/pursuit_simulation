#include "map.h"

namespace MPCC{
    Map::Map(){   //Map is a class  src/pkg_name_jeff_tf_learn/config/Track.yaml
                  //Inside the class, a func ppt is declared, the ppt is called Map()
        YAML::Node node = YAML::LoadFile("/home/jeff/ws/simulation_pid_ws/src/pkg_name_jeff_tf_learn/config/Track.yaml");
        xcenter = node["xcenter"].as < std::vector < double > > ();
        ycenter = node["ycenter"].as < std::vector < double > > ();
        xouter = node["xouter"].as < std::vector < double > > ();
        youter = node["youter"].as < std::vector < double > > ();
        xinner = node["xinner"].as < std::vector < double > > ();
        yinner = node["yinner"].as < std::vector < double > > ();

        xcenter.push_back(xcenter[0]);
        ycenter.push_back(ycenter[0]);
        xouter.push_back(xouter[0]);
        youter.push_back(youter[0]);
        xinner.push_back(xinner[0]);
        yinner.push_back(yinner[0]);

        splineMap();
    }

    void Map::splineMap() {
        std::vector<double> t;
        for (unsigned int i = 0; i < xcenter.size(); ++i) {
            t.push_back(i);
        }
        // normalization
        traj.xcenter.set_points(t, xcenter);
        traj.ycenter.set_points(t, ycenter);
        for (unsigned int j = 0; j < xcenter.size(); ++j) {
            if (j == 0) traj.theta.push_back(0);
            else {
                double rightdiff = sqrt(pow(traj.xcenter(j, 1), 2) + pow(traj.ycenter(j, 1), 2));
                double leftdiff = sqrt(pow(traj.xcenter(j - 1, 1), 2) + pow(traj.ycenter(j - 1, 1), 2));
                traj.theta.push_back(traj.theta[j - 1] + (leftdiff + rightdiff) / 2);
            }
        }
        // spline with theta(length of track centerline)
        traj.xcenter.set_points(traj.theta, xcenter);
        traj.ycenter.set_points(traj.theta, ycenter);
        traj.xouter.set_points(traj.theta, xouter);
        traj.youter.set_points(traj.theta, youter);
        traj.xinner.set_points(traj.theta, xinner);
        traj.yinner.set_points(traj.theta, yinner);
    }

    double Map::calculateTheta(double currentX, double currentY) {
        // just tranverse each item in thetalist to find the nearest one
        double distanceX2, distanceY2, distance, nearestDistance;
        unsigned int nearestId = 0, realPreviousId = 0;
        unsigned int trajSize = traj.theta.size();

        for (unsigned int i = 0; i < trajSize; ++i) {
            distanceX2 = pow(traj.xcenter(traj.theta[i]) - currentX, 2);
            distanceY2 = pow(traj.ycenter(traj.theta[i]) - currentY, 2);
            distance = sqrt(distanceX2 + distanceY2);
            if (i == 0) {
                nearestDistance = distance;
            } else if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestId = i;
            }
        }
        /**
         * vector1: nearest ==> current
         * vector2: nearest ==> previous
         * if vector1 * vector2 > 0:
         *    real previous <- nearest
         * else:
         *    real previous <- previous
         * **/
        unsigned int previousId = (nearestId + trajSize - 1) % trajSize;
        double deltaTheta = 0;
        double nearestX = traj.xcenter(traj.theta[nearestId]);
        double nearestY = traj.ycenter(traj.theta[nearestId]);

        double previousX = traj.xcenter(traj.theta[previousId]);
        double previousY = traj.ycenter(traj.theta[previousId]);
        double this_or_previous = (currentX - nearestX) * (previousX - nearestX) +
                                    (currentY - nearestY) * (previousY - nearestY);
        realPreviousId = this_or_previous > 0 ? previousId : nearestId;
        //todo: I cannot understand it... perhaps this is what we call magic?
        //I can't understand this either....I will skip it since 
        return deltaTheta + traj.theta[realPreviousId];
    }
} //namespace MPCC

