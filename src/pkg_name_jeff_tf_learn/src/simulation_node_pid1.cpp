#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include<geometry_msgs/Pose2D.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "make_it_move.cpp"
#include "make_it_move.h"
#include "map.h" //so that we can use mpcc namespace and map lib
#include <nav_msgs/Path.h>  //used for showing map 

int main( int argc, char** argv )
{
  ros::init(argc, argv, "node_name_arrow_car");
  ros::NodeHandle n;
  ros::Rate loop_rate(64);  //64



  //*****************Make a car in rviz*************************************

  //ros::Publisher rviz_car_model_pub = n.advertise<visualization_msgs::Marker>("rviz_car_model_doubt", 1000);
  //Publish the groundpose?????? what's the differentce between this and rviz_pose
  ros::Publisher rviz_car_pose_doubt_pub = n.advertise<visualization_msgs::Marker>("rviz_car_pose_doubt", 1000);
  //Publish the track to rviz
  //ros::Publisher track_center_pub = n.advertise<nav_msgs::Path>("traj",1000);

  // Set our initial shape type to be an arrow
  //uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker arrow_car;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  arrow_car.header.frame_id = "/global_map";
  arrow_car.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  arrow_car.ns = "arrow_car";
  arrow_car.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  arrow_car.type = arrow_car.ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  arrow_car.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  arrow_car.scale.x = 0.05;
  arrow_car.scale.y = 0.01;
  arrow_car.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  arrow_car.color.r = 0.0f;
  arrow_car.color.g = 1.0f;
  arrow_car.color.b = 0.0f;
  arrow_car.color.a = 1.0;


//***************************************************************************************





//************************make a reference point marker*********************************


ros::Publisher rviz_ref_pose_pub = n.advertise<visualization_msgs::Marker>("rviz_ref_pose", 1000);
  //Publish the track to rviz
  //ros::Publisher track_center_pub = n.advertise<nav_msgs::Path>("traj",1000);

  // Set our initial shape type to be an arrow
  //uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker refe;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  refe.header.frame_id = "/global_map";
  refe.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  refe.ns = "refe";
  refe.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  refe.type = refe.ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  refe.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  refe.scale.x = 0.05;
  refe.scale.y = 0.01;
  refe.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  refe.color.r = 0.0f;
  refe.color.g = 1.0f;
  refe.color.b = 0.0f;
  refe.color.a = 1.0;

//****************************************************************************************************************




//**************************import map, use jalin's code ****************************************
  ros::Publisher track_pub = n.advertise<nav_msgs::Path>("track",1000);
  nav_msgs::Path track;
  track.header.frame_id = "/global_map";   //this needs to push in the ros ok loop
  


  geometry_msgs::PoseStamped temp_pose;
  temp_pose.pose.position.z = 0;       //fix the car to ground
  temp_pose.header.frame_id = "/global_map";
  //geometry_msgs::Quaternion temp_quat;
  //temp_quat.x = temp_quat.y = temp_quat.z = 0;   //set orientation to zero for now
  //roll = atan2(f(x,y,z,w))  => roll = 0
  //pitch = 0
  //yaw = 0;  see picture sent to yinjian.

  MPCC::Map track_obj; //create a map obj  //need to include map.h in order to use this 
  for (size_t i{0}; i<track_obj.xcenter.size(); ++i)
  {
    temp_pose.pose.position.x = track_obj.traj.xcenter(track_obj.traj.theta[i]);
    temp_pose.pose.position.y = track_obj.traj.ycenter(track_obj.traj.theta[i]);
    track.poses.push_back(temp_pose);
  }

//*************************************************************************************************************
  
  
  
  YAML::Node node = YAML::LoadFile("/home/jeff/ws/simulation_pid_ws/src/pkg_name_jeff_tf_learn/config/Track.yaml");  //"../config/Track.yaml"
  std::vector<double> X_ref = node["xcenter"].as < std::vector <double> >();
  std::vector<double> Y_ref = node["ycenter"].as < std::vector <double> >();
  int num_p = X_ref.size();
  //std::cout <<"number of points" <<num_p;
  //std::vector<int> vi = node["numbers"].as<std::vector<int>>();


  int step = 1;
  int point_ind = 1;
  double x_ei = 1.01;
  double y_ei = 0.74;
  //double phi_ei = atan2(track_obj.traj.ycenter(0,1), track_obj.traj.xcenter(0,1));   //jilin's method, confused for now     aim is to align the car to the track at the starting point
  double phi_ei = 0;
  using namespace Make_Move;
  Move_solver please_move;     //create an obj
  please_move.car_model.update_model_state(x_ei,y_ei,phi_ei,0,0,0);
  please_move.car_model.update_model_input(0, 0);
  please_move.x_ref= X_ref.at(point_ind);
  please_move.y_ref= Y_ref.at(point_ind);
  geometry_msgs::Point point;
  geometry_msgs::Point point_refe;
  geometry_msgs::Quaternion quat;

  

  while (ros::ok())
  {

    // Publish the marker
    while (rviz_car_pose_doubt_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    //for testing rviz connection
    // arrow_car.pose.position.x = please_move.x;
    // arrow_car.pose.position.y = please_move.y;

    // please_move.just_move();

    // arrow_car.pose.position.x = please_move.x;
    // arrow_car.pose.position.y = please_move.y;
    
    //debugging part
    std::cout << "i am in ros::ok" << std::endl;
      
    if (point_ind == num_p -1 || point_ind == num_p -2 || point_ind == num_p)
    { 
      std::cout <<"is index reset to zero here?" <<std::endl;
      point_ind = 0;
    }
      
    please_move.PID_solver();   
    //please_move.car_model.delta = 0;   //negative delta turn right
    //please_move.car_model.d = 0.4;                  
                                            //PID changes the d and delta
                                            //then we are ready to find the next state.
    //please_move.car_model.phi = 3*M_PI;
    please_move.car_model.cal_next_model_state(); //This will cal and update the state
    point.x = please_move.car_model.x;
    point.y = please_move.car_model.y;
    point.z = 0;
    point_refe.x = please_move.x_ref;
    point_refe.y = please_move.y_ref;
    point_refe.z = 0;
    //
    //http://wiki.ros.org/tf2/Tutorials/Quaternions
    //https://blog.csdn.net/qit1314/article/details/83280992

    //euler angle in ROS is: Euler angles specified here are 
    //"""intrinsic rotations""" around rotating axis,
    //see http://wiki.ros.org/geometry2/RotationMethods

    //intrinsic euler angle defi:
    //rotations about the axes of the rotating coordinate system XYZ, solidary with 
    //the moving body, which changes its orientation after each elemental rotation

    //phi should be  a valid range could be [−π, π].??? the alfa angle on the wiki
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,please_move.car_model.phi);
    quat = tf2::toMsg(myQuat);  //Convert a tf2 Quaternion type to its equivalent geometry_msgs representation

    arrow_car.pose.position = point;
    arrow_car.pose.orientation = quat;
    refe.pose.position = point_refe;
    rviz_car_pose_doubt_pub.publish(arrow_car);
    rviz_ref_pose_pub.publish(refe);
    track_pub.publish(track);
    double L_dis;
    L_dis = pow((please_move.x_ref - please_move.car_model.x),2) + pow((please_move.y_ref - please_move.car_model.y),2);
    if (L_dis < 0.01)
    {
      
      if(L_dis<0.005)
      {
        please_move.car_model.d = 0.1;
      }
      
      point_ind += step;
    }
    //point_ind += step;
    std::cout <<"i am in ros ok loop, x_ref x is :" <<please_move.x_ref << " "<< please_move.car_model.x<<std::endl;
    std::cout <<"i am in ros ok loop, y_ref y is :" <<please_move.y_ref <<" " <<please_move.car_model.y<<std::endl;
    std::cout <<"i am in ros ok loop, point index is:" <<point_ind <<std::endl;
    
    please_move.x_ref= X_ref.at(point_ind);
    please_move.y_ref= Y_ref.at(point_ind);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}