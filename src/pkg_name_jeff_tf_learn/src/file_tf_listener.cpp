#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "cpp_file_node_name_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/turtle2", "/turtle1",     //transform from frame turtle1 to 2
                               ros::Time(0), transform);       //Time(0) get us the latest available tf
                                                            //transform is the object we want to store the tf

    //    listener.lookupTransform("/turtle2", "/Jeff3",     //transform from frame turtle1 to 2
    //                            ros::Time(0), transform);
        // listener.waitForTransform("/turtle2", "/turtle1",     //transform from frame turtle1 to 2
        //                         now, ros::Duration(3.0));  //block until the tf between two turtles becomes visible
        // listener.lookupTransform("/turtle2", "/turtle1",     //transform from frame turtle1 to 2
        //                         now, transform); 
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);                                             ///compute the difference between this frame turtle2 and turtle1 
                                                                            //and send cmd to turtle2

    rate.sleep();
  }
  return 0;
};
