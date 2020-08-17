#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_new_frame_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    //transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );  //create a new transform, setting the orgin of the new frame 
    transform.setOrigin(tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0));                                                  
    //to be 2 meters away of the other frame (turtle1 here)
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );  //zero rotation
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "Jeff3"));
                                        //transform is the new frame we just created by setting origin and relative rotation
                                                    //Time allows us to get the latest frame
                                                                    //the parent frame or relative frame when we are setting up the transformation
                                                                                    //name of the child frame
    rate.sleep();
  }
  return 0;
};