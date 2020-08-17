#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // tf is package
#include <turtlesim/Pose.h>

std::string turtle_name;



void poseCallback(const turtlesim::PoseConstPtr& msg){  //msg is a const pointer to an obj 
  static tf::TransformBroadcaster br;   //creat a tf broadcaster obj
  tf::Transform transform;           //create a tf obj
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );  //here, 0.0 is relative to parent frame: the world frame in this case
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);  //Set the quaternion using fixed axis RPY.  RPY := raw,pitch,yaw.    elative to parent frame: world frame
                                //	setRPY (const tf2Scalar &roll, const tf2Scalar &pitch, const tf2Scalar &yaw)
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
                                      //the transform itself
                                                // give the transform being pub a timstamp
                                                                    //Name of parent frame
                                                                            //name of the child frame

}

int main(int argc, char** argv){
  ros::init(argc, argv, "node_name_my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};