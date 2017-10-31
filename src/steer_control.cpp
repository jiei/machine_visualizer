#include <ros/ros.h>
#include <math.h>
#include "4wheel.h"

#define L 0.8

//#define RELATIVE
#define ABSOLUTE

geometry_msgs::Vector3 velocity;
geometry_msgs::Pose2D pose;

void PoseCallBack(const geometry_msgs::Pose2D& msg){
  pose=msg;
  ROS_INFO_STREAM_ONCE("pose received !!");
}

void VelocityCallBack(const geometry_msgs::Vector3& msg){
  velocity=msg;
  ROS_INFO_STREAM_ONCE("velocity received !!");
}

int main(int argc,char **argv){
  ros::init(argc,argv,"steer_control");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");

  ros::Subscriber mimic_pose_sub=n.subscribe("mimic_pose",1,PoseCallBack);
  ros::Subscriber velocity_sub=n.subscribe("target_velocity",1,VelocityCallBack);
  ros::Rate loop_rate(30);

  Steering_4 S(L,L,pi,(-1)*pi);

  double yaw[4];
  double phi=0,tire_tmp=0;
  double v_tx=0,v_ty=0;

  while(ros::ok()){
    S.set_pose(pose);

    if((velocity.x==0) && (velocity.y==0)) phi=0;
    else phi=atan2(velocity.y,velocity.x)-pi/2;

#ifdef RELATIVE
    if(velocity.z==0.0){
      if(phi<(-pi)) yaw[0]=phi+2*pi;
      else yaw[0]=phi;

      yaw[1]=yaw[0];
      yaw[2]=yaw[0];
      yaw[3]=yaw[0];
    } else {
      yaw[0]=pi/4;
      yaw[1]=(-1)*pi/4;
      yaw[2]=pi/4;
      yaw[3]=(-1)*pi/4;
    }
#endif

#ifdef ABSOLUTE
    v_tx=velocity.x-L/sqrt(2)*velocity.z*sin(pose.theta+pi/4);
    v_ty=velocity.y+L/sqrt(2)*velocity.z*cos(pose.theta+pi/4);
    tire_tmp=atan2(v_ty,v_tx)-pose.theta-pi/2;
    if(tire_tmp>=pi) yaw[0]=tire_tmp-2*pi;
    else if(tire_tmp<=(-1)*pi) yaw[0]=tire_tmp+2*pi;
    else yaw[0]=tire_tmp;

    v_tx=velocity.x-L/sqrt(2)*velocity.z*sin(pose.theta+pi*3/4);
    v_ty=velocity.y+L/sqrt(2)*velocity.z*cos(pose.theta+pi*3/4);
    tire_tmp=atan2(v_ty,v_tx)-pose.theta-pi/2;
    if(tire_tmp>=pi) yaw[1]=tire_tmp-2*pi;
    else if(tire_tmp<=(-1)*pi) yaw[1]=tire_tmp+2*pi;
    else yaw[1]=tire_tmp;

    v_tx=velocity.x-L/sqrt(2)*velocity.z*sin(pose.theta-pi*3/4);
    v_ty=velocity.y+L/sqrt(2)*velocity.z*cos(pose.theta-pi*3/4);
    tire_tmp=atan2(v_ty,v_tx)-pose.theta-pi/2;
    if(tire_tmp>=pi) yaw[2]=tire_tmp-2*pi;
    else if(tire_tmp<=(-1)*pi) yaw[2]=tire_tmp+2*pi;
    else yaw[2]=tire_tmp;

    v_tx=velocity.x-L/sqrt(2)*velocity.z*sin(pose.theta-pi/4);
    v_ty=velocity.y+L/sqrt(2)*velocity.z*cos(pose.theta-pi/4);
    tire_tmp=atan2(v_ty,v_tx)-pose.theta-pi/2;
    if(tire_tmp>=pi) yaw[3]=tire_tmp-2*pi;
    else if(tire_tmp<=(-1)*pi) yaw[3]=tire_tmp+2*pi;
    else yaw[3]=tire_tmp;
#endif

    S.set_wheels(yaw[0],yaw[1],yaw[2],yaw[3]);

    S.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
