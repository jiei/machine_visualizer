#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

#ifndef pi
#define pi 3.14159265
#endif

/////////////////////////////////////////////////////////////class SteeringTire

class SteeringTire{
	public:
		double length;		//unit [m]
		double width;		//unit [m]
		double max_rotation_range,min_rotation_range;
		double now_yaw;
		double omega;

		visualization_msgs::Marker marker;
		SteeringTire();
		SteeringTire(std::string tire_id,int num_id,double length,double width,double max,double min);
		void renew(std::string tire_id,int num_id,double length,double width,double max,double min);
		void setYaw(double yaw);	//min_rotation_range~max_rotation_range
		tf::TransformBroadcaster br;
  		tf::Transform tf;
		tf::Quaternion q;
};

SteeringTire::SteeringTire(){

}

SteeringTire::SteeringTire(std::string tire_id,int num_id,double length,double width,double max,double min){
	marker.header.frame_id=tire_id;
	marker.ns=tire_id;
	marker.id=num_id;
	marker.type=visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = length;
	marker.scale.y = length;
	marker.scale.z = width;

	marker.color.r = 0.0f;
   marker.color.g = 1.0f;
   marker.color.b = 0.0f;
   marker.color.a = 1.0;

   marker.pose.position.x = 0.0;
   marker.pose.position.y = 0.0;
   marker.pose.position.z = 0.0;

   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 1.0/sqrt(2);
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0/sqrt(2);

   marker.lifetime = ros::Duration();

   max_rotation_range = max;
   min_rotation_range = min;
}

void SteeringTire::renew(std::string tire_id,int num_id,double length,double width,double max,double min){
	marker.header.frame_id=tire_id;
	marker.ns=tire_id;
	marker.id=num_id;
	marker.type=visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = length;
	marker.scale.y = length;
	marker.scale.z = width;

	marker.color.r = 0.0f;
   marker.color.g = 1.0f;
   marker.color.b = 0.0f;
   marker.color.a = 1.0;

   marker.pose.position.x = 0.0;
   marker.pose.position.y = 0.0;
   marker.pose.position.z = 0.0;

   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 1.0/sqrt(2);
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0/sqrt(2);

   marker.lifetime = ros::Duration();

   max_rotation_range = max;
   min_rotation_range = min;
}

void SteeringTire::setYaw(double yaw){
	if((yaw<=max_rotation_range)&&(yaw>=min_rotation_range)) {
		q.setRPY(0.0,0.0,yaw);
		now_yaw=yaw;
	}

	tf.setRotation(q);
}

/////////////////////////////////////////////////////////////class 4wheel_steering

class Steering_4{
	public:
		ros::NodeHandle n;
		ros::Publisher marker_pub;

		geometry_msgs::Pose2D pose;

		tf::TransformBroadcaster m_br;
  		tf::Transform m_tf;
		tf::Quaternion m_q;
		visualization_msgs::MarkerArray m_a;

		SteeringTire tire[4];

		Steering_4(double length,double width,double max_range,double min_range);
		void set_pose(geometry_msgs::Pose2D now_pose);
		void set_wheels(double yaw1,double yaw2,double yaw3,double yaw4);
		void update();
};

Steering_4::Steering_4(double length,double width,double max_range,double min_range){
   marker_pub = n.advertise<visualization_msgs::MarkerArray>("tire_markers", 10);

   tire[0].renew("/tire1",1,0.08,0.025,max_range,min_range);
   tire[1].renew("/tire2",2,0.08,0.025,max_range,min_range);
   tire[2].renew("/tire3",3,0.08,0.025,max_range,min_range);
   tire[3].renew("/tire4",4,0.08,0.025,max_range,min_range);

   pose.x=0;
   pose.y=0;
   pose.theta=0;
   set_pose(pose);

   tire[0].tf.setOrigin(tf::Vector3(width/2.0,length/2.0,0.0));
   tire[1].tf.setOrigin(tf::Vector3((-1)*width/2.0,length/2.0,0.0));
   tire[2].tf.setOrigin(tf::Vector3((-1)*width/2.0,(-1)*length/2.0,0.0));
   tire[3].tf.setOrigin(tf::Vector3(width/2.0,(-1)*length/2.0,0.0));

   set_wheels(0,0,0,0);

}

void Steering_4::set_pose(geometry_msgs::Pose2D now_pose){
	m_q.setRPY(0,0,now_pose.theta);
  m_tf.setRotation(m_q);
  m_tf.setOrigin( tf::Vector3(now_pose.x,now_pose.y,0.0));
}

void Steering_4::set_wheels(double yaw1,double yaw2,double yaw3,double yaw4){
	tire[0].setYaw(yaw1);
	tire[1].setYaw(yaw2);
	tire[2].setYaw(yaw3);
	tire[3].setYaw(yaw4);
}

void Steering_4::update(){
	m_a.markers.clear();
	for(int i=0;i<4;i++){
		tire[i].marker.header.stamp = ros::Time::now();
   	m_a.markers.push_back(tire[i].marker);
   }
   marker_pub.publish(m_a);

   m_br.sendTransform(tf::StampedTransform(m_tf, ros::Time::now(), "/global", "/body"));
   tire[0].br.sendTransform(tf::StampedTransform(tire[0].tf, ros::Time::now(), "/body", "/tire1"));
   tire[1].br.sendTransform(tf::StampedTransform(tire[1].tf, ros::Time::now(), "/body", "/tire2"));
   tire[2].br.sendTransform(tf::StampedTransform(tire[2].tf, ros::Time::now(), "/body", "/tire3"));
   tire[3].br.sendTransform(tf::StampedTransform(tire[3].tf, ros::Time::now(), "/body", "/tire4"));
}
