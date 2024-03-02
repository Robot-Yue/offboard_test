#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_broadcaster.h> 
#include <tf/message_filter.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//协方差
const double odom_pose_covariance[36] =
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] =  //静止时
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] =
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = //静止时
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9};

ros::Publisher odom_publisher;

ros::Publisher final_odom_publisher;

ros::Publisher final_imu_publisher;

geometry_msgs::TwistWithCovariance curTwist;

bool needTransformNed;
int count;

void wheel_odometry_cb(const nav_msgs::Odometry::ConstPtr& odom){

	  nav_msgs::Odometry filter_odom;
	  filter_odom.header=odom->header;
    filter_odom.child_frame_id=odom->child_frame_id;


    //-----------------------------------------------------------
    if(needTransformNed){
      //飞控下NED坐标系转ROS下的ENU坐标系,转换规则 
      //参考：https://bbs.amovlab.com/forum.php?mod=viewthread&tid=975&extra=page%3D2
      filter_odom.pose.pose.position.x=odom->pose.pose.position.y;
      filter_odom.pose.pose.position.y=odom->pose.pose.position.x;
      filter_odom.pose.pose.position.z=-odom->pose.pose.position.z;
      // tf::Quaternion quat;
      // tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
      // double roll, pitch, yaw;
      // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      // filter_odom.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch,yaw-1.5708d);  // Y_Enu = Y_Ned - 90
      filter_odom.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);  // 赋予默认值，不用底盘上报的Yaw角
    }else{

      filter_odom.pose.pose=odom->pose.pose;
      if (count==-1)//2次判断完成后，处理以后的orientation
      {
        filter_odom.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);  // 赋予默认值，不用底盘上报的Yaw角

      }
      
    }

	  

	  filter_odom.twist.twist=odom->twist.twist;//twist不用转

    //----------------------------------------------------------
    curTwist=odom->twist;
   //还是有必要处理动态与静态之分，避免IMU的误差干扰后的异常
    if((odom->twist.twist.linear.x==0&&odom->twist.twist.linear.y==0)||(odom->twist.twist.linear.x==0&&odom->twist.twist.linear.y==-0.0)){//静止时
     
	    memcpy(&filter_odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
      memcpy(&filter_odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
   
    }else{ // 运动时
   
	    memcpy(&filter_odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
      memcpy(&filter_odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    }
  



	odom_publisher.publish(filter_odom);
}

void odom_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_combined){

    nav_msgs::Odometry filter_odom;
    filter_odom.header=odom_combined->header;
    filter_odom.child_frame_id="base_link";
	  filter_odom.pose=odom_combined->pose;
    filter_odom.twist=curTwist;
    final_odom_publisher.publish(filter_odom);

    

}

void imu_cb(const sensor_msgs::Imu::ConstPtr& imu){

    sensor_msgs::Imu filter_imu;
    
     filter_imu.header=imu->header;
     filter_imu.orientation=imu->orientation;
     filter_imu.orientation_covariance={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};//值：0 无相关性，即过滤掉mag信息，避免外界磁场干扰，影响航向角
     filter_imu.angular_velocity=imu->angular_velocity;
     filter_imu.angular_velocity_covariance={1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
     filter_imu.linear_acceleration=imu->linear_acceleration;
     filter_imu.linear_acceleration_covariance={1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    
    final_imu_publisher.publish(filter_imu);

}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "offboard_filter_msg");
	ros::NodeHandle nh;

    //处理odom的协方差
	ros::Subscriber wheel_odometry_sub = nh.subscribe<nav_msgs::Odometry>("mavros/wheel_odometry/odom", 10, wheel_odometry_cb);
	odom_publisher  = nh.advertise<nav_msgs::Odometry>("odom_filter", 10);

    //格式转换，由geometry_msgs::PoseWithCovarianceStamped -> nav_msgs::Odometry
	ros::Subscriber odom_combined_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", 10, odom_combined_cb);
  final_odom_publisher  = nh.advertise<nav_msgs::Odometry>("odom_final", 10);
   
    //过滤掉mag信息，避免外界磁场干扰，影响航向角
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 100, imu_cb);
  final_imu_publisher  = nh.advertise<sensor_msgs::Imu>("imu_final", 100);



  //-----------------------兼容Ardupliot与Mavros的Bug-----------------------


  needTransformNed=false; //默认不转换
  count=0;
  bool isFlag=false;
	ros::Rate rate(1.0);

  tf::TransformListener listener;
  listener.waitForTransform("odom", "base_link", ros::Time(), ros::Duration(2.0));
	
	while(nh.ok() && isFlag==false){

    tf::StampedTransform transform_odom;
    try
    {
      listener.lookupTransform("odom", "base_link", ros::Time(0), transform_odom);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose.orientation.x = transform_odom.getRotation().getX();
      pose_stamped.pose.orientation.y = transform_odom.getRotation().getY();
      pose_stamped.pose.orientation.z = transform_odom.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform_odom.getRotation().getW();

      tf::Quaternion quat_odom;
      tf::quaternionMsgToTF(pose_stamped.pose.orientation, quat_odom);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat_odom).getRPY(roll, pitch, yaw);
      
      ROS_ERROR("yaw=%f",yaw);
      count++;//计数

      //为保证可靠性，连续判断3次，达到目的后，退出判断：isFlag=true，标识：count=-1，区分是原始值还是wheel_odometry_cb处理后的值
      if(count>2){ 
         count=-1;
         isFlag=true;
          ROS_ERROR("last yaw=%f",yaw);
         if(yaw < 1.6 && yaw >= 1.5){//属于NED坐标系,每次AMP重启后，上报的Pose为NED坐标系，yaw角均为:PI/2, 即 1.57，
           needTransformNed=true; //当前为NED，启动坐标系转换
           ROS_ERROR("AMP first start ，the Pose of odom->base_link need Transform");

         }else{
           needTransformNed=false;//当前为ENU，不启动坐标系转换
           ROS_ERROR("the Pose of odom->base_link don't need Transform ");

         }
      }


    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("odom->base_link TransformException");
    }

		ros::spinOnce();
		rate.sleep();
	}

	ros::spin();
	return EXIT_SUCCESS;
}

