#include "nav_msgs/Path.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h> 
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/action/action.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include "ros/time.h"
#include <std_msgs/Empty.h>
#include "api_mavlink_tcp/Switch.h"

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::microseconds;

const uint8_t  MODEL_MAN = 1; //手动模式
const uint8_t  MODEL_BUILD_MAP = 2;//建图模式
const uint8_t  MODEL_NAV = 3;//导航模式
const uint8_t  MODEL_CHARGE = 4;//回充模式
const uint8_t  MODEL_NONE = 0;//默认模式
//扫描待发缓冲池的频率，可以通过增大该值来加快发送的响应速度
const uint8_t HZ=5;
const uint8_t PAYLOAD_SIZE=253;

Mavsdk mavsdk_cc_0;
bool isFrame_0=false;
Mavsdk mavsdk_cc_1;
bool isFrame_1=false;


struct SendData {
    
    uint8_t* data_0;//通道0的数据内容
    uint32_t size_0;//通道0的数据大小
    bool isflag;
};

void Path_global_cb(const nav_msgs::Path::ConstPtr& msg);
void Path_local_cb(const nav_msgs::Path::ConstPtr& msg);
void send_handshake_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,uint32_t data_size);
void send_path_data_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,uint8_t* data,uint32_t from,uint8_t payload_size,uint16_t seqnr);
void send_path_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,SendData& sendData);


//缓冲池
std::vector<SendData> send_data_buffer;
std::vector<SendData> send_data_buffer_local;
//最终要发送的数据对象
SendData send_data_final;
SendData send_data_final_local;


ros::ServiceClient client;

void Path_global_cb(const nav_msgs::Path::ConstPtr& msg){

   uint32_t totalSize=msg->poses.size()*6;   //每个点取x和y两个数 
   //转换数据
   SendData send_data;
   send_data.isflag=true;

   send_data.size_0=totalSize;
   uint8_t* dataBytes_0=new uint8_t[send_data.size_0];
   
   for(u_int32_t i=0;i<totalSize/6;i++){

    if(msg->poses[i].pose.position.x>=0){
      dataBytes_0[6*i+0]=0;//符号0：+、1：-
    }else{
      dataBytes_0[6*i+0]=1;
    }
    uint32_t temp_x=abs(msg->poses[i].pose.position.x)*100;//取两位小数 
    float temp_x_1=temp_x/800.0f;//缩放大小 
    uint8_t x_value_1=(uint8_t)temp_x_1;
    uint8_t x_value_2=(uint8_t)(((float)temp_x_1-x_value_1)*100);
    dataBytes_0[6*i+1]=x_value_1;
    dataBytes_0[6*i+2]=x_value_2;

    if(msg->poses[i].pose.position.y>=0){
      dataBytes_0[6*i+3]=0;//符号0：+、1：-
    }else{
      dataBytes_0[6*i+3]=1;
    }

    uint32_t temp_y=abs(msg->poses[i].pose.position.y)*100;//取两位小数 
    float temp_y_1=temp_y/800.0f;//缩放大小 
    uint8_t y_value_1=(uint8_t)temp_y_1;
    uint8_t y_value_2=(uint8_t)(((float)temp_y_1-y_value_1)*100);
    dataBytes_0[6*i+4]=y_value_1;
    dataBytes_0[6*i+5]=y_value_2;


   }

   send_data.data_0 = dataBytes_0;
   send_data_buffer.emplace_back(send_data);//加入缓存容器，等待Mavlink发送

}

void Path_local_cb(const nav_msgs::Path::ConstPtr& msg){

    // ROS_ERROR("--------------Path_local_cb=%ld",msg->poses.size());
    uint32_t totalSize=msg->poses.size()*6;   //每个点取x和y两个数 
    //转换数据
    SendData send_data;
    send_data.isflag=true;
    tf::TransformListener listener; //定义监听器

    geometry_msgs::PointStamped* dataBytes_poits=new geometry_msgs::PointStamped[msg->poses.size()];
    listener.waitForTransform("/map","/odom",ros::Time(0),ros::Duration(0.22));
    for(u_int32_t i=0;i<msg->poses.size();i++){
          geometry_msgs::PointStamped point_src;
          point_src.point.x=msg->poses[i].pose.position.x;
          point_src.point.y=msg->poses[i].pose.position.y;
          point_src.point.z=msg->poses[i].pose.position.z;
          point_src.header.stamp=ros::Time();
          point_src.header.frame_id="/odom";
          geometry_msgs::PointStamped point_target;
          try{
            
             listener.transformPoint("/map",point_src,point_target);  //将odom坐标系下的点坐标转换为map坐标系下
          }
          catch (tf::TransformException &ex) {
            ROS_ERROR("Offbord>>%s",ex.what());
          }
          dataBytes_poits[i]=point_target;

   }

   send_data.size_0=totalSize;
   uint8_t* dataBytes_0=new uint8_t[send_data.size_0];
  
   for(u_int32_t i=0;i<totalSize/6;i++){

    if(dataBytes_poits[i].point.x>=0){
      dataBytes_0[6*i+0]=0;//符号0：+、1：-
    }else{
      dataBytes_0[6*i+0]=1;
    }
    uint32_t temp_x=abs(dataBytes_poits[i].point.x)*100;//取两位小数 
    float temp_x_1=temp_x/800.0f;//缩放大小 
    uint8_t x_value_1=(uint8_t)temp_x_1;
    uint8_t x_value_2=(uint8_t)(((float)temp_x_1-x_value_1)*100);
    dataBytes_0[6*i+1]=x_value_1;
    dataBytes_0[6*i+2]=x_value_2;

    if(dataBytes_poits[i].point.y>=0){
      dataBytes_0[6*i+3]=0;//符号0：+、1：-
    }else{
      dataBytes_0[6*i+3]=1;
    }

    uint32_t temp_y=abs(dataBytes_poits[i].point.y)*100;//取两位小数 
    float temp_y_1=temp_y/800.0f;//缩放大小 
    uint8_t y_value_1=(uint8_t)temp_y_1;
    uint8_t y_value_2=(uint8_t)(((float)temp_y_1-y_value_1)*100);
    dataBytes_0[6*i+4]=y_value_1;
    dataBytes_0[6*i+5]=y_value_2;
    
   }


   send_data.data_0 = dataBytes_0;
   send_data_buffer_local.emplace_back(send_data);//加入缓存容器，等待Mavlink发送

}

int main(int argc, char **argv)
{
	  ros::init(argc, argv, "offboard_nav_manager");
	  ros::NodeHandle nh;

    client = nh.serviceClient<api_mavlink_tcp::Switch>("/model_status");
    ros::Subscriber path_global_sub = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, Path_global_cb);   //该话题的坐标系为map
    ros::Subscriber path_local_sub = nh.subscribe<nav_msgs::Path>("/move_base/DWAPlannerROS/local_plan", 10, Path_local_cb);//注意：该话题的坐标系为odom,非map


    sleep(10);//延时10s，等待RTAB启动完成

    api_mavlink_tcp::Switch srv;
    srv.request.param1=3;  //导航模式
    if (client.call(srv))
    {
        ROS_ERROR("change to nav model !");

    }else{
        ROS_ERROR("No Response");
    }

    ConnectionResult connection_result_0 = mavsdk_cc_0.add_any_connection("tcp://192.168.10.92:15550");  
    if (connection_result_0 != ConnectionResult::Success) {
        ROS_ERROR("0 Adding connection failed !");

    }
    ConnectionResult connection_result_1 = mavsdk_cc_1.add_any_connection("tcp://192.168.10.92:15551");  
    if (connection_result_1 != ConnectionResult::Success) {
        ROS_ERROR("1 Adding connection failed !");

    }

    ros::Rate rate(HZ);
    //等待客户端系统连接
	  while(ros::ok() && (mavsdk_cc_0.systems().size() == 0||mavsdk_cc_1.systems().size() == 0)){
		    ros::spinOnce();
        ROS_ERROR("wait !");
		   rate.sleep();
	  }

    //监听客户端系统连接状态
    mavsdk_cc_0.subscribe_on_new_system([]() {
        auto system = mavsdk_cc_0.systems().back();
        if (system->is_connected()) {
            ROS_ERROR("mavsdk_cc_0---------------discovered  a new client tcp---------------");

        } else {
            ROS_ERROR("xxxxxxxxxxxxxxx the client is disconnect tcp xxxxxxxxxxxxxxx");
        }

        isFrame_0=false;
    });
     //监听客户端系统连接状态
    mavsdk_cc_1.subscribe_on_new_system([]() {
        auto system = mavsdk_cc_1.systems().back();
        if (system->is_connected()) {
            ROS_ERROR("mavsdk_cc_1---------------discovered  a new client tcp---------------");

        } else {
            ROS_ERROR("xxxxxxxxxxxxxxx the client is disconnect tcp xxxxxxxxxxxxxxx");
        }

        isFrame_1=false;
    });

    auto system_0=mavsdk_cc_0.systems().at(0);
    auto mavlink_passthrough_0 = MavlinkPassthrough{system_0};  

    auto system_1=mavsdk_cc_1.systems().at(0);
    auto mavlink_passthrough_1 = MavlinkPassthrough{system_1};  


    ROS_ERROR("TCP Waiting for commands ! tcp");

     mavlink_passthrough_0.subscribe_message_async(MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, [&mavlink_passthrough_0](const mavlink_message_t& message) {
          mavlink_data_transmission_handshake_t msg;
          ROS_WARN("xxxxxxxxxxxxxxx the client is disconnect tcp xxxxxxxxxxxxxxx");

          mavlink_msg_data_transmission_handshake_decode(&message, &msg);
          ROS_ERROR("0>>size: %d,  width: %d,  height: %d,  packets: %d,  type: %d, systemID=%d ",msg.size,msg.width,msg.height,msg.packets,msg.type,message.sysid);
          if(send_data_buffer.size()>0){
             send_path_with_id(110,mavlink_passthrough_0,send_data_final);    
          }
      
    });
     mavlink_passthrough_1.subscribe_message_async(MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, [&mavlink_passthrough_1](const mavlink_message_t& message) {
          mavlink_data_transmission_handshake_t msg;
          ROS_WARN("xxxxxxxxxxxxxxx the client is disconnect tcp xxxxxxxxxxxxxxx");

          mavlink_msg_data_transmission_handshake_decode(&message, &msg);
          // ROS_ERROR("1>>size: %d,  width: %d,  height: %d,  packets: %d,  type: %d, systemID=%d ",msg.size,msg.width,msg.height,msg.packets,msg.type,message.sysid);
          if(send_data_buffer_local.size()>0){
             send_path_with_id(111,mavlink_passthrough_1,send_data_final_local);    
          }
      
    });
    while(ros::ok()){
        if(isFrame_0==false){

          if(send_data_buffer.size()>1){
             ROS_INFO("send_handshake again tcp");
             send_data_final=send_data_buffer.back();

             send_handshake_with_id(system_0->get_system_id(),mavlink_passthrough_0,send_data_final.size_0);
            
            
          }
        }
        if(isFrame_1==false){

          if(send_data_buffer_local.size()>1){
             ROS_INFO("send_handshake again tcp");
             send_data_final_local=send_data_buffer_local.back();

             send_handshake_with_id(system_1->get_system_id(),mavlink_passthrough_1,send_data_final_local.size_0);
            
            
          }
        }
      

        ros::spinOnce();
		    rate.sleep();
    }

	return EXIT_SUCCESS;
}


 /**
  * @brief  发送握手协议
  * 
  * @param mavlink_passthrough 
  * @param data_size  路径字节数组大小
  */

void send_handshake_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,uint32_t data_size)
{
   if(system_id==110){
        isFrame_0=true;
    }else if(system_id==111){
        isFrame_1=true;
    }
   
    uint16_t packetCount=data_size/PAYLOAD_SIZE;
    
    if(data_size %(PAYLOAD_SIZE)>0){
      packetCount=packetCount+1;
    }

    // ROS_WARN("packetCount=%d" ,packetCount);
    mavlink_message_t message;

    mavlink_msg_data_transmission_handshake_pack( 
        system_id,
        mavlink_passthrough.get_our_compid(),
        &message,
        MAVLINK_DATA_STREAM_IMG_PGM,
        data_size,
        0,
        0,
        packetCount,
        PAYLOAD_SIZE,
        100
        );

    mavlink_passthrough.send_message(message);
}



 /**
  * @brief 分批发送路径数据
  * 
  * @param mavlink_passthrough 
  * @param SendData  路径对象
  */

void send_path_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,SendData& sendData){
         uint16_t packets_0=sendData.size_0/PAYLOAD_SIZE;
         ROS_WARN("tcp----------------------------------------send begin");

         ROS_WARN("systemID=%d    tcp size:%d",system_id,sendData.size_0);
        switch (system_id)
         {    
          case 110:
               if(sendData.size_0 %(PAYLOAD_SIZE)>0){
                  packets_0=packets_0+1;
               }

               for(uint16_t i=0;i<packets_0;i++){
                  //逐一发送
                  send_path_data_with_id(system_id,mavlink_passthrough,sendData.data_0,i*PAYLOAD_SIZE,PAYLOAD_SIZE,i);
               }
               send_data_buffer.erase(send_data_buffer.begin(),send_data_buffer.end()-1);
               //  send_data_buffer.clear();
               isFrame_0=false;
              //  std::cout << "tcp clear all data, send_data_buffer.size="<<send_data_buffer.size()<< '\n';
              //  std::cout << "tcp ----------------------------------------send over"<< '\n';
            break;

          case 111:
               if(sendData.size_0 %(PAYLOAD_SIZE)>0){
                  packets_0=packets_0+1;
               }

               for(uint16_t i=0;i<packets_0;i++){
                  //逐一发送
                  send_path_data_with_id(system_id,mavlink_passthrough,sendData.data_0,i*PAYLOAD_SIZE,PAYLOAD_SIZE,i);
               }

               send_data_buffer_local.erase(send_data_buffer_local.begin(),send_data_buffer_local.end()-1);
               //  send_data_buffer_local.clear();
               isFrame_1=false;
               std::cout << "tcp clear all data, send_data_buffer_local.size="<<send_data_buffer_local.size()<< '\n';
               std::cout << "tcp ----------------------------------------send over"<< '\n';
            break;
          default:
            break;
        }
        


}

/**
 * @brief 发送 encapsulated_data数据，即拆分后的路径元包
 * 
 * @param mavlink_passthrough 
 * @param data 路径字节数据
 * @param from 拆分起点下标
 * @param payload_size 每帧大小
 * @param seqnr 每帧序号
 */

void send_path_data_with_id(uint8_t system_id,MavlinkPassthrough& mavlink_passthrough,uint8_t* data,uint32_t from,uint8_t payload_size,uint16_t seqnr)
{
    mavlink_message_t message;
    uint8_t* sendData =new uint8_t[payload_size];

    std::copy(data+from,data+(from+payload_size),sendData);  //拆分数据包

    mavlink_msg_encapsulated_data_pack(
        system_id,
        mavlink_passthrough.get_our_compid(),
        &message,
        seqnr,
        sendData
    );
    mavlink_passthrough.send_message(message);
}

