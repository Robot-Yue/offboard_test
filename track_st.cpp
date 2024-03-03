#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

ros::Publisher mavros_rc_publisher;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    mavros_msgs::OverrideRCIn rc_msg;

    // 根据 /cmd_vel 线性和角速度计算机器人的左右轮速度
    double linear_vel = msg->twist.linear.x;
    double angular_vel = msg->twist.angular.z;

    double right_wheel_speed = linear_vel + angular_vel;
    double left_wheel_speed = linear_vel - angular_vel;

    // 将速度转换为 RC PWM 值，具体转换公式根据电机和控制器进行调整
    int right_pwm = static_cast<int>(right_wheel_speed * 1000);
    int left_pwm = static_cast<int>(left_wheel_speed * 1000);

    // 发布 RC PWM 值到 pixhawk
    // 这里假设机器人左右两个电机 RC 通道为 1 和 3
    rc_msg.channels[1] = left_pwm;
    rc_msg.channels[3] = right_pwm;

    mavros_rc_publisher.publish(rc_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "track_drive_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 创建一个发布器，用于发布 mavros 接收到的 RC PWM 值
    mavros_rc_publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    // 创建一个订阅器，用于接收 /cmd_vel 话题
    ros::Subscriber cmd_vel_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 10, cmdVelCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 设置飞控为 GUIDED 模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    // 设置飞控解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "GUIDED" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("GUIDED enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    
    return 0;
}
