#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Imu.h>
#include <realsense2_camera/IMUInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

rosbag::Bag bag;


void imu_accel_info_Callback(const realsense2_camera::IMUInfo &msg){
     bag.write("/camera/accel/imu_info", ros::Time::now(), msg);
}

void imu_gyro_info_Callback(const realsense2_camera::IMUInfo &msg){
     bag.write("/camera/gyro/imu_info", ros::Time::now(), msg);
}

void imuCallback(const sensor_msgs::Imu &msg){
     bag.write("/camera/imu", msg.header.stamp, msg);
}

void image_callback(const sensor_msgs::ImageConstPtr& infra1, const sensor_msgs::CameraInfoConstPtr& infra1_info,
                        const sensor_msgs::ImageConstPtr& infra2, const sensor_msgs::CameraInfoConstPtr& infra2_info){

    bag.write("/camera/infra1/image_rect_raw", infra1->header.stamp, infra1);
    bag.write("/camera/infra1/camera_info", infra1->header.stamp, infra1_info);
    bag.write("/camera/infra2/image_rect_raw", infra1->header.stamp, infra2);
    bag.write("/camera/infra2/camera_info", infra1->header.stamp, infra2_info);
}

void tf_Callback(const tf2_msgs::TFMessage& msg){
    bag.write("/tf_static",msg.transforms[0].header.stamp,msg);
}

int main(int argc, char** argv){

    ros::init(argc,argv,"clean_bag");
    ros::NodeHandle nh;

    ROS_INFO("In the node\n");

    bag.open("test_clean.bag", rosbag::bagmode::Write);

    message_filters::Subscriber<sensor_msgs::Image> infra1_sub(nh, "/camera/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> infra1_info_sub(nh, "/camera/infra1/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::Image> infra2_sub(nh, "/camera/infra2/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> infra2_info_sub(nh, "/camera/infra2/camera_info", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,sensor_msgs::Image, sensor_msgs::CameraInfo>
                                            sync(infra1_sub, infra1_info_sub,infra2_sub, infra2_info_sub,10);
    sync.registerCallback(boost::bind(&image_callback, _1, _2,_3,_4));


    ros::Subscriber imu_sub = nh.subscribe("/camera/imu", 1000, imuCallback);
    ros::Subscriber imu_info_sub = nh.subscribe("/camera/accel/imu_info", 10, imu_accel_info_Callback);
    ros::Subscriber imu_gyro_sub = nh.subscribe("/camera/gyro/imu_info", 10, imu_gyro_info_Callback);
    ros::Subscriber tf_sub = nh.subscribe("/tf_static", 10, tf_Callback);

    ros::spin();
    bag.close();
    return 0;

}