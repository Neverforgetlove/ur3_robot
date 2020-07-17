#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "skeleton_track.h"
#include "signal.h"
#include "iostream"
#include "std_msgs/Float64.h"
#include "string"
#include "geometry_msgs/Point.h"
#include "cmath"

using namespace std;

bool flag = true;
void sigfunc(int sig)
{
    flag = false;
}

float angle(float x1, float y1, float x2, float y2,int angle_num)
{
	float angle_temp;
	double xx, yy;
	xx = x2 - x1;
	yy = y2 - y1;
	if (xx == 0.0)
		angle_temp = M_PI / 2.0;

    if(angle_num == 1){
		angle_temp = atan(xx / yy);
        if(angle_temp<0){
            angle_temp += M_PI;
        }   
    }
    if(angle_num == 2){
		angle_temp = atan(xx / yy);
        if(angle_temp <0){
            angle_temp = -angle_temp;
        }
        else{
            angle_temp = M_PI - angle_temp;
        }
    }

	return (angle_temp*57.29578 /*+ 180*/);
}


int main(int argc, char** argv)
{
    signal(SIGINT, sigfunc);
    ros::init(argc, argv, "skeleton_track");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_depth(nh), it_sk(nh);
    image_transport::Publisher depth_pub=it_depth.advertise("depth_image", 10);
    image_transport::Publisher sk_img_pub=it_sk.advertise("skeleton_image", 10);
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Point>("right_shoulder",1000);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Point>("right_elbow",1000);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::Point>("right_hand",1000);
    ros::Publisher angle1_pub = nh.advertise<std_msgs::Float64>("angle1",1);
    ros::Publisher angle2_pub = nh.advertise<std_msgs::Float64>("angle2",1);
    ros::Publisher angle3_pub = nh.advertise<std_msgs::Float64>("angle3",1);
    Orbbec_Camera cam;

    geometry_msgs::Point right_shoulder_point;
    geometry_msgs::Point right_elbow_point;
    geometry_msgs::Point right_hand_point;

    geometry_msgs::Point x_z_angle_vector1;
    geometry_msgs::Point x_z_angle_vector2;

    geometry_msgs::Point y_z_angle_vector1;
    geometry_msgs::Point y_z_angle_vector2;

    Skeleton_Track track(&cam);
    namedWindow("image");
    startWindowThread();
    track.start_track();
    ros::Rate loop(10);
    while(ros::ok() && flag ) {
        Mat depth = cam.getColorImage();
        sensor_msgs::ImagePtr img_depth;
        std_msgs::Header imgHeader;
        imgHeader.frame_id="orbbec_depth_frame";
        imgHeader.stamp=ros::Time::now();
        img_depth = cv_bridge::CvImage(imgHeader, sensor_msgs::image_encodings::MONO16, depth).toImageMsg();
        depth_pub.publish(img_depth);
        Mat img;
        depth.convertTo(img, CV_8U);
        if (!track.is_tracking()) {
            imshow("image", img);
            waitKey(15);
            continue;
        }
        
        vector<cv::Point3f> points = track.get_skeleton_point3f();
        if(points.size()) {
            track.draw_skeleton(track.convert_3fto2f(points), img);
            sensor_msgs::ImagePtr img_sk;
            std_msgs::Header imgHeader;
            imgHeader.frame_id="orbbec_depth_frame";
            imgHeader.stamp=ros::Time::now();
            img_sk = cv_bridge::CvImage(imgHeader, sensor_msgs::image_encodings::MONO8, img).toImageMsg();
            depth_pub.publish(img_sk);
            right_shoulder_point.x = points[3].x;
            right_shoulder_point.y = points[3].y;
            right_shoulder_point.z = points[3].z;
            pub1.publish(right_shoulder_point);

            right_elbow_point.x = points[5].x;
            right_elbow_point.y = points[5].y;
            right_elbow_point.z = points[5].z;
            pub2.publish(right_elbow_point);

            float x_z_angle = angle(points[3].x, points[3].z, points[5].x, points[5].z,1);

            std_msgs::Float64 msg;
            msg.data = x_z_angle;
            angle1_pub.publish(msg);

            float y_z_angle = angle(points[3].x, points[3].y, points[5].x, points[5].y,2);

            std_msgs::Float64 msg1;
            msg1.data = y_z_angle;
            angle2_pub.publish(msg1);

            right_hand_point.x = points[7].x;
            right_hand_point.y = points[7].y;
            right_hand_point.z = points[7].z;
            pub3.publish(right_hand_point);

        }
        imshow("image", img);
        waitKey(15);
        loop.sleep();
    }
    track.stop_track();

    return 0;
}
