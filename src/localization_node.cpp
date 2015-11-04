#include<ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdio.h>
#include <stdlib.h>


//#include <geometry_msgs>


class localization_node
{

public:
    ros::NodeHandle n_;
    ros::Publisher localization_pub_;

    ros::Publisher vis_pub_;
    ros::Subscriber encoder_sub_;

    geometry_msgs::Pose2D pose_;


    /*Pose variables*/

    /*Deltas*/

    double delta_sl_;
    double delta_sr_;
    double delta_s_;

    double delta_x_;
    double delta_y_;
    double delta_theta_;




    /*Kinematic constants*/

    double b_; //Wheel base
    double d_; //Wheel diameter

    double degrees_per_tick_;


    /*Marker variables*/

    /*TODO rename variables with trailing underscore(_) OR move to another node*/



    visualization_msgs::Marker marker;



    localization_node()
    {
        n_ = ros::NodeHandle("n");

        encoder_sub_ = n_.subscribe("/arduino/encoders",1,&localization_node::encoderCallback, this);

        localization_pub_ = n_.advertise<geometry_msgs::Pose2D>("/localization/encoder",1);

        vis_pub_ = n_.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 ); //topic name to be changed

        degrees_per_tick_ = 1.0;

        /*Initializing constants*/

        b_ = 0.21; //Wheel base
        d_ = 0.1; //Wheel diameter

        /*Marker initialization*/

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;

        //marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 5;
        marker.pose.position.y = 5;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";



    }

    void encoderCallback(const ras_arduino_msgs::Encoders encodermsg)
    {

        /*Delta Calculations*/
        delta_sl_ = ((M_PI*d_)*encodermsg.delta_encoder1)/360.0; //Meters
        delta_sr_ = ((M_PI*d_)*encodermsg.delta_encoder2)/360.0; //Meters

        delta_s_ = (delta_sl_+ delta_sr_)/2.0; //Meters

        delta_theta_ = ((delta_sr_ - delta_sl_)/b_); //Degrees?

        delta_x_ = delta_s_*cos(pose_.theta + (delta_theta_/2.0));

        delta_y_ = delta_s_*sin(pose_.theta + (delta_theta_/2.0));

        /*Pose Calculations*/




        /*Update*/

        pose_.x = pose_.x + delta_x_*0.10;

        pose_.y = pose_.y + delta_y_*0.10;

        pose_.theta = (pose_.theta + delta_theta_); //Radians

        pose_.theta = fmod(pose_.theta,2*3.14159); //Modulus to get values in 0 - 2 pi range

        //pose_.theta = pose_.theta * (180.0/3.141592); //Degrees

        /*Marker update*/
        marker.pose.position.x = pose_.x;
        marker.pose.position.y = pose_.y;


    }

    void update()
    {
        localization_pub_.publish(pose_);
        vis_pub_.publish( marker );

    }



};

int main(int argc, char **argv)

{

    ros::init(argc, argv, "localization_node");

    localization_node l_node;

    ros::Rate loop_rate(10);
    //l_node.vis_pub_.publish( marker );


    while(l_node.n_.ok())
    {
        l_node.update();
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}
