#include<ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

//#include <geometry_msgs>


class localization_node
{

public:
	ros::NodeHandle n_;
	ros::Publisher localization_pub_;
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



	
	
	localization_node()
	{	
		n_ = ros::NodeHandle("n");
		
		encoder_sub_ = n_.subscribe("/kobuki/encoders",1,&localization_node::encoderCallback, this);
		
		localization_pub_ = n_.advertise<geometry_msgs::Pose2D>("/localization/encoder",1);

        degrees_per_tick_ = 1.0;

        /*Initializing constants*/

        b_ = 0.23; //Wheel base
        d_ = 2 * 0.0352; //Wheel diameter

	
	}

	void encoderCallback(const ras_arduino_msgs::Encoders encodermsg)
	{

        /*Delta Calculations*/
        delta_sl_ = ((3.14159*d_)*encodermsg.delta_encoder1)/360.0;
        delta_sr_ = ((3.14159*d_)*encodermsg.delta_encoder2)/360.0;

        delta_s_ = (delta_sl_+delta_sr_)/2.0;

        delta_theta_ = (delta_sr_ - delta_sl_)/b_;

        delta_x_ = delta_s_*cos(pose_.theta + (delta_theta_/2.0));

        delta_y_ = delta_s_*sin(pose_.theta + (delta_theta_)/2.0);

        /*Pose Calculations*/




        /*Update*/

        pose_.x = pose_.x + delta_x_;

        pose_.y = pose_.y + delta_y_;

        pose_.theta = (pose_.theta + delta_theta_); //Radians

        pose_.theta = fmod(pose_.theta,2*3.14159); //Modulus to get values in 0 - 2 pi range

        //pose_.theta = pose_.theta * (180.0/3.141592); //Degrees
	}
	
	void update()
	{
		localization_pub_.publish(pose_);
	}

	
		
};

int main(int argc, char **argv)

{
	ros::init(argc, argv, "localization_node");


	
	
	localization_node l_node;
	
	ros::Rate loop_rate(10);
	
	while(l_node.n_.ok())
	{
		l_node.update();
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	
	
	return 0;
}
