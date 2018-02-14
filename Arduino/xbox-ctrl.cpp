/*
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class NeuroRacerTeleop
{

public:
  NeuroRacerTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  float leftTrigger, rightTrigger ;
  float crossVertical, crossHorizontal ;
  int leftTriggerIndex, rightTriggerIndex ;
  int crossVerticalIndex, crossHorizontalIndex ;

  // top right hand buttons
  int xButton, yButton, bButton, aButton;
  // buttons next to triggers
  int leftButton, rightButton ;
  // app button
  int appButton ;

  bool throttleInitialized ;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;

  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool aPressed, bPressed, yPressed, xPressed;
  bool leftPressed, rightPressed;
  bool appPressed ;
  bool followerOn ;
  std::string buttonASound, buttonBSound, buttonXSound, buttonYSound ;
  std::string buttonLeftSound, buttonRightSound ;
  std::string buttonAppSoundOn, buttonAppSoundOff ;
  bool zero_twist_published_;
  ros::Timer timer_;
};

NeuroRacerTeleop::NeuroRacerTeleop():
	ph_("~"),
	linear_(1),
	angular_(0),
	deadman_axis_(4),
	l_scale_(0.3),
	leftTriggerIndex(5),
	rightTriggerIndex(4),
	crossVerticalIndex(7),
	crossHorizontalIndex(6),
	leftTrigger(0),
	rightTrigger(0),
	crossVertical(0),
	crossHorizontal(0),
	xButton(3),
	bButton(1),
	//aButton(0),
	leftButton(6),
	rightButton(7),
	appButton(11),
	aPressed(false),
	bPressed(false),
	xPressed(false),
	yPressed(false),
	leftPressed(false),
	rightPressed(false),
	appPressed(false),
	followerOn(false),
	a_scale_(0.9),
	throttleInitialized(false)
{
	ph_.param("axis_linear", linear_, linear_);
	ph_.param("axis_angular", angular_, angular_);
	ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
	ph_.param("scale_angular", a_scale_, a_scale_);
	ph_.param("scale_linear", l_scale_, l_scale_);

	ph_.param("button_a", aButton, aButton);
	ph_.param("button_b", bButton, bButton);
	ph_.param("button_x", xButton, xButton);
	ph_.param("button_left", leftButton, leftButton);
	ph_.param("button_right", rightButton, rightButton);
	ph_.param("button_app", appButton, appButton);
	buttonASound="noSound" ;
	ph_.param("buttonASound", buttonASound, buttonASound);
	ph_.param("buttonBSound", buttonBSound, buttonASound);
	ph_.param("buttonXSound", buttonXSound, buttonASound);
	ph_.param("buttonYSound", buttonYSound, buttonASound);

	ph_.param("buttonLeftSound", buttonLeftSound, buttonASound);
	ph_.param("buttonRightSound", buttonRightSound, buttonASound);

	ph_.param("buttonAppSoundOn", buttonAppSoundOn, buttonASound) ;
	ph_.param("buttonAppSoundOff", buttonAppSoundOff, buttonASound) ;

	// Get the other trigger and cross buttons
	ph_.param("leftTrigger",leftTriggerIndex,leftTriggerIndex) ;
	ph_.param("rightTrigger",rightTriggerIndex,rightTriggerIndex) ;
	ph_.param("crossAxisHorizontal",crossHorizontalIndex,crossHorizontalIndex) ;
	ph_.param("crossAxisVertical",crossVerticalIndex,crossVerticalIndex) ;

	deadman_pressed_ = false;
	zero_twist_published_ = false;

	vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &NeuroRacerTeleop::joyCallback, this);

	timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&NeuroRacerTeleop::publish, this));
}

void sleepok(int t, ros::NodeHandle &nh) {
	if (nh.ok()) sleep(t);
}

void NeuroRacerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	// A Button als Deadman setzen
	deadman_pressed_ = joy->buttons[aButton];
	//aPressed = joy->buttons[aButton] == 1;
	bPressed = joy->buttons[bButton] == 1;
	xPressed = joy->buttons[xButton] == 1 ;
	leftPressed = joy->buttons[leftButton] == 1 ;
	rightPressed = joy->buttons[rightButton] == 1;
	appPressed = joy->buttons[appButton] == 1 ;

	leftTrigger = joy->axes[leftTriggerIndex];
	rightTrigger = joy->axes[rightTriggerIndex];
	crossVertical = joy->axes[crossVerticalIndex];
	crossHorizontal = joy->axes[crossHorizontalIndex];

	// On some controllers, the right trigger is zero until it is used the first time
	geometry_msgs::Twist vel;
	if (!throttleInitialized && (joy->axes[linear_] != 0.0)) {
		throttleInitialized = true ;
	}

	// Steering WInkel
	//
	// TODO: Mappen
	vel.angular.z = a_scale_*joy->axes[angular_];

	// ESC regulieren
	if (throttleInitialized == true) {
		vel.linear.x = l_scale_*joy->axes[rightTriggerIndex];
	} else {
		vel.linear.x = l_scale_*0.0;
	}

	last_published_ = vel;
}

void NeuroRacerTeleop::publish() {
	boost::mutex::scoped_lock lock(publish_mutex_);

	if (deadman_pressed_) {
		vel_pub_.publish(last_published_);

		zero_twist_published_=false;
	}
	else if(!deadman_pressed_ && !zero_twist_published_) {
		// set steering to middle, throttle off
		geometry_msgs::Twist twist;
		twist.angular.z = 0.0;
		twist.linear.x = 0.0;
		vel_pub_.publish(twist);

		zero_twist_published_=true;
	}
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "neuroracer_teleop");
  ros::NodeHandle nh;
  NeuroRacerTeleop neuroracer_teleop;
  ros::spin();
}
