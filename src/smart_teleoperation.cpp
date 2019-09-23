// Yanjun's controller for smart_teleoperation. These codes are adapted from Dr. Newman's program. 9/16/19

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <stdlib.h>


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot
#define PI 3.14159265
// these values to be set within the laser callback
float ping_dist_in_front_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
float R = 0.15; // the radius of the robot
int ping_index_right_ = -1;
int ping_index_left_ = -1;
int ping_index_ = -1; // NOT real; callback will have to find this
int index_to_check = -1;
double angle_to_check = 0.0;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
double range_to_check = 0.0;
bool laser_alarm_ = false;
char *topic_name;


geometry_msgs::Twist twist_cmd;
geometry_msgs::Twist twist_output;

ros::Publisher vel_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void des_vel_Callback(const geometry_msgs::Twist& cmd) {
	twist_cmd = cmd;
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	if (ping_index_ < 0) {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;


		// what is the index of the ping that is straight ahead?
		// BETTER would be to use transforms, which would reference how the LIDAR is mounted;
		// but this will do for simple illustration
		ping_index_ = (int)((0.0 - angle_min_) / angle_increment_);
		ping_index_right_ = (int)((-PI / 2 - angle_min_) / angle_increment_);
		ping_index_left_ = (int)((PI / 2 - angle_min_) / angle_increment_);
		ROS_DEBUG("LIDAR setup: ping_index = %d", ping_index_);

	}

	ping_dist_in_front_ = laser_scan.ranges[ping_index_];
	index_to_check = ping_index_right_;
	angle_to_check = angle_min_ + index_to_check * angle_increment_ + PI / 2;
	laser_alarm_ = false;

	while (index_to_check < ping_index_left_ + 1)
	{
		range_to_check = laser_scan.ranges[index_to_check];
		if (angle_to_check < atan(MIN_SAFE_DISTANCE / R)) {
			if (range_to_check < R / cos(angle_to_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		if ((angle_to_check > atan(MIN_SAFE_DISTANCE / R)) && (angle_to_check < PI - atan(MIN_SAFE_DISTANCE / R))) {
			if (range_to_check < MIN_SAFE_DISTANCE/sin(angle_to_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		if (angle_to_check > PI - atan(MIN_SAFE_DISTANCE / R)) {
			if (range_to_check < R / cos(PI - angle_to_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		index_to_check++;
		angle_to_check = angle_to_check + angle_increment_;
	}
	//ROS_INFO("angle wrong = %f", angle_to_check);
        //ROS_INFO("index wrong = %i", index_to_check);
	//ROS_INFO("min dis = %f", R / cos(angle_to_check));
	//ROS_INFO("my dis = %f", range_to_check);
	twist_output = twist_cmd;
	if (laser_alarm_) {
		ROS_WARN("DANGER, WILL ROBINSON!!");
	        ROS_INFO("DISTANCE = %f", range_to_check);
                ROS_INFO("DIRECTION = %f", angle_to_check - PI / 2);
		twist_output.linear.x = (twist_output.linear.x) / 10;
		//if ping_dist_in_front_< MIN_SAFE_DISTANCE, decrease the speed
		if (angle_to_check < PI/2) {
			twist_output.angular.z = 10 + twist_output.angular.z;
		}
		else {
			twist_output.angular.z = -10 + twist_output.angular.z;
		}
	}
	//if ping_dist_in_front_< MIN_SAFE_DISTANCE, increase the angular velocity
	vel_publisher_.publish(twist_output);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "smart_teleoperation"); //name this node
	// Process command line parameter looking for a -n string name
	// and should be placed after the ros::init() invocation.
	// rosrun <package_name> <executable_name> -n <new_name>
	// or
	// rosrun subscriber_package subscriber_node -n alternate_topic
	int opt;
	while ((opt = getopt(argc, (argv), "n:")) != -1) {
  		switch (opt) {
    			case 'n':
      				topic_name = optarg;
      				break;
    			default:
      				printf("The -%c is not a recognized parameter\n", opt);
      				break; 
  		}
	}
	ros::NodeHandle nh;
	//create a Subscriber object and have it subscribe to the lidar topic
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	vel_publisher_ = pub; // let's make this global, so callback can use it
	ros::Subscriber des_vel = nh.subscribe("des_vel", 1, des_vel_Callback);
	ros::Subscriber lidar_subscriber = nh.subscribe(topic_name, 1, laserCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}
