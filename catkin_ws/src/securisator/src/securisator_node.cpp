#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"

#include "../../../devel/include/clusterisator/Persons.h"

#include <math.h>

#define PERSON_IMG_TOPIC "/clusterisator/person"
#define CAMERA_INFO_TOPIC "/depth/camera_info"

#define RECIEVED_DATA_TOPIC "/securisator/recieved_data"

#define SECURITY_DISTANCE 100

class Securisator {

private:
ros::Subscriber person_img_sub;
ros::Subscriber camera_info_sub;
ros::Publisher recieved_data_pub;

uint32_t h,w;

int resolution_factor;

double f;
double cx,cy;

std::vector<geometry_msgs::Point> robot;

public:

Securisator(){

	ros::NodeHandle nh;

	person_img_sub = nh.subscribe(PERSON_IMG_TOPIC, 1, &Securisator::personImageCallback, this);
	camera_info_sub = nh.subscribe(CAMERA_INFO_TOPIC, 1, &Securisator::cameraInfoCallback, this);
	
	recieved_data_pub = nh.advertise<sensor_msgs::Image>(RECIEVED_DATA_TOPIC, 5);

}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info){
	f = camera_info->K[0];
	cx = camera_info->K[2];
	cy = camera_info->K[5];
}

geometry_msgs::Point depth_to_cartesian(int x, int y, uint16_t depth){
	geometry_msgs::Point p;
	p.x = ((x-cx)*depth)/f;
	p.y = ((y-cy)*depth)/f;
	p.z = depth;
	return p;
}

void collect_robot_data(const clusterisator::Persons::ConstPtr& img_persons){

	const sensor_msgs::Image img = img_persons->img;
	
	recieved_data_pub.publish(img);

	uint32_t i,j;
	
	resolution_factor = img_persons->resolution_factor;

	bool found_robot = false;
	int i_robot = 0;
	
	geometry_msgs::Point min_robot,max_robot;
	for(i=0;i<h;i++){
		for(j=0;j<w;j++){
			uint8_t type = img.data[i*3*w+j*3+2];
			uint16_t depth = (img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3];
			if(type == img_persons->ROBOT){
				geometry_msgs::Point current = depth_to_cartesian(i,j,depth);
				if(!found_robot){
					min_robot = current;
					max_robot = current;
					//min_robot.x = current.x;
					//min_robot.y = current.y;
					//min_robot.z = current.z;
					//max_robot.x = current.x;
					//max_robot.y = current.y;
					//max_robot.z = current.z;
					found_robot = true;
				}
				else{
					min_robot.x = fmin(current.x,min_robot.x);
					min_robot.y = fmin(current.y,min_robot.y);
					min_robot.z = fmin(current.z,min_robot.z);
					max_robot.x = fmax(current.x,max_robot.x);
					max_robot.y = fmax(current.y,max_robot.y);
					max_robot.z = fmax(current.z,max_robot.z);
				}
				robot.push_back(current);
				i_robot++;
			}
		}
	}
	//     ^
	//   x |
	//     |  y
	//     @---->
	//    /
	//   / z
	//  v
	if(found_robot){
		ROS_INFO("Found robot at position (%f,%f,%f)", (3*max_robot.x-min_robot.x)/2.0f, (3*max_robot.y-min_robot.y)/2.0f, (3*max_robot.z-min_robot.z)/2.0f);
		ROS_INFO("Robot size : (%f,%f,%f)",(max_robot.x-min_robot.x),(max_robot.y-min_robot.y),(max_robot.z-min_robot.z));
	}
}

static uint16_t distance(const geometry_msgs::Point a, const geometry_msgs::Point b){
	return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2) + pow((a.z - b.z),2));
}

void personImageCallback(const clusterisator::Persons::ConstPtr& img_persons){
	sensor_msgs::Image img = img_persons->img;
	h = img.height;
	w = img.width;
	
	uint32_t i,j,k;
	
	if(img_persons->there_is_a_robot){
		collect_robot_data(img_persons);

		bool danger = false;
		for(i=0;i<h && !danger;i++){
			for(j=0;j<w && !danger;j++){
				uint8_t type = img.data[i*3*w+j*3+2];
				uint16_t depth = (img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3];
				if(type == img_persons->MOVING_PERSON){
					geometry_msgs::Point p = depth_to_cartesian(i,j,depth);
					for(k=0; k<robot.size() && !danger; k++){
						uint16_t dist = distance(p,robot[k]);
						if(dist < SECURITY_DISTANCE){
							ROS_WARN("BAAAAH ATTENTION ! Distance : %d",dist);
							danger = true;
						}
					}
				}
			}
		}
	}
}

};

int main(int argc, char **argv){

	ros::init(argc, argv, "securisator_node");

	Securisator securisator;

	ros::spin();

	return 0;
}
