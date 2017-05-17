#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"

#include "../../../devel/include/clusterisator/Persons.h"

#include <math.h>

#define PERSON_IMG_TOPIC "/clusterisator/person"
#define CAMERA_INFO_TOPIC "/depth/camera_info"

#define STATIC_OBJECT 0
#define MOVING_OBJECT 1
#define MOVING_PERSON 2
#define ROBOT         3

class Securisator {

private:
ros::Subscriber person_img_sub;
ros::Subscriber camera_info_sub;

uint32_t h,w;

double f;
double cx,cy;

std::vector<geometry_msgs::Point> robot;

public:

Securisator(){

	ros::NodeHandle nh;

	person_img_sub = nh.subscribe(PERSON_IMG_TOPIC, 1, &Securisator::personImageCallback, this);
	camera_info_sub = nh.subscribe(CAMERA_INFO_TOPIC, 1, &Securisator::cameraInfoCallback, this);

}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info){
	f = camera_info->K[0];
	cx = camera_info->K[2];
	cx = camera_info->K[5];
}

geometry_msgs::Point depth_to_cartesian(int x, int y, uint16_t depth){
	geometry_msgs::Point p;
	p.x = ((x-cx)*depth)/f;
	p.y = ((y-cy)*depth)/f;
	p.z = depth;
	return p;
}

void collect_robot_data(const sensor_msgs::Image img){

	uint32_t i,j;

	bool found_robot = false;
	int i_robot = 0;
	geometry_msgs::Point min_robot,max_robot;
	for(i=0;i<h;i++){
		for(j=0;j<w;j++){
			uint8_t type = img.data[i*3*w+j*3+2];
			uint16_t depth = (img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3];
			if(type == ROBOT){
				robot.push_back(depth_to_cartesian(i,j,depth));
				if(!found_robot){
					min_robot.x = robot[i_robot].x;
					min_robot.y = robot[i_robot].y;
					min_robot.z = robot[i_robot].z;
					max_robot.x = robot[i_robot].x;
					max_robot.y = robot[i_robot].y;
					max_robot.z = robot[i_robot].z;
					found_robot = true;
				}
				else{
					min_robot.x = fmin(robot[i_robot].x,min_robot.x);
					min_robot.y = fmin(robot[i_robot].y,min_robot.y);
					min_robot.z = fmin(robot[i_robot].z,min_robot.z);
					max_robot.x = fmax(robot[i_robot].x,max_robot.x);
					max_robot.y = fmax(robot[i_robot].y,max_robot.y);
					max_robot.z = fmax(robot[i_robot].z,max_robot.z);
				}
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
		ROS_INFO("Found robot of size %d between (%d,%d,%d) and (%d,%d,%d)",i_robot,(int)min_robot.x,(int)min_robot.y,(int)min_robot.z,(int)max_robot.x,(int)max_robot.y,(int)max_robot.z);
		ROS_INFO("Robot size : (%d,%d,%d)",(int)(max_robot.x-min_robot.x),(int)(max_robot.y-min_robot.y),(int)(max_robot.z-min_robot.z));
	}
}

void personImageCallback(const clusterisator::Persons::ConstPtr& img_persons){
	sensor_msgs::Image img = img_persons->img;
	h = img.height;
	w = img.width;
	
	uint32_t i,j;
	
	if(img_persons->there_is_a_robot){
		collect_robot_data(img);
	
		for(i=0;i<h;i++){
			for(j=0;j<w;j++){
				uint8_t type = img.data[i*3*w+j*3+2];
				uint16_t depth = (img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3];
				if(type == MOVING_PERSON){
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
