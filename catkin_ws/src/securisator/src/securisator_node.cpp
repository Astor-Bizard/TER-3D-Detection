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

struct robot_t{
	geometry_msgs::Point pt_min_x;
	geometry_msgs::Point pt_min_y;
	geometry_msgs::Point pt_min_z;
	geometry_msgs::Point pt_max_x;
	geometry_msgs::Point pt_max_y;
	geometry_msgs::Point pt_max_z;
};

struct robot_t robot;

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
	
	uint8_t data_type[h*w];
	uint16_t data_depth[h*w];
	
	for(i=0;i<h;i++){
		for(j=0;j<w;j++){
			data_type[i*w+j] = img.data[i*3*w+j*3+2];
			data_depth[i*w+j] = (img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3];
		}
	}
	
	geometry_msgs::Point cartesian_map[h*w];
	uint32_t index_min_x, index_min_y, index_min_z, index_max_x, index_max_y, index_max_z;
	
	for(i=0;i<h;i++){
		for(j=0;j<w;j++){
			uint8_t type = data_type[i*w+j];
			uint16_t depth = data_depth[i*w+j];
			if(type == img_persons->ROBOT){
				geometry_msgs::Point current = depth_to_cartesian(i,j,depth);
				cartesian_map[i*w+j] = current;
				if(!found_robot){
					index_min_x = index_max_x =
					index_min_y = index_max_y =
					index_min_z = index_max_z =
												i*w+j;
					found_robot = true;
				}
				else{
					#define MAJ_INDEX(INDEX,XYZ,CMP) INDEX = cartesian_map[INDEX].XYZ CMP current.XYZ ? INDEX : i*w+j;
					MAJ_INDEX(index_min_x,x,<);
					MAJ_INDEX(index_min_y,y,<);
					MAJ_INDEX(index_min_z,z,<);
					MAJ_INDEX(index_max_x,x,>);
					MAJ_INDEX(index_max_y,y,>);
					MAJ_INDEX(index_max_z,z,>);
				}
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
	
	robot.pt_min_x = cartesian_map[index_min_x];
	robot.pt_min_y = cartesian_map[index_min_y];
	robot.pt_min_z = cartesian_map[index_min_z];
	robot.pt_max_x = cartesian_map[index_max_x];
	robot.pt_max_y = cartesian_map[index_max_y];
	robot.pt_max_z = cartesian_map[index_max_z];
	
	if(found_robot){
		ROS_INFO("Found robot at position (%f,%f,%f)", robot.pt_min_x.x + (robot.pt_max_x.x-robot.pt_min_x.x)/2.0f, robot.pt_min_y.y + (robot.pt_max_y.y-robot.pt_min_y.y)/2.0f, robot.pt_min_z.z + (robot.pt_max_z.z-robot.pt_min_z.z)/2.0f);
		ROS_INFO("Robot size : (%f,%f,%f)",(robot.pt_max_x.x-robot.pt_min_x.x),(robot.pt_max_y.y-robot.pt_min_y.y),(robot.pt_max_z.z-robot.pt_min_z.z));
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
					uint16_t dist;
					#define CHECK_DISTANCE_WITH_P(PT)	dist = distance(p,PT);\
														if(dist < SECURITY_DISTANCE){\
															ROS_WARN("BAAAAH ATTENTION ! Distance : %d",dist);\
															danger = true;\
														}
					CHECK_DISTANCE_WITH_P(robot.pt_min_x);
					CHECK_DISTANCE_WITH_P(robot.pt_min_y);
					CHECK_DISTANCE_WITH_P(robot.pt_min_z);
					CHECK_DISTANCE_WITH_P(robot.pt_max_x);
					CHECK_DISTANCE_WITH_P(robot.pt_max_y);
					CHECK_DISTANCE_WITH_P(robot.pt_max_z);
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
