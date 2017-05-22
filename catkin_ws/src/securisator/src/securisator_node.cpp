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

#define I_MIN_X 0
#define I_MIN_Y 1
#define I_MIN_Z 2
#define I_MAX_X 3
#define I_MAX_Y 4
#define I_MAX_Z 5
geometry_msgs::Point robot[6];

uint8_t CODE_STATIC_OBJECT;
uint8_t CODE_MOVING_OBJECT;
uint8_t CODE_MOVING_PERSON;
uint8_t CODE_ROBOT;

std::vector<uint8_t> data_type;
std::vector<uint16_t> data_depth;

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

void collect_robot_data(){

	uint32_t i,j;
	
	geometry_msgs::Point cartesian_map[h*w];	
	uint32_t index_in_cartesian_map[6];
	
	bool found_robot = false;
	for(i=0;i<h;i++){
		for(j=0;j<w;j++){
			uint8_t type = data_type[i*w+j];
			uint16_t depth = data_depth[i*w+j];
			if(type == CODE_ROBOT){
				geometry_msgs::Point current = depth_to_cartesian(i,j,depth);
				cartesian_map[i*w+j] = current;
				if(!found_robot){
					for(short int k=0;k<6;k++)
						index_in_cartesian_map[k] = i*w+j;
					
					found_robot = true;
				}
				else{
					#define MAJ_INDEX(INDEX,XYZ,CMP) index_in_cartesian_map[INDEX] = cartesian_map[index_in_cartesian_map[INDEX]].XYZ CMP current.XYZ ? index_in_cartesian_map[INDEX] : i*w+j;
					MAJ_INDEX(I_MIN_X,x,<);
					MAJ_INDEX(I_MIN_Y,y,<);
					MAJ_INDEX(I_MIN_Z,z,<);
					MAJ_INDEX(I_MAX_X,x,>);
					MAJ_INDEX(I_MAX_Y,y,>);
					MAJ_INDEX(I_MAX_Z,z,>);
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
	
	for(short int k=0;k<6;k++)
		robot[k] = cartesian_map[index_in_cartesian_map[k]];
	
	if(found_robot){
		ROS_INFO("Found robot at position (%f,%f,%f)", robot[I_MIN_X].x + (robot[I_MAX_X].x-robot[I_MIN_X].x)/2.0f, robot[I_MIN_Y].y + (robot[I_MAX_Y].y-robot[I_MIN_Y].y)/2.0f, robot[I_MIN_Z].z + (robot[I_MAX_Z].z-robot[I_MIN_Z].z)/2.0f);
		ROS_INFO("Robot size : (%f,%f,%f)",(robot[I_MAX_X].x-robot[I_MIN_X].x),(robot[I_MAX_Y].y-robot[I_MIN_Y].y),(robot[I_MAX_Z].z-robot[I_MIN_Z].z));
	}
}

static uint16_t distance(const geometry_msgs::Point a, const geometry_msgs::Point b){
	return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2) + pow((a.z - b.z),2));
}

void check_security_distance(){
	
	uint32_t i,j;
	
	bool danger = false;
	for(i=0;i<h && !danger;i++){
		for(j=0;j<w && !danger;j++){
			uint8_t type = data_type[i*w+j];
			uint16_t depth = data_depth[i*w+j];
			if(type == CODE_MOVING_PERSON){
				geometry_msgs::Point p = depth_to_cartesian(i,j,depth);
				uint16_t dist;
				for(short int k=0; k<6 && !danger; k++){
					dist = distance(p,robot[k]);
					if(dist < SECURITY_DISTANCE){
						ROS_WARN("BAAAAH ATTENTION ! Distance : %d",dist);
						danger = true;
					}
				}
			}
		}
	}
}

void personImageCallback(const clusterisator::Persons::ConstPtr& img_persons){
	
	const sensor_msgs::Image img = img_persons->img;
	
	recieved_data_pub.publish(img);
	
	h = img.height;
	w = img.width;
	
	resolution_factor = img_persons->resolution_factor;
	
	CODE_STATIC_OBJECT = img_persons->STATIC_OBJECT;
	CODE_MOVING_OBJECT = img_persons->MOVING_OBJECT;
	CODE_MOVING_PERSON = img_persons->MOVING_PERSON;
	CODE_ROBOT = img_persons->ROBOT;
	
	if(img_persons->there_is_a_robot){
	
		uint32_t i,j;
		for(i=0;i<h;i++){
			for(j=0;j<w;j++){
				data_type.push_back(img.data[i*3*w+j*3+2]);
				data_depth.push_back((img.data[i*3*w + j*3 + 1] << 8) + img.data[i*3*w + j*3]);
			}
		}
	
		collect_robot_data();

		check_security_distance();
		
	}
}

};

int main(int argc, char **argv){

	ros::init(argc, argv, "securisator_node");

	Securisator securisator;

	ros::spin();

	return 0;
}
