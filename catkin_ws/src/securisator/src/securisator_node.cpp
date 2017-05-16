#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"

#define PERSON_IMG_TOPIC "/clusterisator/person"
#define CAMERA_INFO_TOPIC "/depth/camera_info"

class Securisator {

private:
ros::Subscriber person_img_sub;
ros::Subscriber camera_info_sub;

uint32_t h,w;

double f;
double cx,cy;

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
}

void personImageCallback(const sensor_msgs::Image::ConstPtr& img){
	h = img->height;
	w = img->width;
	
	uint32_t i,j;
	
	bool found = false;
	for(i=0;i<h;i++)
		for(j=0;j<w;j++)
			if(img->data[i*3*w+j*3+2] > 0)
				found = true;
	
	if(found)
		ROS_INFO("Found !");
}

};

int main(int argc, char **argv){

	ros::init(argc, argv, "securisator_node");

	Securisator securisator;

	ros::spin();

	return 0;
}
