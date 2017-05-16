#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#define PERSON_IMG_TOPIC "/clusterisator/person"

class Securisator {

private:
ros::Subscriber person_img_sub;

uint32_t h,w;

public:

Securisator(){

	ros::NodeHandle nh;

	person_img_sub = nh.subscribe(PERSON_IMG_TOPIC, 1, &Securisator::personImageCallback, this);

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
