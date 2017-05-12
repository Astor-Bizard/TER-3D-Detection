#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <sstream>

#define CAMERA_DEPTH_TOPIC "/depth/image_raw"

#define CLUSTERS_TOPIC "/clusterisator/clusters"
#define CLUSTERS_FRAME_ID "/clusterisator_clusters_frame"

#define IMAGE_OPTI_TOPIC "/clusterisator/image_opti"
#define IMAGE_OPTI_FRAME_ID "/clusterisator_image_opti_frame"

#define PERSON_TOPIC "/clusterisator/person"
#define PERSON_FRAME_ID "/clusterisator_person_frame"

#define CLUSTER_THRESHOLD 100
#define MAX_CLUSTERS_NB 65535

#define DO_OPTI true
#define T_OPTI 5 // taille du carré d'optimisation

uint32_t max2(uint32_t a, uint32_t b){
	return a>b ? a : b;
}
uint32_t max3(uint32_t a, uint32_t b, uint32_t c){
	return max2(max2(a,b),c);
}
uint32_t min2(uint32_t a, uint32_t b){
	return a<b ? a : b;
}
uint32_t min3(uint32_t a, uint32_t b, uint32_t c){
	return min2(min2(a,b),c);
}


class Clusterisator {

private:
ros::Subscriber img_sub;

ros::Publisher clusters_publisher;
#if DO_OPTI
ros::Publisher image_opti_publisher;
#endif
ros::Publisher person_publisher;

std::vector<uint16_t> background;

bool first_pass;

struct cluster_t{
	int size;
	int min_i;
	int max_i;
	int min_j;
	int max_j;
};
struct cluster_t clusters[MAX_CLUSTERS_NB];

/*

Format des données recues par openni_node :
uint8_t data[size], avec size = h*step = h*w*2

Interpretation : données sur 16bits, donc sur 2 cases du tableau

[[(bits 8->15)] [(bits 0->7)]]

-> uint16_t depth[i,j] = (data[i*w*2 + j*2 + 1] << 8) + data[i*w*2 + j*2];

*/


public:

Clusterisator(){

	ros::NodeHandle nh;

	clusters_publisher = nh.advertise<sensor_msgs::Image>(CLUSTERS_TOPIC, 5);
#if DO_OPTI
	image_opti_publisher = nh.advertise<sensor_msgs::Image>(IMAGE_OPTI_TOPIC, 5);
#endif
	person_publisher = nh.advertise<sensor_msgs::Image>(PERSON_TOPIC, 5);

	img_sub = nh.subscribe(CAMERA_DEPTH_TOPIC, 5, &Clusterisator::depthImageCallback, this);
	
	first_pass = true;
}


#if DO_OPTI
// Publish the computed optimized data
void publish_data_opti(uint16_t data[], uint32_t h, uint32_t w, const sensor_msgs::Image::ConstPtr& img){
	sensor_msgs::Image opti_img;
	opti_img.header.stamp = ros::Time::now();
	opti_img.header.frame_id = IMAGE_OPTI_FRAME_ID;
	opti_img.height = h;
	opti_img.width = w;
	opti_img.encoding = img->encoding;
	opti_img.is_bigendian = img->is_bigendian;
	opti_img.step = 2*w;
	
	for(uint32_t i=0; i<h; i++){
		for(uint32_t j=0; j<w; j++){
			opti_img.data.push_back((uint8_t)(data[i*w+j] & 255));
			opti_img.data.push_back((uint8_t)(data[i*w+j] >> 8));
		}
	}
	
	image_opti_publisher.publish(opti_img);
}
#endif


// Compute and publish persons detected
void publish_person(uint16_t cluster_num[], uint32_t h, uint32_t w, uint16_t n,
					struct cluster_t clusters[]){
	sensor_msgs::Image person;
	person.header.stamp = ros::Time::now();
	person.header.frame_id = PERSON_FRAME_ID;
	person.height = h;
	person.width = w;
	person.encoding = "rgb8";
	person.is_bigendian = false;
	person.step = 3*w;
	
	int nb_person=0;
	uint8_t cluster_person[n]; // 1 if it is a person
	uint32_t i,j;
	
	for(i=0;i<n;i++)
		cluster_person[i]=0;
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			uint16_t c_n = cluster_num[i*w+j];
			int c_height = clusters[c_n].max_i - clusters[c_n].min_i + 1;
			int c_width = clusters[c_n].max_j - clusters[c_n].min_j + 1;
			if(c_width > 10 && c_width < 50 && c_height > 60){
				if(cluster_person[c_n] == 0){
					cluster_person[c_n] = 1;
					nb_person++;
				}
				person.data.push_back((uint8_t)255);
				person.data.push_back((uint8_t)0);
				person.data.push_back((uint8_t)0);
			}
			else{
				person.data.push_back((uint8_t)0);
				person.data.push_back((uint8_t)255);
				person.data.push_back((uint8_t)0);
			}
		}
	}
	person_publisher.publish(person);
}


void compute_clusterisation(const sensor_msgs::Image::ConstPtr& img){

	uint32_t i,j;
	
#if DO_OPTI
	static const uint32_t h = img->height/T_OPTI;
	static const uint32_t w = img->width/T_OPTI;
	// Pretraitement d'optimisation : reduction de la taille de data
	uint16_t data[h*w];
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			uint32_t x,y;
			x=i*T_OPTI;
			y=j*T_OPTI;
			data[i*w+j] = (img->data[x*img->width*2 + y*2 + 1] << 8) + img->data[x*img->width*2 + y*2];
		}
	}
	
	publish_data_opti(data,h,w,img);
#else
	static const uint32_t h = img->height;
	static const uint32_t w = img->width;
	uint16_t data[h*w];
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			data[i*w+j] = (img->data[i*w*2 + j*2 + 1] << 8) + img->data[i*w*2 + j*2];
		}
	}
#endif
	
	if(first_pass){
		first_pass = false;
		for(i=0; i<h; i++)
			for(j=0; j<w; j++)
				background.push_back(data[i*w+j]);
				
		/*
		// Suppression du bruit dans le background
		for(i=1;i<h-1;i++)
			for(j=1;j<w-1;j++)
				// Si le pixel est différent de tous ses voisins, c'est du bruit
				if(	  abs(background[i*w+j] - background[i*w+j-1]) < CLUSTER_THRESHOLD
				   && abs(background[i*w+j] - background[i*w+j+1]) < CLUSTER_THRESHOLD
				   && abs(background[i*w+j] - background[(i-1)*w+j]) < CLUSTER_THRESHOLD
				   && abs(background[i*w+j] - background[(i+1)*w+j]) < CLUSTER_THRESHOLD){

				 		background[i*w+j] = background[i*w+j-1];
				}
		*/
	}
	
	#define NEW_CLUSTER(INDEX_I,INDEX_J) n++;\
										 nb_clusters++;\
										 cluster_num[INDEX_I*w+INDEX_J] = n;\
										 clusters[n].size = 1;\
										 clusters[n].min_i = INDEX_I;\
										 clusters[n].max_i = INDEX_I;\
										 clusters[n].min_j = INDEX_J;\
										 clusters[n].max_j = INDEX_J;\
										 
	
	uint16_t cluster_num[h*w];
	
	uint16_t x,y;
	uint16_t nb_clusters=0;
	uint16_t n = 0;
	NEW_CLUSTER(0,0);
	for(i=1; i<h; i++){
		if(abs(data[i*w] - data[(i-1)*w]) < CLUSTER_THRESHOLD){
			x = cluster_num[(i-1)*w];
			cluster_num[i*w] = x;
			clusters[x].size ++;
			if(clusters[x].max_i < i)
				clusters[x].max_i = i;
		}
		else{
			NEW_CLUSTER(i,0);
		}
	}
	for(j=1; j<w; j++){
		if(abs(data[j] - data[j-1]) < CLUSTER_THRESHOLD){
			y = cluster_num[j-1];
			cluster_num[j] = y;
			clusters[y].size ++;
			if(clusters[y].max_j < j)
				clusters[y].max_j = j;
		}
		else{
			NEW_CLUSTER(0,j);
		}
	}
	for(i=1; i<h; i++){
		for(j=1; j<w; j++){
			bool next_to_top  = abs(data[i*w+j] - data[(i-1)*w+j]) < CLUSTER_THRESHOLD;
			bool next_to_left = abs(data[i*w+j] - data[i*w+j-1]) < CLUSTER_THRESHOLD;
			x = cluster_num[(i-1)*w+j];
			y = cluster_num[i*w+j-1];
			if(next_to_top){
				if(next_to_left){
					// top & left
					if(x != y){
						uint16_t small,big;
						if(clusters[x].size < clusters[y].size){
							small = x;
							big = y;
						}
						else{
							small = y;
							big = x;
						}
						//TODO optimiser la substitution
						//ROS_INFO("Optimise-moi");
						for(uint32_t i2=0;i2<=h;i2++)
							for(uint32_t j2=0;j2<w;j2++)
								if(cluster_num[i2*w+j2]==small)
									cluster_num[i2*w+j2]=big;
						
						cluster_num[i*w+j] = big;
						nb_clusters--;
						clusters[big].size += clusters[small].size + 1;
						clusters[big].min_i = min3(clusters[big].min_i,clusters[small].min_i,i);
						clusters[big].min_j = min3(clusters[big].min_j,clusters[small].min_j,j);
						clusters[big].max_i = max3(clusters[big].max_i,clusters[small].max_i,i);
						clusters[big].max_j = max3(clusters[big].max_j,clusters[small].max_j,j);
					}
					else{ // x=y
						cluster_num[i*w+j] = x;
						clusters[x].size ++;
						if(clusters[x].max_i < i)
							clusters[x].max_i = i;
						if(clusters[y].max_j < j)
							clusters[y].max_j = j;
						
					}
				}
				else{
					// top & !left
					cluster_num[i*w+j] = x;
					clusters[x].size ++;
					if(clusters[x].max_i < i)
						clusters[x].max_i = i;
				}
			}
			else if(next_to_left){
				// !top & left
				cluster_num[i*w+j] = y;
				clusters[y].size ++;
				if(clusters[y].max_j < j)
					clusters[y].max_j = j;
				
			}
			else{
				// !top & !left
				NEW_CLUSTER(i,j);
			}
		}
	}
	
	/*
	// Suppression du bruit
	for(i=1;i<h-1;i++)
		for(j=1;j<w-1;j++)
			// Si c'est un cluster d'1 pixel, il est considéré comme du bruit.
			if(cluster_num[i*w+j] != cluster_num[i*w+j-1] && cluster_num[i*w+j] != cluster_num[i*w+j+1] &&
			   cluster_num[i*w+j] != cluster_num[(i+1)*w+j] && cluster_num[i*w+j] != cluster_num[(i-1)*w+j]){
			 		cluster_num[i*w+j] = cluster_num[i*w+j-1];
					nb_clusters--;
			}
	*/
	
	sensor_msgs::Image clst;
	clst.header.stamp = ros::Time::now();
	clst.header.frame_id = CLUSTERS_FRAME_ID;
	clst.height = h;
	clst.width = w;
	clst.encoding = img->encoding;
	clst.is_bigendian = img->is_bigendian;
	clst.step = 2*w;
	
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			clst.data.push_back((uint8_t)(cluster_num[i*w+j] & 255));
			clst.data.push_back((uint8_t)(cluster_num[i*w+j] >> 8));
		}
	}
	clusters_publisher.publish(clst);
	
	publish_person(cluster_num,h,w,n,clusters);
}


void depthImageCallback(const sensor_msgs::Image::ConstPtr& img){
	compute_clusterisation(img);
}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "clusterisator_node");

	Clusterisator clusterisator;

	ros::spin();

	return 0;
}
