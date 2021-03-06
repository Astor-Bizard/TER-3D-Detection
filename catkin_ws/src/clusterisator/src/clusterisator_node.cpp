#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "perception_msgs/Persons.h"

#include <cmath>
#include <sstream>

#define CAMERA_DEPTH_TOPIC "/depth/image_raw"

#define CLUSTERS_TOPIC "/clusterisator/clusters"
#define CLUSTERS_FRAME_ID "/clusterisator_clusters_frame"

#define IMAGE_OPTI_TOPIC "/clusterisator/image_opti"
#define IMAGE_OPTI_FRAME_ID "/clusterisator_image_opti_frame"

#define BACKGROUND_TOPIC "/clusterisator/background"
#define BACKGROUND_FRAME_ID "/clusterisator_background_frame"

#define PERSON_TOPIC "/clusterisator/person"
#define PERSON_FRAME_ID "/clusterisator_person_frame"

#define RGB_PERSON_TOPIC "/clusterisator/rgb_person"
#define RGB_PERSON_FRAME_ID "/clusterisator_rgb_person_frame"

#define STATIC_THRESHOLD 50	    // the minimum distance (in mm) between moving objects and the background
#define CLUSTER_THRESHOLD 80	// the maximum depth diff between two pixels (in mm) for them to be in the same cluster
#define MAX_CLUSTERS_NB 65535
#define MAX_PERSONS_NB 100

#define MAX_RANGE 10000    // the maximum reliable range of the camera

//#define MAX_DYNAMICNESS 5
//#define DYNAMICNESS_THRESHOLD 3

#define DO_OPTI true
#define T_OPTI 2 // taille du carré d'optimisation

#define CLUSTER_NOISE_SIZE 50

/*
#define FREE 0
#define OBSTACLE 1
#define UKNOWN 2
*/

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
ros::Publisher background_publisher;

ros::Publisher person_publisher;
ros::Publisher rgb_person_publisher;

uint32_t h,w;

std::vector<uint16_t> background;
//std::vector<uint16_t> dynamicness;

bool first_pass;

struct cluster_t{
	uint32_t size;
	uint32_t min_i;
	uint32_t max_i;
	uint32_t min_j;
	uint32_t max_j;
	uint16_t dist;
};
struct cluster_t clusters[MAX_CLUSTERS_NB];

struct person_t{
	uint16_t dist;
	uint32_t width;
	uint32_t height;
};
struct person_t persons[MAX_PERSONS_NB];

/*

Format des données recues par openni_node :
uint8_t data[size], avec size = h*step = h*w*2

Interpretation : données sur 16bits, donc sur 2 cases du tableau

[[(low bits 8->15)] [(high bits 0->7)]]

-> uint16_t depth[i,j] = (data[i*w*2 + j*2 + 1] << 8) + data[i*w*2 + j*2];

*/


public:

Clusterisator(){

	ros::NodeHandle nh;

	clusters_publisher = nh.advertise<sensor_msgs::Image>(CLUSTERS_TOPIC, 5);
#if DO_OPTI
	image_opti_publisher = nh.advertise<sensor_msgs::Image>(IMAGE_OPTI_TOPIC, 5);
#endif
	background_publisher = nh.advertise<sensor_msgs::Image>(BACKGROUND_TOPIC, 5);
	
	person_publisher = nh.advertise<perception_msgs::Persons>(PERSON_TOPIC, 5);
	rgb_person_publisher = nh.advertise<sensor_msgs::Image>(RGB_PERSON_TOPIC, 5);

	img_sub = nh.subscribe(CAMERA_DEPTH_TOPIC, 1, &Clusterisator::depthImageCallback, this);
	
	first_pass = true;
}


#if DO_OPTI
// Publish the computed optimized data
void publish_data_opti(const uint16_t data[], const sensor_msgs::Image::ConstPtr& img){
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


// Publish the background
void publish_background(const sensor_msgs::Image::ConstPtr& img){

	sensor_msgs::Image background_img;
	background_img.header.stamp = ros::Time::now();
	background_img.header.frame_id = BACKGROUND_FRAME_ID;
	background_img.height = h;
	background_img.width = w;
	background_img.encoding = img->encoding;
	background_img.is_bigendian = img->is_bigendian;
	background_img.step = 2*w;
	
	for(uint32_t i=0; i<h; i++){
		for(uint32_t j=0; j<w; j++){
			background_img.data.push_back((uint8_t)(background[i*w+j] & 255));
			background_img.data.push_back((uint8_t)(background[i*w+j] >> 8));
		}
	}
	
	background_publisher.publish(background_img);
}

static bool is_a_person(const struct cluster_t c){
	uint16_t dist = c.dist;
	int width = c.max_j - c.min_j + 1;
	int height = c.max_i - c.min_i + 1;

	int opti_coef;
	opti_coef = DO_OPTI ? T_OPTI : 1;
	
	// TODO calculer la taille de l'objet/personne en fonction de la distance à la caméra
	//return (width > 50/opti_coef && width < 250/opti_coef && height > 250/opti_coef && dist > 0);
	return dist>0 && dist<9870 && width > 50/opti_coef && width < 350/opti_coef && height > 180/opti_coef;
}

// Compute and publish persons detected
void publish_person(const uint16_t cluster_num[], const uint16_t n, const struct cluster_t clusters[], const uint16_t data[]){
	
	// Image of data - for each pixel :
	// original depth on 16 bits (in the original format)
	// last 8 bits : 0 if static, 1 if moving, 2 if person, 3 if robot
	perception_msgs::Persons msg_person;
	sensor_msgs::Image img_person;
	img_person.header.stamp = ros::Time::now();
	img_person.header.frame_id = PERSON_FRAME_ID;
	img_person.height = h;
	img_person.width = w;
	img_person.encoding = "rgb8";
	img_person.is_bigendian = false;
	img_person.step = 3*w;

	// Image for rgb visualisation with rviz
	sensor_msgs::Image img_rgb_person;
	img_rgb_person.header.stamp = ros::Time::now();
	img_rgb_person.header.frame_id = RGB_PERSON_FRAME_ID;
	img_rgb_person.height = h;
	img_rgb_person.width = w;
	img_rgb_person.encoding = "rgb8";
	img_rgb_person.is_bigendian = false;
	img_rgb_person.step = 3*w;
	
	int nb_persons=0;
	uint8_t cluster_person[n]; // non 0 if it is a person, index in persons[]
	uint32_t i,j;
	
	uint16_t cluster_robot = 0;
	
	// Init robot
	// TODO recognize the robot
	// here we just chose a cluster at the center of the image
	for(i=(4*h)/10; i<(6*h)/10 && cluster_robot == 0; i++)
		for(j=(4*w)/10; j<(6*w)/10 && cluster_robot == 0; j++)
			if(cluster_num[i*w+j] != 0 && ! is_a_person(clusters[cluster_num[i*w+j]]))
				cluster_robot = cluster_num[i*w+j];
	
	// Init persons
	for(i=0;i<n;i++)
		cluster_person[i]=0;
	
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			img_person.data.push_back((uint8_t)(data[i*w+j] & 255));
			img_person.data.push_back((uint8_t)(data[i*w+j] >> 8));
		
			uint16_t c_n = cluster_num[i*w+j];
			if(c_n != 0){
				if(is_a_person(clusters[c_n])){
					if(cluster_person[c_n] == 0){
						nb_persons++;
						cluster_person[c_n] = nb_persons;
						persons[nb_persons].dist = clusters[c_n].dist;
						persons[nb_persons].width = clusters[c_n].max_j - clusters[c_n].min_j + 1;
						persons[nb_persons].height = clusters[c_n].max_i - clusters[c_n].min_i + 1;
					}
					img_person.data.push_back(msg_person.MOVING_PERSON);
					// Person : red
					img_rgb_person.data.push_back((uint8_t)255);
					img_rgb_person.data.push_back((uint8_t)0);
					img_rgb_person.data.push_back((uint8_t)0);
				}
				else{
					if(cluster_robot == c_n){
						// A moving object in the center of the view : it's the robot
						img_person.data.push_back(msg_person.ROBOT);
						// Robot : yellow
						img_rgb_person.data.push_back((uint8_t)255);
						img_rgb_person.data.push_back((uint8_t)255);
						img_rgb_person.data.push_back((uint8_t)0);
					}
					else{
						img_person.data.push_back(msg_person.MOVING_OBJECT);
						// Not a person : green
						img_rgb_person.data.push_back((uint8_t)0);
						img_rgb_person.data.push_back((uint8_t)255);
						img_rgb_person.data.push_back((uint8_t)0);
					}
				}
			}
			else{
				img_person.data.push_back(msg_person.STATIC_OBJECT);
				// Background : blue
				img_rgb_person.data.push_back((uint8_t)0);
				img_rgb_person.data.push_back((uint8_t)0);
				img_rgb_person.data.push_back((uint8_t)255);
			}
		}
	}
	//if(nb_persons > 0)
	//	ROS_INFO("Found %d person%s !",nb_persons,nb_persons>1?"s":"");
	//for(i=1;i<=nb_persons;i++){
	//	ROS_INFO("Person %d at dist %d : width %d and height %d",i,persons[i].dist,persons[i].width,persons[i].height);
	//}
	msg_person.there_is_a_robot = (cluster_robot != 0);
	msg_person.nb_persons = nb_persons;
	msg_person.resolution_factor = DO_OPTI ? T_OPTI : 1;
	msg_person.img = img_person;
	person_publisher.publish(msg_person);
	rgb_person_publisher.publish(img_rgb_person);
}


	
void replace_cluster(uint16_t *cluster_num, const uint16_t old_c, const uint16_t new_c, const uint32_t i, const uint32_t j){
	if(cluster_num[i*w+j] == old_c){
		cluster_num[i*w+j] = new_c;
		if(i>0)
			replace_cluster(cluster_num,old_c,new_c,i-1,j);
		if(j>0)
			replace_cluster(cluster_num,old_c,new_c,i,j-1);
		if(i<h-1)
			replace_cluster(cluster_num,old_c,new_c,i+1,j);
		if(j<w-1)
			replace_cluster(cluster_num,old_c,new_c,i,j+1);
	}
}


void compute_clusterisation(const sensor_msgs::Image::ConstPtr& img){

	uint32_t i,j;
	
#if DO_OPTI
	h = img->height/T_OPTI;
	w = img->width/T_OPTI;
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
	
	publish_data_opti(data,img);
#else
	h = img->height;
	w = img->width;
	uint16_t data[h*w];
	for(i=0; i<h; i++){
		for(j=0; j<w; j++){
			data[i*w+j] = (img->data[i*w*2 + j*2 + 1] << 8) + img->data[i*w*2 + j*2];
		}
	}
#endif

	uint16_t cluster_num[h*w];
	//uint8_t grid[MAX_RANGE][h][w]; // Size problem here (4000x320x240 -> 307Mo)
	
	if(first_pass){
		first_pass = false;

		// Suppression du bruit
		for(i=1;i<h-1;i++){
			for(j=1;j<w-1;j++){
				// Si le pixel est à 0, c'est du bruit
				// On le remplace par le pixel non-0 le plus proche
				if(data[i*w+j] == 0){
					if(data[(i-1)*w+j] != 0)
				 		data[i*w+j] = data[(i-1)*w+j];
				 	else if(data[i*w+j-1] != 0)
				 		data[i*w+j] = data[i*w+j-1];
				 	else if(data[(i+1)*w+j] != 0)
				 		data[i*w+j] = data[(i+1)*w+j];
				 	else if(data[i*w+j+1] != 0)
				 		data[i*w+j] = data[i*w+j+1];
				}
			}
		}
		
		for(i=0; i<h; i++){
			for(j=0; j<w; j++){
				background.push_back(data[i*w+j]);
				//dynamicness.push_back(1);
			}
		}
	}
	else{
		/*
		// Explicit depth grid
		uint16_t k;
		for(i=0; i<h; i++){
			for(j=0; j<w; j++){
				
				if(data[i*w+j]<MAX_RANGE){
					for(k=0; k<data[i*w+j]; k++)
						grid[k][i][j] = FREE;
					
					grid[data[i*w+j]][i][j] = OBSTACLE;
				
					for(k=data[i*w+j]+1; k<MAX_RANGE; k++)
						grid[k][i][j] = UKNOWN;
				}
				else{
					for(k=0; k<MAX_RANGE; k++)
						grid[k][i][j] = FREE;
				}
			}
		}*/
		// The grid is indexed plane by plane, facing the camera
		// We could do model recognition plane by plane
	
		#define NEW_CLUSTER(INDEX_I,INDEX_J) nb_clusters++;\
											 cluster_num[INDEX_I*w+INDEX_J] = n;\
											 clusters[n].size = 1;\
											 clusters[n].min_i = INDEX_I;\
											 clusters[n].max_i = INDEX_I;\
											 clusters[n].min_j = INDEX_J;\
											 clusters[n].max_j = INDEX_J;\
											 clusters[n].dist = data[INDEX_I*w+INDEX_J];\
											 n++;
	
		#define IS_IN_BACKGROUND(INDEX) (abs(data[INDEX] - background[INDEX]) < STATIC_THRESHOLD || data[INDEX]==0 || data[INDEX]>MAX_RANGE)
		
		uint16_t x,y;
		uint16_t nb_clusters=0;
		uint16_t n = 1;	// start at 1, 0 is for static points (equals to background)
		if(IS_IN_BACKGROUND(0))
			cluster_num[0] = 0;
		else
			NEW_CLUSTER(0,0);

		for(i=1; i<h; i++){
			if(IS_IN_BACKGROUND(i*w)){
				cluster_num[i*w] = 0;
				//dynamicness[i*w] = max2(1,dynamicness[i*w]-1);
			}
			else{
				//dynamicness[i*w] = min2(MAX_DYNAMICNESS,dynamicness[i*w]+1);
				if(abs(data[i*w] - data[(i-1)*w]) < CLUSTER_THRESHOLD && cluster_num[(i-1)*w] != 0){
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
		}
		for(j=1; j<w; j++){
			if(IS_IN_BACKGROUND(j)){
				cluster_num[j] = 0;
				//dynamicness[j] = max2(1,dynamicness[j]-1);
			}
			else{
				//dynamicness[j] = min2(MAX_DYNAMICNESS,dynamicness[j]+1);
				if(abs(data[j] - data[j-1]) < CLUSTER_THRESHOLD && cluster_num[j-1] != 0){
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
		}
		for(i=1; i<h; i++){
			for(j=1; j<w; j++){
				if(IS_IN_BACKGROUND(i*w+j)){
					cluster_num[i*w+j] = 0;
					//dynamicness[i*w+j] = max2(1,dynamicness[i*w+j]-1);
				}
				else{
					//dynamicness[i*w+j] = min2(MAX_DYNAMICNESS,dynamicness[i*w+j]+1);
					bool next_to_top  = abs(data[i*w+j] - data[(i-1)*w+j]) < CLUSTER_THRESHOLD && cluster_num[(i-1)*w+j] != 0;
					bool next_to_left = abs(data[i*w+j] - data[i*w+j-1]) < CLUSTER_THRESHOLD && cluster_num[i*w+j-1] != 0;
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
							
								// On fusionne les clusters (le petit dans le grand)
								cluster_num[i*w+j] = small;
								replace_cluster(cluster_num,small,big,i,j);
						
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
		}
		/*
		// Suppression du bruit
		for(i=0;i<h;i++){
			for(j=0;j<w;j++){
				// Si le pixel n'a pas de dynamicité, c'est du bruit
				if(dynamicness[i*w+j] <= DYNAMICNESS_THRESHOLD){
				 	clusters[cluster_num[i*w+j]].size --;
				 	if(clusters[cluster_num[i*w+j]].size == 0)
						nb_clusters--;
				 	cluster_num[i*w+j] = 0;
				}
			}
		}
		*/
		for(i=0;i<h;i++){
			for(j=0;j<w;j++){
				// Si c'est un petit cluster, il est considéré comme du bruit.
				if(clusters[cluster_num[i*w+j]].size < CLUSTER_NOISE_SIZE){
				 	clusters[cluster_num[i*w+j]].size --;
				 	if(clusters[cluster_num[i*w+j]].size == 0)
						nb_clusters--;
				 	cluster_num[i*w+j] = 0;
				}
			}
		}
	
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
	
		publish_person(cluster_num,n,clusters,data);
	
	}
	
	publish_background(img);
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
