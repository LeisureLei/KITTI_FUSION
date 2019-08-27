#include <cmath>
#include <vector>

#include <opencv/cv.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>

#include <signal.h>

ros::Publisher pubLaserCloud;


void laserCloudHandler(pcl::PointCloud<pcl::PointXYZ> laserCloudIn)
{

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  int cloudSize = laserCloudIn.points.size();

  sensor_msgs::PointCloud2 cornerPointsMsg;
  pcl::toROSMsg(laserCloudIn, cornerPointsMsg);

  pubLaserCloud.publish(cornerPointsMsg);
  
}

std::string getFrameStr(unsigned int frame)
{
	if(frame>9999)
		return "00000"+std::to_string(frame);
	else if(frame>999)
		return "000000"+std::to_string(frame);
	else if(frame>99)
		return "0000000"+std::to_string(frame);
	else if(frame>9)
		return "00000000"+std::to_string(frame);
	else if(frame<=9)
		return "000000000"+std::to_string(frame);
}

void my_handler(int s){
    std::cout<<"Finishing program with signal "<<s<<std::endl;
    exit(1); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserhandle");
  ros::NodeHandle nh;

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/velodyne_cloud", 10);

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float)); //equal to float *data = float[num];分配内存空间

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  unsigned int currentFrame = 0;
  
  std::string file = "/Downloads/2011_09_26_drive_0009_sync/velodyne_points/data/" + getFrameStr(currentFrame) + ".bin";
  
  FILE *stream;
  stream = fopen (file.c_str(),"rb");
  //std::cout<<file<<std::endl;

  ros::Rate r(10);

  while(stream!=NULL)
  {
	  num = fread(data,sizeof(float),num,stream)/4; 

	  cloud.clear();

	  for (int32_t i=0; i<num; i++) {
	    pcl::PointXYZ point;
	    point.x = *px;
	    point.y = *py;
	    point.z = *pz;
	
	    cloud.push_back(point);
	    px+=4; py+=4; pz+=4; pr+=4;
	  }

	  laserCloudHandler(cloud);

	  //reset variables to read a new sweep
	  fclose(stream);
	  currentFrame++;
	  file = "/Downloads/2011_09_26_drive_0009_sync/velodyne_points/data/" + getFrameStr(currentFrame) + ".bin";
	  //std::cout<<file<<std::endl;
	  fflush(stream);
	  stream = fopen (file.c_str(),"rb");
	  free(data);
	  num = 1000000;
	  data = (float*)malloc(num*sizeof(float));
	  px = data+0;
	  py = data+1;
    pz = data+2;
    pr = data+3;

    r.sleep();
  }

  free(data);

  return 0;
}
