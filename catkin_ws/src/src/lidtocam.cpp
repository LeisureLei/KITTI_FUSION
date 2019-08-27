#include <Eigen/Core>

#include <cmath>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <cstdlib>
#include <string>


using namespace cv;
using namespace std;

unsigned int currentFrame = 0;

//3x3 rectifying rotation to make image planes co-planar, R_rect_0X:3x3
Eigen::Matrix<double,4,4> R_rect_02;

//3x4 projection matrix after rectification, P_rect_02:3x4						
Eigen::Matrix<double,3,4>  P_rect_02;

//Transform from velo to cam0, T:4x4
Eigen::Matrix<double,4,4> extrinsicT;

Mat	image ;

string getFrameStr(unsigned int frame)
{
	if(frame>9999)
		return "00000"+to_string(frame);
	else if(frame>999)
		return "000000"+to_string(frame);
	else if(frame>99)
		return "0000000"+to_string(frame);
	else if(frame>9)
		return "00000000"+to_string(frame);
	else if(frame<=9)
		return "000000000"+to_string(frame);
}

//把激光点云坐标投影到相机二维平面
Eigen::Vector3d transformProject(const Eigen::Vector4d& P_lidar)
{	Eigen::Vector3d z_P_uv = P_rect_02*R_rect_02*extrinsicT*P_lidar;
	return Eigen::Vector3d( int( z_P_uv[0]/z_P_uv[2] + 0.5 ) , int( z_P_uv[1]/z_P_uv[2] + 0.5 ), 1 );
}


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &PointCloudMsg)
{	
	std::string image_name = "/home/leibing/Downloads/2011_09_26_drive_0009_sync/image_02/data/" + getFrameStr(currentFrame) + ".png";
	cout << "currentFrame:"<< currentFrame << endl;
	image = imread(image_name);//原图
	Mat image_show = image.clone();

	//将点云转为pcl格式
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	pcl::fromROSMsg(*PointCloudMsg, laserCloudIn);

	int pointcloudsize = laserCloudIn.size(); //点云个数
	cout<<"received pointcloud feature size per sweep = "<<pointcloudsize<<endl;

	vector<Point2f> point; 

	int count = pointcloudsize;

	for(int i = 0; i < pointcloudsize; i++){
		
		Point2f cv_point;
		Eigen::Vector4d P_lidar(laserCloudIn.points[i].x,
										laserCloudIn.points[i].y,
											laserCloudIn.points[i].z,
												1);
		
		Eigen::Vector3d P_uv = transformProject(P_lidar);
		
		//去除不在图像上的投影点,并把点转为cv::Point2f类型
		if(P_uv[0] >= 0 && P_uv[1] >= 0 && P_uv[0]<=1242 && P_uv[1]<=375){
					
			cv_point.x = P_uv[0];
			cv_point.y = P_uv[1];
			circle(image_show,cv_point,1,Scalar(0,255,0));
			point.push_back(cv_point);
	
		}		
		else{
			count--;
		}			
	}
	int num = point.size();
	
	cout<<"pointcloudsize after verify = "<<num<<endl<<endl;
	

	imshow("fusion" , image_show);
	waitKey(10);

	currentFrame++;	
}

int main( int argc, char** argv)
{	
	ros::init(argc, argv, "lidtocam");   
	ros::NodeHandle nh;

	R_rect_02 <<  9.999758e-01, -5.267463e-03, -4.552439e-03, 0,
				5.251945e-03, 9.999804e-01, -3.413835e-03, 0,
				 4.570332e-03, 3.389843e-03, 9.999838e-01, 0,
				 0  ,0  ,  0,  1;

	P_rect_02 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
				 0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
				  0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

	extrinsicT << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
			1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
			9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
			0 ,	0,	0,	1;
	
	ros::Subscriber pointCloudSub = nh.subscribe("/velodyne_cloud", 10 ,pointCloudCallback );
	ros::spin();
	
	return 0;
}
