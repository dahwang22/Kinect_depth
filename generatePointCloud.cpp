#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


const double camera_factor = 1000; 
const double camera_cx = 258.462;
const double camera_cy = 211.267;
const double camera_fx = 367.124;
const double camera_fy = 367.124;


int main( int argc, char** argv )
{
	cv::Mat rgb, depth;
	rgb = cv::imread( "/home/gls/kinect/kinect_depth/rgb.png" );
	depth = cv::imread( "/home/gls/kinect/kinect_depth/depth.png", -1 );

	PointCloud::Ptr cloud ( new PointCloud );
	for (int m = 0; m < depth.rows; m++)
		for (int n=0; n < depth.cols; n++)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if (d == 0)
				continue;
			PointT p;

			p.z = double(d) / camera_factor;
			p.x = (-(n - camera_cx)) * p.z / camera_fx;
			p.y = (-(m - camera_cy)) * p.z / camera_fy;
			
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];

			cloud->points.push_back( p );
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout<<"point cloud size = "<<cloud->points.size()<<endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile( "/home/gls/kinect/kinect_depth/generatepointcloud.pcd", *cloud );
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->addPointCloud(cloud);
	viewer->resetCamera();
	viewer->initCameraParameters();
	viewer->addCoordinateSystem(1.0);
	while (!viewer->wasStopped()){
		viewer->spinOnce();
	}
	// 清除数据并退出
	cloud->points.clear();
	cout<<"Point cloud saved."<<endl;
	return 0;
}
