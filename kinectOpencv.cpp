#include <map>
#include <iostream>
#include <string>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include "Python.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/config.h>

using namespace std;
using namespace cv;

#define CALL_PYTHON


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


const double camera_factor = 1000; 
const double camera_cx = 258.462;
const double camera_cy = 211.267;
const double camera_fx = 367.124;
const double camera_fy = 367.124;

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
	protonect_shutdown = true;
}
//////////////////////////
template<class K, class V>
inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
	V v = defaultValue;
	typename std::map<K, V>::const_iterator i = m.find(key);
	if(i != m.end())
	{
		v = i->second;
	}
	return v;
}
///////////////////////////

int main(int argc, char const *argv[])
{
	std::cout << "Hello World!" << std::endl;

#ifdef CALL_PYTHON	
	
	Py_Initialize();
	
	if ( !Py_IsInitialized() )
	{
		return -1;
	}
	
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./')");
	PyObject *pName,*pModule,*pDict,*pFunc,*pArgs;
	
	pName = PyString_FromString("my_ssd_detect");
	pModule = PyImport_Import(pName);
	if ( !pModule )
	{
		printf("can't find my_ssd_detect.py\n");
		getchar();
		return -1;
	}
	pDict = PyModule_GetDict(pModule);
	if ( !pDict )
	{
		return -1;
	}
#endif

	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = NULL;
	libfreenect2::PacketPipeline  *pipeline = NULL;

	if(freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();

	std::cout << "SERIAL: " << serial << std::endl;

#if 1 // sean
	int depthProcessor = Processor_cl;

	if(depthProcessor == Processor_cpu)
	{
		if(!pipeline)
			//! [pipeline]
			pipeline = new libfreenect2::CpuPacketPipeline();
		//! [pipeline]
	}
	else if (depthProcessor == Processor_gl) // if support gl
	{
		#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if(!pipeline)
		{
			pipeline = new libfreenect2::OpenGLPacketPipeline();
		}
		#else
			std::cout << "OpenGL pipeline is not supported!" << std::endl;
		#endif
	}
	else if (depthProcessor == Processor_cl) // if support cl
	{
		#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if(!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
		#else
			std::cout << "OpenCL pipeline is not supported!" << std::endl;
		#endif
	}

	if(pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}

	if(dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	libfreenect2::SyncMultiFrameListener listener(
			libfreenect2::Frame::Color |
			libfreenect2::Frame::Depth |
			libfreenect2::Frame::Ir);
	libfreenect2::FrameMap frames;
	libfreenect2::Freenect2Device::Config config;
	config.EnableBilateralFilter = true;
	config.EnableEdgeAwareFilter = true;
	config.MinDepth = 0.3f;
	config.MaxDepth = 12.0f;
	dev->setConfiguration(config);

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
	// sean: that is a driver bug
	// check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger

	int r1 = 0, r2 = 0, r3 = 0, r4 = 0;
	cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2, resizergbmat;

	cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);

	
	//VideoWriter video("/home/gls/kinect/capima/out.avi",CV_FOURCC('M','J','P','G'),30,Size(rgbmat.cols,rgbmat.rows),true);
	VideoWriter cvideo("/home/gls/kinect/capima/Color.avi",CV_FOURCC('M','J','P','G'),30,Size(1920,1080),true);

	while(!protonect_shutdown)
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		
		depth = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data).convertTo(depthmat,CV_16U,1);
		cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data).copyTo(resizergbmat);
	
		cv::flip(rgbmat, rgbmat, 1); 
		cv::flip(depthmat, depthmat, 1);
		
		
		cv::imshow("depth", depthmat*20);
		
		int height = rgbmat.rows;  
		int width = rgbmat.cols;  
	  
		 
//		Size dsize = Size(round(0.31 * width), round(0.31 * height)); 
		cvtColor(rgbmat, resizergbmat, CV_BGRA2BGR);
//		cv::resize(resizergbmat, resizergbmat, dsize, INTER_LINEAR); 
		cv::imwrite("/home/gls/kinect/kinect_depth/caffeimage/rgb.jpg", resizergbmat);
		cv::imwrite("/home/gls/kinect/kinect_depth/rgb.png", rgbmat);
		cv::imwrite("/home/gls/kinect/kinect_depth/depth.png",depthmat);
//		cvideo.write(rgbmat);

#ifdef CALL_PYTHON	
		//////call python///////
		pFunc = PyDict_GetItemString(pDict, "detection");
		if ( !pFunc || !PyCallable_Check(pFunc) )
		{
			printf("can't find function [detection]");
			getchar();
			return -1;
		}
		pArgs = Py_BuildValue("(s)","/home/gls/kinect/kinect_depth/caffeimage/rgb.jpg");
		PyObject *pyValue = PyObject_CallObject(pFunc,pArgs);
		PyObject *iterator = PyObject_GetIter(pyValue);
		PyObject *item;

		if (iterator == NULL) {
			cout << "iterator is NULL!" << endl;
		}

		while (item = PyIter_Next(iterator)) {
			/* do something with item */
			PyArg_ParseTuple(item,"i|i|i|i",&r1,&r2,&r3,&r4);//分析返回的元组值  
			if(pyValue)  
			{  
		//		rectangle (rgbmat, Rect(r1*0.31, r2*0.31, r3*0.31, r4*0.31), Scalar(0, 0, 255), 3, 8);
				rectangle (rgbmat, Rect(r1, r2, r3, r4), Scalar(0, 0, 255), 3, 8);
				printf("%d   %d   %d   %d\n",r1,r2,r3,r4);  
			}   
			/* release reference when done */
			Py_DECREF(item);
		}
		cv::imshow("rgb", rgbmat);
		Py_DECREF(iterator);
		/////////////////////////
#endif
	
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
		
		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

		listener.release(frames);
	}
	
	dev->stop();
	dev->close();
	
#ifdef CALL_PYTHON		
	Py_DECREF(pName);
	Py_DECREF(pArgs);
	Py_DECREF(pModule);
	Py_Finalize();
#endif

	delete registration;
#endif
	PointCloud::Ptr cloud ( new PointCloud );

	for (int m = 0; m < depthmat.rows; m++)
		for (int n=0; n < depthmat.cols; n++)
		{
			ushort d = depthmat.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;

			p.z = double(d) / camera_factor;
			p.x = (-(n - camera_cx)) * p.z / camera_fx;
			p.y = (-(m - camera_cy)) * p.z / camera_fy;
			
			p.b = rgbmat.ptr<uchar>(m)[n*3];
			p.g = rgbmat.ptr<uchar>(m)[n*3+1];
			p.r = rgbmat.ptr<uchar>(m)[n*3+2];

			// 把p加入到点云中
			cloud->points.push_back( p );
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout<<"point cloud size = "<<cloud->points.size()<<endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile( "/home/gls/kinect/kinect_depth/pointcloud.pcd", *cloud );
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->addPointCloud(cloud);
	viewer->resetCamera();
	viewer->initCameraParameters();
	viewer->addCoordinateSystem(1.0);
	while (!viewer->wasStopped()){
		viewer->spinOnce();
	}
	cloud->points.clear();
	cout<<"Point cloud saved."<<endl;
	return 0;
}
