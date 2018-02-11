#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <thread>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	Mat bgr(1080, 1920, CV_8UC4);
	bgr = imread("/home/gls/kinect/kinect_depth/rgb.png");
	Mat depth(424, 512, CV_16UC1);
	depth = imread("/home/gls/kinect/kinect_depth/depth.png", -1); 
	// 3. 显示
//	thread th = std::thread([&]{
//		while (true)
//		{
//			imshow("原始彩色图", bgr);
//			waitKey(1);
//			imshow("原始深度图", depth*20);
//			waitKey(1);
//		}
//	});
	
	imshow("color_image", bgr);
	imshow("depth_image", depth*20);
	waitKey(0);

	Eigen::Matrix3f K_ir;           // ir内参矩阵
	K_ir <<
		368.8057, 0, 255.5000,
		0, 369.5268, 211.5000,
		0, 0, 1;
	Eigen::Matrix3f K_rgb;          // rgb内参矩阵
	K_rgb <<
		1044.7786, 0, 985.9435,
		0, 1047.2506, 522.7765,
		0, 0, 1;

	Eigen::Matrix3f R_ir2rgb;
	Eigen::Matrix3f R;
	Eigen::Vector3f T_temp;
	Eigen::Vector3f T;
	R_ir2rgb <<
		0.9996, 0.0023, -0.0269,
		-0.0018, 0.9998, 0.0162,
		0.0269, -0.0162, 0.9995;
	T_temp <<
		65.9080,
		-4.1045,
		-13.9045;
	R = K_rgb*R_ir2rgb*K_ir.inverse();
	T = K_rgb*T_temp;


	//投影计算部分
	Mat result(424, 512, CV_8UC3);
	int i = 0;
	for (int row = 0; row < 424; row++)
	{
		for (int col = 0; col < 512; col++)
		{
			unsigned short* p = (unsigned short*)depth.data;
			unsigned short depthValue = p[row * 512 + col];
			//cout << "depthValue       " << depthValue << endl;
			if (depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != 0 && depthValue != 65535)
			{
				// 投影到彩色图上的坐标
				Eigen::Vector3f uv_depth(col, row, 1.0f);
				Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   //用于计算映射，核心式子

				int X = static_cast<int>(uv_color[0] / uv_color[2]);         //计算X，即对应的X值
				int Y = static_cast<int>(uv_color[1] / uv_color[2]);         //计算Y，即对应的Y值

				if ((X >= 0 && X < 1920) && (Y >= 0 && Y < 1080))
				{

					result.data[i * 3] = bgr.data[3 * (Y * 1920 + X)];
					result.data[i * 3 + 1] = bgr.data[3 * (Y * 1920 + X) + 1];
					result.data[i * 3 + 2] = bgr.data[3 * (Y * 1920 + X) + 2];
				}
			}
			i++;
		}
	}
	
//	thread th2 = std::thread([&]{
//		while (true)
//		{
//			imshow("结果图", result);
//			waitKey(1);
//		}
//	});
	imshow("result_image", result);
	imwrite("/home/gls/kinect/kinect_depth/peizhun.png",result);
	waitKey(0);

//	th.join();
//	th2.join();
	return 0;
}
