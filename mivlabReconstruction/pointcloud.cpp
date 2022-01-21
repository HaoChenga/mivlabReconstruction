#include "pointcloud.h"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>
#include <iostream>
#include <fstream>
using namespace std; 

MyPointCloud::MyPointCloud(std::string rgb_path, std::string depth_path, std::string incamera_path)
{
	rgb = cv::imread(rgb_path);
	depth = cv::imread(depth_path, cv::IMREAD_ANYDEPTH);
	//cv::Mat cameraMatrix, distCoef;
	cv::FileStorage fs(incamera_path, cv::FileStorage::READ);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoef;
	fs.release();

}
void MyPointCloud::depth2cloud()
{
	PointCloudRGB::Ptr cloud(new PointCloudRGB);
	float U0,V0,fX,fy;
	U0 = cameraMatrix.at<double>(0, 2);
	V0 = cameraMatrix.at<double>(1, 2);
	fX = cameraMatrix.at<double>(0, 0);
	fy = cameraMatrix.at<double>(1, 1);

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			PointT p;
			ushort d = depth.ptr<ushort>(m)[n];
			if (d == 0)
			{
				continue;
			}
			else
			{
				p.z = double(d) / 1000;
				p.x = (n - U0) * p.z / fX;
				p.y = ((m - V0) * p.z / fy);

				p.b = rgb.ptr<uchar>(m)[n * 3];;
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
				cloud->points.push_back(p);
			}
		}

	//PointT minPt, maxPt;
	//pcl::getMinMax3D(*cloud, minPt, maxPt);
	////进行滤波,只保留人脸部分
	//pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>);
	//pcl::PassThrough<PointT> pass;     //创建滤波器对象
	//pass.setInputCloud(cloud);                //设置待滤波的点云
	//pass.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
	//pass.setFilterLimits(minPt.z, minPt.z + 0.3);    //设置滤波范围
	//pass.setFilterLimitsNegative(false);      //保留
	//pass.filter(*cloud_filter);               //滤波并存储
	//cloud_filter->height = 1;
	//cloud_filter->width = cloud_filter->points.size();
	//std::cout << "point cloud size = " << cloud_filter->points.size() << std::endl;
	//cloud_filter->is_dense = false;
	//cloudoutput = cloud_filter;
	cloud->height = 1;
	cloud->width = cloud->points.size();
	try {
		//保存点云图
		pcl::io::savePCDFile("pointcloud.pcd", *cloud);

	}
	catch (pcl::IOException &e) {
		std::cout << e.what() << std::endl;
	}

	//显示点云图
	pcl::visualization::CloudViewer viewer("pointcloud");//直接创造一个显示窗口
	viewer.showCloud(cloud);//在这个窗口显示点云
	while (!viewer.wasStopped())
	{
	}
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
}

void MyPointCloud::joinMap()
{
	vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿

	ifstream fin("../data/pose.txt");
	if (!fin)
	{
		cerr<<"Check the path of pose.txt"<<endl;
		return ;
	}

	for (int i = 0; i < 5; i++)
	{
		boost::format fmt("../data/%s/%d.%s"); //图像文件格式
		colorImgs.push_back(cv::imread((fmt%"color" % (i + 1) % "png").str()));
		depthImgs.push_back(cv::imread((fmt%"depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像

		double data[7] = { 0 };
		for (auto& d : data)
			fin >> d;
		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
		Eigen::Isometry3d T(q);
		T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
		poses.push_back(T);
	}

	// 计算点云并拼接
	// 相机内参 
	double cx = 325.5;
	double cy = 253.5;
	double fx = 518.0;
	double fy = 519.0;
	double depthScale = 1000.0;

	cout << "Converting the image to a point cloud..." << endl;

	// 定义点云使用的格式：这里用的是XYZRGB
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	// 新建一个点云
	PointCloud::Ptr pointCloud(new PointCloud);
	for (int i = 0; i < 5; i++)
	{
		cout << "Converting the image: " << i + 1 << endl;
		cv::Mat color = colorImgs[i];
		cv::Mat depth = depthImgs[i];
		Eigen::Isometry3d T = poses[i];
		for (int v = 0; v < color.rows; v++)
			for (int u = 0; u < color.cols; u++)
			{
				unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
				if (d == 0) continue; // 为0表示没有测量到
				Eigen::Vector3d point;
				point[2] = double(d) / depthScale;
				point[0] = (u - cx)*point[2] / fx;
				point[1] = (v - cy)*point[2] / fy;
				Eigen::Vector3d pointWorld = T*point;

				PointT p;
				p.x = pointWorld[0];
				p.y = pointWorld[1];
				p.z = pointWorld[2];
				p.b = color.data[v*color.step + u*color.channels()];
				p.g = color.data[v*color.step + u*color.channels() + 1];
				p.r = color.data[v*color.step + u*color.channels() + 2];
				pointCloud->points.push_back(p);
			}
	}

	pointCloud->is_dense = false;
	cout << "Pointcloud has " << pointCloud->size() << " points." << endl;
	pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
	pcl::visualization::CloudViewer viewer("pointcloud");//直接创造一个显示窗口
	viewer.showCloud(pointCloud);//在这个窗口显示点云
	while (!viewer.wasStopped())
	{
	}
	
}
