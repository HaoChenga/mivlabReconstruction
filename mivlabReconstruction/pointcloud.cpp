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
	////�����˲�,ֻ������������
	//pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>);
	//pcl::PassThrough<PointT> pass;     //�����˲�������
	//pass.setInputCloud(cloud);                //���ô��˲��ĵ���
	//pass.setFilterFieldName("z");             //������Z�᷽���Ͻ����˲�
	//pass.setFilterLimits(minPt.z, minPt.z + 0.3);    //�����˲���Χ
	//pass.setFilterLimitsNegative(false);      //����
	//pass.filter(*cloud_filter);               //�˲����洢
	//cloud_filter->height = 1;
	//cloud_filter->width = cloud_filter->points.size();
	//std::cout << "point cloud size = " << cloud_filter->points.size() << std::endl;
	//cloud_filter->is_dense = false;
	//cloudoutput = cloud_filter;
	cloud->height = 1;
	cloud->width = cloud->points.size();
	try {
		//�������ͼ
		pcl::io::savePCDFile("pointcloud.pcd", *cloud);

	}
	catch (pcl::IOException &e) {
		std::cout << e.what() << std::endl;
	}

	//��ʾ����ͼ
	pcl::visualization::CloudViewer viewer("pointcloud");//ֱ�Ӵ���һ����ʾ����
	viewer.showCloud(cloud);//�����������ʾ����
	while (!viewer.wasStopped())
	{
	}
	// ������ݲ��˳�
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
}

void MyPointCloud::joinMap()
{
	vector<cv::Mat> colorImgs, depthImgs;    // ��ɫͼ�����ͼ
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // ���λ��

	ifstream fin("../data/pose.txt");
	if (!fin)
	{
		cerr<<"Check the path of pose.txt"<<endl;
		return ;
	}

	for (int i = 0; i < 5; i++)
	{
		boost::format fmt("../data/%s/%d.%s"); //ͼ���ļ���ʽ
		colorImgs.push_back(cv::imread((fmt%"color" % (i + 1) % "png").str()));
		depthImgs.push_back(cv::imread((fmt%"depth" % (i + 1) % "pgm").str(), -1)); // ʹ��-1��ȡԭʼͼ��

		double data[7] = { 0 };
		for (auto& d : data)
			fin >> d;
		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
		Eigen::Isometry3d T(q);
		T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
		poses.push_back(T);
	}

	// ������Ʋ�ƴ��
	// ����ڲ� 
	double cx = 325.5;
	double cy = 253.5;
	double fx = 518.0;
	double fy = 519.0;
	double depthScale = 1000.0;

	cout << "Converting the image to a point cloud..." << endl;

	// �������ʹ�õĸ�ʽ�������õ���XYZRGB
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	// �½�һ������
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
				unsigned int d = depth.ptr<unsigned short>(v)[u]; // ���ֵ
				if (d == 0) continue; // Ϊ0��ʾû�в�����
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
	pcl::visualization::CloudViewer viewer("pointcloud");//ֱ�Ӵ���һ����ʾ����
	viewer.showCloud(pointCloud);//�����������ʾ����
	while (!viewer.wasStopped())
	{
	}
	
}
