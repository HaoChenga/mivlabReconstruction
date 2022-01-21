#ifndef _HC_POINTCLOUD_H
#define _HC_POINTCLOUD_H

#include <string>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common.h> 
#include <pcl/filters/passthrough.h>  //直通滤波相关
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudRGBPtr;
//typedef pcl::PointCloud<pcl::Normal>::Ptr PointNormalPtr;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;
//typedef pcl::PolygonMesh Mesh;
typedef pcl::PointCloud<PointT> PointCloudRGB;
class MyPointCloud
{
public:
	MyPointCloud(std::string rgb_path, std::string depth_path, std::string incamera_path);
	virtual ~MyPointCloud() {};
	void depth2cloud();
	void joinMap();
protected:

private:
	cv::Mat rgb, depth,cameraMatrix, distCoef;
	PointCloudRGB::Ptr cloudoutput;

};

#endif
