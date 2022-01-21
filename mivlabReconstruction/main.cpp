#include "pointcloud.h"

int main()
{
	//system("chcp 65001");
	MyPointCloud Genpointcloud = MyPointCloud("../data/color/1.png", "../data/depth/1.pgm", "../data/cameradata.yml");
	Genpointcloud.depth2cloud();
	Genpointcloud.joinMap();
}