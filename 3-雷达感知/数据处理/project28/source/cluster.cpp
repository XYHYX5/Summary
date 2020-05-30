#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);//创建了两个指向点云的指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 可视化部分
    pcl::visualization::PCLVisualizer viewer ("cluster");
    // 创建一个独立的视口
    int v1 (0),v2 (1);
    //里面的四个参数代表了
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    // 我们将要使用的颜色
    float BackgroundColor = 0.0;  // 黑色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 255,
                                                                                 0);//赋予显示点云的颜色，绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color1(output_cloud, 255, 255,
                                                                                  255);//赋予显示点云的颜色，红色
    viewer.addPointCloud<pcl::PointXYZ> (cloud,cloud_color, "cloud", v1);
    viewer.addPointCloud<pcl::PointXYZ> (output_cloud, cloud_color1, "colored_cloud", v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.5, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.5, "colored_cloud");
    viewer.setBackgroundColor (BackgroundColor, BackgroundColor, BackgroundColor, v1);
     viewer.setBackgroundColor (BackgroundColor, BackgroundColor, BackgroundColor, v2);
    // 设置相机位置和方向
    //前三个参数是相机位置，后三个是方向
    viewer.setCameraPosition (0,50, 20, 0, -50, 0, 0);
    viewer.setSize (1920, 1080);  // 设置可视化窗口的尺寸
    // Fill in the cloud data
   if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zhangjincan/PCD/pig.pcd",*cloud)==-1)//输入文件名
   {
       PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
       return (-1);
    }
   //欧式聚类
   // 建立kd-tree对象用来搜索 .
       pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
       kdtree->setInputCloud(cloud);
       // Euclidean 聚类对象.
       pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
       // 设置聚类的最小值 2cm (small values may cause objects to be divided
       // in several clusters, whereas big values may join objects in a same cluster).
       clustering.setClusterTolerance(0.3);
       // 设置聚类的小点数和最大点云数
       clustering.setMinClusterSize(100);
       clustering.setMaxClusterSize(1000000);
       clustering.setSearchMethod(kdtree);
       clustering.setInputCloud(cloud);
       std::vector<pcl::PointIndices> clusters;
       clustering.extract(clusters);
       // For every cluster...
       int currentClusterNum = 1;
       pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
       for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
       {
           //添加所有的点云到一个新的点云中
           for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
               cluster->points.push_back(cloud->points[*point]);
           cluster->width = cluster->points.size();
           cluster->height = 1;
           cluster->is_dense = true;
           // 保存
           if (cluster->points.size() <= 0)
               break;
           //std::cout << "Cluster " << currentClusterNum << " has " << output_cloud->points.size() << " points." << std::endl;
           //std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
           //pcl::io::savePCDFileASCII(fileName, *cluster);
           currentClusterNum++;
       }
       *output_cloud=*cluster;
       cout<<"input_cloud->points.size()="<<cloud->points.size()<<endl;
       cout<<"output_cloud->points.size()="<<output_cloud->points.size()<<endl;
   //
   viewer.updatePointCloud (cloud,cloud_color, "cloud");
   viewer.updatePointCloud (output_cloud,cloud_color1,"colored_cloud");
   while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (0);
}

