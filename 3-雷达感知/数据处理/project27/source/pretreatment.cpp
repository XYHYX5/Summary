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
using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);//创建了两个指向点云的指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // 可视化部分
    pcl::visualization::PCLVisualizer viewer ("filter");
    // 创建一个独立的视口
    int v1 (0),v2 (1);
    //里面的四个参数代表了
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    // 我们将要使用的颜色
    float BackgroundColor = 0.0;  // 黑色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 255,
                                                                                 0);//赋予显示点云的颜色，绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color1(cloud_filtered, 255, 255,
                                                                                  255);//赋予显示点云的颜色，红色
    viewer.addPointCloud<pcl::PointXYZ> (cloud,cloud_color, "cloud", v1);
    viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, cloud_color1, "colored_cloud", v2);
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
   //点云滤波
   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;// 创建滤波器对象
   sor.setInputCloud(cloud);                        //设置呆滤波的点云
   sor.setMeanK(50);                                //设置在进行统计时考虑查询点邻近点数
   sor.setStddevMulThresh(1.0);                    //设置判断是否为离群点的阈值
   sor.filter(*cloud_filtered);                    //执行滤波处理保存内点到cloud_filtered
   //
   viewer.updatePointCloud (cloud,cloud_color, "cloud");
   viewer.updatePointCloud (cloud_filtered,cloud_color1,"colored_cloud");
   while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (0);
}

