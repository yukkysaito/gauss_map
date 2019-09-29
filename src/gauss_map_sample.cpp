#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d_omp.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
}

int main()
{
    // load
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("torus.pcd", *cloud);
    pcl::io::loadPCDFile("car6.pcd", *cloud);
    // pcl::io::loadPCDFile("raw_1.pcd", *cloud);

    // downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vgf;
    vgf.setInputCloud(cloud);
    vgf.setLeafSize(0.1f, 0.1f, 0.1f);
    vgf.filter(*cloud_filtered);

    // centroid
    double centroid_x(0.0), centroid_y(0.0), centroid_z(0.0);
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    {
        centroid_x += cloud_filtered->points.at(i).x;
        centroid_y += cloud_filtered->points.at(i).y;
        centroid_z += cloud_filtered->points.at(i).z;
    }
    centroid_x = centroid_x / (double)cloud_filtered->points.size();
    centroid_y = centroid_y / (double)cloud_filtered->points.size();
    centroid_z = centroid_z / (double)cloud_filtered->points.size();

    // estimate normal
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setViewPoint(centroid_x, centroid_y, centroid_z);
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_normals);

    // viewer
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer1"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_cloud(cloud_filtered, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, green_cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, cloud_normals, 1, 0.1, "normals");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(100);
        // }
    }
    // create gauss map
    pcl::PointCloud<pcl::PointXYZ>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud_normals->points.size(); ++i)
    {
        pcl::PointXYZ point;
        point.x = cloud_normals->points.at(i).normal_x;
        point.y = cloud_normals->points.at(i).normal_y;
        point.z = cloud_normals->points.at(i).normal_z;
        double l2_norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        point.x = point.x / l2_norm;
        point.y = point.y / l2_norm;
        point.z = point.z / l2_norm;
        gauss_cloud->points.push_back(point);
    }

    // viewer
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gauss_cloud_color(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (size_t i = 0; i < gauss_cloud->points.size(); ++i)
        {
            pcl::PointXYZRGBA point;
            point.x = gauss_cloud->points.at(i).x;
            point.y = gauss_cloud->points.at(i).y;
            point.z = gauss_cloud->points.at(i).z;
            point.r = 0;
            point.g = 255;
            point.b = 0;
            point.a = 100;
            gauss_cloud_color->points.push_back(point);
        }
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer2"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(gauss_cloud_color);
        viewer->addPointCloud<pcl::PointXYZRGBA>(gauss_cloud_color, rgba, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
    }
    return 0;
}