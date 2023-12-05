#include "AndreiUtils/utilsBinarySerialization.hpp"
#include "Model_PreProsessing.h"
#include "Database.h"


#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
#include <chrono>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h>


#include "example.hpp"
#include <librealsense2/rs.hpp>
#include <pcl/filters/random_sample.h>


const float K = 0.1;
const float C = 0.25;
const int M = 76742;
const float Ps = 0.9;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr points_to_pcl(const rs2::points& points);

void register_glfw_callbacks(window& app, glfw_state& app_state);
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);






int main() {


    OuterMap generatedMap = DB::create_hashMap(0.01,0.001, "../YCB_ply/Selected_two");
    DB::to_serialize_hashMap(generatedMap, "Adel.bin");
    std::string filename = "Adel.bin";
    OuterMap deserialized_map = DB::to_deserialize_hashMap(filename);


    /*
    window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
    register_glfw_callbacks(app, app_state);

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    pipe.start();
    while (app) // Application still alive?
    {
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        if (!color)
            color = frames.get_infrared_frame();
        pc.map_to(color);

        auto depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        auto pcl_points_S = points_to_pcl(points);
        // from S to S*
        float resolution = 0.02f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
        octree.setInputCloud(pcl_points_S);
        octree.addPointsFromInputCloud();

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector voxel_centers;
        octree.getOccupiedVoxelCenters(voxel_centers);

        downsampledCloud->points.resize(voxel_centers.size());
        std::copy(voxel_centers.begin(), voxel_centers.end(), downsampledCloud->points.begin());

        std::cout << "num of points after downsampling " << voxel_centers.size() << std::endl;
        // until here, I downsampled the scene by considering only the centroid of each voxel.
        // downsampledCloud : is the cloud of S*

        int n = downsampledCloud->size();  // scene points
        int N = (-n * log(1 - Ps)) / (M * K * C);    // number of iterations needed

        int num_points_to_sample = 1;

        pcl::PointCloud<pcl::PointXYZ>::Ptr Point_A_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Point_B_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ PointA;
        pcl::PointXYZ PointB;

        std::vector<int> point_indices;
        std::vector<float> point_distances;



        for (int i = 0; i < N ; ++i) {

            // Sample two points from S*
            pcl::RandomSample<pcl::PointXYZ> random_sample;
            random_sample.setInputCloud(downsampledCloud);
            random_sample.setSample(num_points_to_sample);
            random_sample.filter(*Point_A_cloud);
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_of_Model_Cloud;
            kdtree_of_Model_Cloud.setInputCloud(downsampledCloud);
            PointA = Point_A_cloud->points[0];
            kdtree_of_Model_Cloud.radiusSearch(PointA, radius, point_indices, point_distances);
            for (auto Ind : point_indices) {
                Point_B_cloud->points.push_back(downsampledCloud->points[Ind]);
            }
            PointB = Point_B_cloud->points[0];

            // Now, I sampled a pair of points

            // Now, I compute the normal of each point in the pair

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> na;
            na.setInputCloud (Point_A_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr treeA (new pcl::search::KdTree<pcl::PointXYZ> ());
            na.setSearchMethod (treeA);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_A (new pcl::PointCloud<pcl::Normal>);
            na.setRadiusSearch (0.03);
            na.compute (*cloud_normals_A);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nb;
            nb.setInputCloud (Point_B_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr treeB (new pcl::search::KdTree<pcl::PointXYZ> ());
            nb.setSearchMethod (treeB);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_B (new pcl::PointCloud<pcl::Normal>);
            nb.setRadiusSearch (0.03);
            nb.compute (*cloud_normals_B);

            // now, I concatenate the norm with the xyz
            pcl::PointCloud<pcl::PointXYZLNormal>::Ptr Pn_A (new pcl::PointCloud<pcl::PointXYZLNormal>);
            pcl::PointCloud<pcl::PointXYZLNormal>::Ptr Pn_B (new pcl::PointCloud<pcl::PointXYZLNormal>);

            pcl::concatenateFields (*Point_A_cloud, *cloud_normals_A, *Pn_A);
            pcl::concatenateFields (*Point_B_cloud, *cloud_normals_B, *Pn_B);


            // from now on, use Pn_A, Pn_B
            Eigen::Vector3f DESCRIPTOR = Computer_the_descriptor(Pn_A->points[0], Pn_B->points[0]);




            if (generatedMap.find(X) != generatedMap.end()) {
                std::cout << "this descriptor is in the hashMap" << std::endl;
            } else
                {
                    std::cout << "NOT in the hashMap" << std::endl;
                }






            }

        app_state.tex.upload(color);
        draw_pointcloud(app.width(), app.height(), app_state, points);
    }

    return EXIT_SUCCESS;
*/


    return 0;
}







pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}