// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "kdl_conversions/kdl_msg.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <collision_convex_model/collision_convex_model.h>

#include "pointcloud_utils_msgs/FindPlanes.h"

#include "rcprg_ros_utils/marker_publisher.h"

class FindFeaturesPC {
public:
    ros::NodeHandle &nh_;
    bool point_cloud_processed_;
    double tolerance_;

    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud::Ptr pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;

    MarkerPublisher mp_;

public:

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        if (point_cloud_processed_) {
            point_cloud_processed_ = false;
            pcl::fromROSMsg(*cloud, *pc_);
            pc_stamp_ = cloud->header.stamp;
            pc_frame_id_ = cloud->header.frame_id;
        }
    }

    FindFeaturesPC(ros::NodeHandle &n) :
        nh_(n),
        mp_(n),
        point_cloud_processed_(true),
        tolerance_(0.04),
        pc_(new PclPointCloud)
    {
        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, tf_listener_, "world", 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&FindFeaturesPC::insertCloudCallback, this, _1));
    }

    ~FindFeaturesPC() {
    }
};

std::shared_ptr<FindFeaturesPC > pcf;

bool findPlanes(pointcloud_utils_msgs::FindPlanes::Request  &req,
         pointcloud_utils_msgs::FindPlanes::Response &res)
{
    if (pcf->pc_->points.size() == 0) {
        ROS_INFO("findPlanes: point cloud is empty");
        return false;
    }

    ROS_INFO("findPlanes: point cloud has %d points", (int)pcf->pc_->points.size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>(*pcf->pc_));

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (pc);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.01);

    // Compute the features
    ne.compute (*cloud_normals);

    ROS_INFO("findPlanes: computed normals");

    // flip all normals towards the camera
    if (cloud_normals->points.size() != pc->points.size()) {
        ROS_INFO("findPlanes: cloud_normals->points.size() != pc->points.size()");
        return false;
    }
    for (int i = 0; i < pc->points.size(); ++i) {
        double dot = pc->points[i].x * cloud_normals->points[i].normal_x + 
            pc->points[i].y * cloud_normals->points[i].normal_y +
            pc->points[i].z * cloud_normals->points[i].normal_z;
        if (dot > 0) {
            cloud_normals->points[i].normal_x = -cloud_normals->points[i].normal_x;
            cloud_normals->points[i].normal_y = -cloud_normals->points[i].normal_y;
            cloud_normals->points[i].normal_z = -cloud_normals->points[i].normal_z;
        }
    }

    ROS_INFO("findPlanes: flipped normals");

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    // Provide the original point cloud (without normals)
    principalCurvaturesEstimation.setInputCloud (pc);

    // Provide the point cloud with normals
    principalCurvaturesEstimation.setInputNormals(cloud_normals);

    // Use the same KdTree from the normal estimation
    principalCurvaturesEstimation.setSearchMethod (tree2);
    principalCurvaturesEstimation.setRadiusSearch(0.01);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvatures);

    int counter = 0;
    for (int i = 0; i < principalCurvatures->points.size(); ++i) {
        if (principalCurvatures->points[i].pc1 < 0.1) {
            counter++;
        }        
    }

    ROS_INFO("findPlanes: points on planes: %d", counter);






//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_blob (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (pc);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  // Convert to the templated PointCloud
//  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int m_id = 0;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    int size = cloud_p->points.size();
    double c_r = (double)(rand()%1000) / 1000.0;
    double c_g = (double)(rand()%1000) / 1000.0;
    double c_b = (double)(rand()%1000) / 1000.0;
//    for (int i = 0; i < 500; i++) {
//        int ptidx = rand()%size;
    for (int i = 0; i < size; i+=2) {
        int ptidx = i;
        m_id = pcf->mp_.addSinglePointMarker(m_id, KDL::Vector(cloud_p->points[ptidx].x, cloud_p->points[ptidx].y, cloud_p->points[ptidx].z), c_r, c_g, c_b, 1, 0.01, pcf->pc_frame_id_);
    }

//    std::stringstream ss;
//    ss << "table_scene_lms400_plane_" << i << ".pcd";
//    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

pcf->mp_.publish();





  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_features_pc");
    ros::NodeHandle n;
    pcf.reset( new FindFeaturesPC(n) );

    ros::ServiceServer service = n.advertiseService("find_planes", findPlanes);
    ros::spin();

    return 0;
}


