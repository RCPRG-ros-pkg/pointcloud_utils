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

#include <collision_convex_model/collision_convex_model.h>

class BoundingSphere {
public:
    double radius_;
    KDL::Vector pos_; 
};

class PCFilter {
    ros::NodeHandle nh_;
    bool point_cloud_processed_;
    //double tolerance_;

    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
    ros::Publisher pub_pc_;//, pub_pc_ex_;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;

    // ROS params
    double horizontal_fov_;
    double axial_fov_;

public:

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        double aspect_w_h = double(cloud->width)/double(cloud->height);
        double dx = tan(horizontal_fov_/2);
        double dy = dx/aspect_w_h;
        if (axial_fov_ == 0.0) {
            axial_fov_ = atan(sqrt(dx*dx + dy*dy));            
        }

        if (point_cloud_processed_) {
            point_cloud_processed_ = false;
            sensor_msgs::PointCloud2 resized;
            resized.header = cloud->header;
            resized.height = cloud->height/2;
            resized.width = cloud->width/2;
            resized.fields = cloud->fields;
            resized.is_bigendian = cloud->is_bigendian;
            resized.point_step = cloud->point_step;
            resized.row_step = cloud->row_step/2;
            resized.is_dense = cloud->is_dense;
            resized.data.resize(resized.height * resized.row_step);
            for (int y = 0; y < cloud->height; y += 2) {
                for (int x = 0; x < cloud->width; x += 2) {
                    for (int i = 0; i < cloud->point_step; ++i) {
                        resized.data[int(y/2)*resized.row_step + int(x/2)*resized.point_step + i] = cloud->data[y*cloud->row_step + x*cloud->point_step + i];
                    }
                    unsigned char *ptr = &resized.data[int(y/2)*resized.row_step + int(x/2)*resized.point_step];
                    float px = *((float*)(ptr));
                    float py = *((float*)(ptr)+1);
                    float pz = *((float*)(ptr)+2);

                    if (px != px || py != py || pz!= pz) {
                        px = (float(x)/float(cloud->width)*2.0-1.0)*dx*2.5;
                        py = (float(y)/float(cloud->height)*2.0-1.0)*dy*2.5;
                        pz = 2.5;
                        *((float*)(ptr)) = px;
                        *((float*)(ptr)+1) = py;
                        *((float*)(ptr)+2) = pz;
                        ptr[16] = 100;
                        ptr[17] = 100;
                        ptr[18] = 100;
                        ptr[19] = 100;
                    }
                }
            }

            pcl::fromROSMsg(resized, pc_);
            pc_stamp_ = cloud->header.stamp;
            pc_frame_id_ = cloud->header.frame_id;
        }
    }

    PCFilter() :
        nh_(),
        point_cloud_processed_(true),
        //tolerance_(0.04),
        axial_fov_(0.0)
    {
        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, tf_listener_, "world", 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&PCFilter::insertCloudCallback, this, _1));

        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_out", 10);
        //pub_pc_ex_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_ex_out", 10);

        // TODO: read FOV from ROS param
        horizontal_fov_ = 1.047;
    }

    ~PCFilter() {
    }

    void spin() {

        // read ROS param
        std::string robot_description_str;
        nh_.getParam("/robot_description", robot_description_str);

        //
        // collision model
        //
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);

        std::vector<KDL::Frame > links_tf(col_model->getLinksCount());

        //boost::shared_ptr< self_collision::Collision > shpere = self_collision::createCollisionSphere(tolerance_, KDL::Frame());

        ros::Rate loop_rate(5);
        int errors = 0;
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();

            if (errors > 5) {
                point_cloud_processed_ = true;
            }

            if (!point_cloud_processed_) {

                tf::StampedTransform tf_W_C;
                try {
                    tf_listener_.lookupTransform("world", pc_frame_id_, pc_stamp_, tf_W_C);
                } catch(tf::TransformException& ex){
                    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                    errors++;
                    continue;
                }
                geometry_msgs::TransformStamped tfm_W_C;
                KDL::Frame T_W_C;
                tf::transformStampedTFToMsg(tf_W_C, tfm_W_C);
                tf::transformMsgToKDL(tfm_W_C.transform, T_W_C);

                // for debug
                std::vector<std::string > dbg_links_inside;

                std::vector<bool > pt_col(pc_.points.size(), false);
                bool tf_error = false;
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    const boost::shared_ptr< self_collision::Link > plink = col_model->getLink(l_idx);
                    if (plink->collision_array.size() == 0) {
                        continue;
                    }
                    tf::StampedTransform tf_W_L;
                    try {
                        tf_listener_.lookupTransform("world", plink->name, pc_stamp_, tf_W_L);
                    } catch(tf::TransformException& ex){
                        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                        tf_error = true;
                        break;
                    }
                    geometry_msgs::TransformStamped tfm_W_L;
                    KDL::Frame T_W_L;
                    tf::transformStampedTFToMsg(tf_W_L, tfm_W_L);
                    tf::transformMsgToKDL(tfm_W_L.transform, T_W_L);
                    KDL::Frame T_C_L = T_W_C.Inverse() * T_W_L;
                    links_tf[l_idx] = T_W_L;

                    // check if the bounding sphere of any collision object of the link intersects with the frustrum
                    bool outside = true;
                    for (self_collision::Link::VecPtrCollision::const_iterator it=plink->collision_array.begin(), end=plink->collision_array.end(); it != end; ++it) {
                        KDL::Frame T_C_COL = T_C_L * (*it)->origin;
                        KDL::Vector bs_C = T_C_COL * KDL::Vector();
                        double x = sqrt(bs_C.x()*bs_C.x() + bs_C.y()*bs_C.y());
                        double y = bs_C.z();
                        double v = x * tan(axial_fov_) + y; // z coordinate of point projection into z axis along frustrum rarial
                        double d = x * cos(axial_fov_) - y * sin(axial_fov_);
                        if ( (v > 0 && d < (*it)->geometry->getBroadphaseRadius()) ||
                                (v <= 0 && bs_C.Norm() < (*it)->geometry->getBroadphaseRadius())) {
                            outside = false;

                            for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                                if (pt_col[pidx]) {
                                    continue;
                                }
                                KDL::Vector r_e(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z);
                                KDL::Vector r_s = r_e;
                                r_s.Normalize();
                                r_s = r_s * 0.04;
                                if (col_model->checkRayCollision((*it)->geometry.get(), T_C_COL, r_s, r_e)) {
                                    pt_col[pidx] = true;
                                }
                            }
                        }
                    }
                    if (outside) {
                        continue;
                    }

                    dbg_links_inside.push_back(plink->name);
/*
                    for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                        if (pt_col[pidx]) {
                            continue;
                        }
                        KDL::Frame T_C_P(KDL::Vector(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z));
                        if (self_collision::checkCollision(shpere, T_C_P, plink, T_C_L)) {
                            pt_col[pidx] = true;
                        }
                    }
*/
                }
                if (tf_error) {
                    errors++;
                    continue;
                }

                std::cout << "links in frustrum: ";
                for (int i = 0; i < dbg_links_inside.size(); ++i) {
                    std::cout << dbg_links_inside[i] << ", ";
                }
                std::cout << std::endl;

                PclPointCloud pc_out;
                //PclPointCloud pc_ex_out;
                for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                    //pc_ex_out.push_back( pc_.points[pidx] );
                    if (!pt_col[pidx]) {
                        pc_out.push_back( pc_.points[pidx] );
                    }
                }

                sensor_msgs::PointCloud2 ros_pc_out;
                toROSMsg(pc_out, ros_pc_out);
                ros_pc_out.header.stamp = pc_stamp_;
                ros_pc_out.header.frame_id = pc_frame_id_;
                pub_pc_.publish(ros_pc_out);

                //sensor_msgs::PointCloud2 ros_pc_ex_out;
                //toROSMsg(pc_ex_out, ros_pc_ex_out);
                //ros_pc_ex_out.header.stamp = pc_stamp_;
                //ros_pc_ex_out.header.frame_id = pc_frame_id_;
                //pub_pc_ex_.publish(ros_pc_ex_out);

                point_cloud_processed_ = true;
                errors = 0;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_filter");
    PCFilter pcf;
    pcf.spin();
    return 0;
}


