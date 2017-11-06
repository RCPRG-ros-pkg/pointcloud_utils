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

#include <pointcloud_utils_msgs/AttachObject.h>

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
    ros::Publisher pub_pc_;
    ros::ServiceServer ss_attach_object_;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;

    // ROS params
    double horizontal_fov_;

    double vertical_fov_;
    double axial_fov_;

    int width_;
    int height_;

    std::vector<moveit_msgs::AttachedCollisionObject > attached_;
    boost::shared_ptr<self_collision::CollisionModel> col_model_;

public:

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        double aspect_w_h = double(cloud->width)/double(cloud->height);
        double dx = tan(horizontal_fov_/2);
        double dy = dx/aspect_w_h;
        if (axial_fov_ == 0.0) {
            axial_fov_ = atan(sqrt(dx*dx + dy*dy));
            vertical_fov_ = horizontal_fov_/double(cloud->width)*double(cloud->height);
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
            width_ = resized.width;
            height_ = resized.height;
        }
    }

    PCFilter()
        : nh_()
        , point_cloud_processed_(true)
        //, tolerance_(0.04)
        , axial_fov_(0.0)
        , vertical_fov_(0)
        , width_(0)
        , height_(0)
    {
        ros::NodeHandle private_nh("~");

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, tf_listener_, "world", 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&PCFilter::insertCloudCallback, this, _1));

        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_out", 10);
        //pub_pc_ex_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_ex_out", 10);

        // TODO: read FOV from ROS param
        private_nh.param("horizontal_fov", horizontal_fov_, horizontal_fov_);

        ss_attach_object_ = private_nh.advertiseService("attach_object", &PCFilter::attachObject, this);
    }

    ~PCFilter() {
    }

    bool attachObject(pointcloud_utils_msgs::AttachObject::Request  &req,
                      pointcloud_utils_msgs::AttachObject::Response &res)
    {
        // check validity of the request
        for (int i = 0; i < req.obj.size(); ++i) {
            bool found = false;
            for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
                if (req.obj[i].link_name == col_model_->getLink(l_idx)->name) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                ROS_ERROR_STREAM( "attachObject service: could not find link: " << req.obj[i].link_name );
                res.success = false;
                return false;
            }

            for (int j = 0; j < req.obj[i].object.primitives.size(); ++j) {
                if ( req.obj[i].object.primitives[j].type != shape_msgs::SolidPrimitive::SPHERE &&
                    req.obj[i].object.primitives[j].type != shape_msgs::SolidPrimitive::CYLINDER )
                {
                    ROS_ERROR_STREAM( "attachObject service: primitive shape of type " << int(req.obj[i].object.primitives[j].type)
                        << " is not supported (object attached to link " << req.obj[i].link_name << ")." );
                    res.success = false;
                    return false;
                }
            }

            if (req.obj[i].object.meshes.size() != 0) {
                ROS_ERROR_STREAM( "attachObject service: mesh shape is not supported (object attached to link "
                    << req.obj[i].link_name << ")." );
                res.success = false;
                return false;
            }
        }

        attached_ = req.obj;
        res.success = true;
        return true;
    }

    void calculateRayHits(KDL::Frame T_C_O, const self_collision::Geometry *geom, std::vector<bool > &pt_col) {
        const double PI = 3.141592653589793L;
        KDL::Vector bs_C = T_C_O * KDL::Vector();
        double dist_C = bs_C.Norm();
        double radius = geom->getBroadphaseRadius()+0.04;
        if (dist_C < 0.01 || dist_C < radius) {
            // the origin of the bounding sphere is near camera origin,
            // or the origin of camera is inside bounding sphere
            // perform check for all rays
            for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                if (pt_col[pidx]) {
                    continue;
                }
                KDL::Vector r_e(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z);
                KDL::Vector r_s = r_e;
                r_s.Normalize();
                r_s = r_s * 0.06;
                r_e = r_e + r_s;
                if (col_model_->checkRayCollision(geom, T_C_O, r_s, r_e)) {
                    pt_col[pidx] = true;
                }
            }
            //std::cout << "full check for link: " << plink->name << std::endl;
        }
        else {
            // get the angular size of the bounding sphere
            double angle_R = asin(radius/dist_C);
            // get angle of sphere origin (camera z axis is treated as x axis for atan2)
            // 1. the xz plane
            double angle_C_x = atan2(bs_C.x(), bs_C.z());
            if (fabs(angle_C_x) > horizontal_fov_/2 + PI/2) {
                // the bounding sphere is surely outside the frustrum
                return;
            }
            // determine the horizontal range
            int x_min = ((angle_C_x-angle_R)+horizontal_fov_/2)/horizontal_fov_*width_;
            int x_max = ((angle_C_x+angle_R)+horizontal_fov_/2)/horizontal_fov_*width_;
            x_min = std::max(std::min(x_min, width_), 0);
            x_max = std::max(std::min(x_max, width_), 0);

            // 2. the yz plane
            double angle_C_y = atan2(bs_C.y(), bs_C.z());
            if (fabs(angle_C_y) > vertical_fov_/2 + PI/2) {
                // the bounding sphere is surely outside the frustrum
                return;
            }
            // determine the vertical range
            int y_min = ((angle_C_y-angle_R)+vertical_fov_/2)/vertical_fov_*height_;
            int y_max = ((angle_C_y+angle_R)+vertical_fov_/2)/vertical_fov_*height_;
            y_min = std::max(std::min(y_min, height_), 0);
            y_max = std::max(std::min(y_max, height_), 0);

            //bool partial_check = false;
            for (int y = y_min; y < y_max; ++y) {
                for (int x = x_min; x < x_max; ++x) {
                    //partial_check = true;
                    int pidx = y * width_ + x;
                    if (pt_col[pidx]) {
                        continue;
                    }
                    KDL::Vector r_e(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z);
                    KDL::Vector r_s = r_e;
                    r_s.Normalize();
                    r_s = r_s * 0.06;
                    r_e = r_e + r_s;
                    if (col_model_->checkRayCollision(geom, T_C_O, r_s, r_e)) {
                        pt_col[pidx] = true;
                    }
                }
            }
            //if (partial_check) {
            //    std::cout << "partial check for link: " << plink->name << std::endl;
            //}
        }
    }

    void spin() {

        // read ROS param
        std::string robot_description_str;
        nh_.getParam("/robot_description", robot_description_str);

        //
        // collision model
        //
        col_model_ = self_collision::CollisionModel::parseURDF(robot_description_str);

        std::vector<KDL::Frame > links_tf(col_model_->getLinksCount());

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

                std::vector<bool > pt_col(pc_.points.size(), false);
                bool tf_error = false;
                //for (double t = -0.2; t <= 0.2; t += 0.02) {
                double t=0.0;
                {
                    tf::StampedTransform tf_W_C;
                    try {
                        tf_listener_.lookupTransform("world", pc_frame_id_, ros::Time(pc_stamp_.toSec()+t), tf_W_C);
                    } catch(tf::TransformException& ex){
                        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                        errors++;
                        continue;
                    }
                    geometry_msgs::TransformStamped tfm_W_C;
                    KDL::Frame T_W_C;
                    tf::transformStampedTFToMsg(tf_W_C, tfm_W_C);
                    tf::transformMsgToKDL(tfm_W_C.transform, T_W_C);

                    for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
                        const boost::shared_ptr< self_collision::Link > plink = col_model_->getLink(l_idx);
                        bool attached = false;
                        for (int i = 0; i < attached_.size(); ++i) {
                            if (attached_[i].link_name == plink->name) {
                                attached = true;
                            }
                        }
                        if (plink->collision_array.size() == 0 && !attached) {
                            continue;
                        }
                        tf::StampedTransform tf_W_L;
                        try {
                            tf_listener_.lookupTransform("world", plink->name, ros::Time(pc_stamp_.toSec()+t), tf_W_L);
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

                        //support for attached objects
                        for (int i = 0; i < attached_.size(); ++i) {
                            if (attached_[i].link_name == plink->name) {
                                for (int j = 0; j < attached_[i].object.primitives.size(); ++j) {
                                    KDL::Frame T_L_O;
                                    tf::poseMsgToKDL(attached_[i].object.primitive_poses[j], T_L_O);
                                    if ( attached_[i].object.primitives[j].type == shape_msgs::SolidPrimitive::SPHERE ) {
                                        double r = attached_[i].object.primitives[j].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
                                        self_collision::Sphere sphere(r);
                                        calculateRayHits(T_C_L * T_L_O, &sphere, pt_col);
                                    }
                                    else if ( attached_[i].object.primitives[j].type == shape_msgs::SolidPrimitive::CYLINDER ) {
                                        double h = attached_[i].object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
                                        double r = attached_[i].object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
                                        self_collision::Capsule capsule(r,h);
                                        calculateRayHits(T_C_L * T_L_O, &capsule, pt_col);
                                    }
                                }
                            }
                        }

                        for (self_collision::Link::VecPtrCollision::const_iterator it=plink->collision_array.begin(), end=plink->collision_array.end(); it != end; ++it) {
                            calculateRayHits(T_C_L * (*it)->origin, (*it)->geometry.get(), pt_col);
                        }
                    }

                    if (tf_error) {
                        //break;
                    }
                }

                if (tf_error) {
                    errors++;
                    continue;
                }

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


