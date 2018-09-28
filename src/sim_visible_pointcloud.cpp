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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <collision_convex_model/collision_convex_model.h>
#include <kin_dyn_model/kin_model.h>

#include <dart/dynamics/MeshShape.hpp>

#include <pointcloud_utils_msgs/SimVisiblePointCloud.h>
#include <pointcloud_utils_msgs/FindPointCloudCorrespondence.h>

class SimVisiblePointCloud {
    ros::NodeHandle nh_;
    bool point_cloud_processed_;
    double tolerance_;

    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
    ros::Publisher pub_pc_;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;


    boost::shared_ptr<KinematicModel> kin_model_;
    boost::shared_ptr<self_collision::CollisionModel> col_model_;
    std::vector<KDL::Frame > links_fk_;
    std::vector<std::vector<std::pair<KDL::Frame, const aiScene* > > > collision_meshes_;
    std::vector<std::pair<double, KDL::Vector > > link_bounding_spheres_;

    ros::ServiceServer ss_sim_visible_pointcloud_;
    ros::ServiceServer ss_find_pointcloud_correspondence_;

    // ROS parameters
    std::vector<std::string > articulated_joint_names_;
    std::string camera_frame_id_;
    double pc_width_;
    double pc_height_;
    double fov_x_;
    double fov_y_;

public:

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        if (point_cloud_processed_) {
            point_cloud_processed_ = false;
            pcl::fromROSMsg(*cloud, pc_);
            pc_stamp_ = cloud->header.stamp;
            pc_frame_id_ = cloud->header.frame_id;
        }
    }

    SimVisiblePointCloud()
        : nh_("~")
        , pc_width_(0)
        , pc_height_(0)
        , fov_x_(0)
        , fov_y_(0)
//        , point_cloud_processed_(true)
//        , tolerance_(0.04)
    {
        ss_sim_visible_pointcloud_ = nh_.advertiseService("sim_visible_pointcloud", &SimVisiblePointCloud::simVisiblePointCloud, this);
        ss_find_pointcloud_correspondence_ = nh_.advertiseService("find_pointcloud_correspondence", &SimVisiblePointCloud::findPointCloudCorrespondence, this);

//        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
//        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, tf_listener_, "world", 5);
//        m_tfPointCloudSub->registerCallback(boost::bind(&SimVisiblePointCloud::insertCloudCallback, this, _1));
//        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_out", 10);
    }

    ~SimVisiblePointCloud() {
    }

    bool rayTriangleIntersect(const KDL::Vector &orig, const KDL::Vector &dir,
                                const KDL::Vector &v0, const KDL::Vector &v1, const KDL::Vector &v2,
                                const KDL::Vector &N, double &t)
    {
        const double kEpsilon = 0.000001;

        // compute plane's normal
//        KDL::Vector v0v1 = v1 - v0;
//        KDL::Vector v0v2 = v2 - v0;
        // no need to normalize
//        KDL::Vector N = v0v1 * v0v2; // N

//        double area2 = N.Norm();
//        if (area2 < kEpsilon) {
//            return false;
//        }

        // Step 1: finding P

        // check if ray and plane are parallel ?
        double NdotRayDirection = KDL::dot(N, dir);
        if (fabs(NdotRayDirection) < kEpsilon) // almost 0
        return false; // they are parallel so they don't intersect !

        // compute d parameter using equation 2
        double d = -KDL::dot(N,v0);

        // compute t (equation 3)
        t = -(KDL::dot(N,orig) + d) / NdotRayDirection;
        // check if the triangle is in behind the ray
        if (t < 0) return false; // the triangle is behind

        // compute the intersection point using equation 1
        KDL::Vector P = orig + (t * dir);
        if (fabs(KDL::dot(P,N) + d) > 0.00001) {
            std::cout << "rayTriangleIntersect: computational error #1" << std::endl;
        }

        // Step 2: inside-outside test
        KDL::Vector C; // vector perpendicular to triangle's plane

        // edge 0
        KDL::Vector edge0 = v1 - v0;
        KDL::Vector vp0 = P - v0;
        C = edge0 * vp0;
        if (KDL::dot(N,C) < 0) return false; // P is on the right side

        // edge 1
        KDL::Vector edge1 = v2 - v1;
        KDL::Vector vp1 = P - v1;
        C = edge1 * vp1;
        if (KDL::dot(N,C) < 0) return false; // P is on the right side

        // edge 2
        KDL::Vector edge2 = v0 - v2;
        KDL::Vector vp2 = P - v2;
        C = edge2 * vp2;
        if (KDL::dot(N,C) < 0) return false; // P is on the right side;

        return true; // this ray hits the triangle
    } 

    bool load() {
        std::string robot_description_str;
        nh_.getParam("/robot_description", robot_description_str);

        nh_.getParam("articulated_joint_names", articulated_joint_names_);
        if (articulated_joint_names_.size() == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'articulated_joint_names' is not set" );
            return false;
        }

        nh_.getParam("camera_frame_id", camera_frame_id_);
        if (camera_frame_id_.empty()) {
            ROS_ERROR_STREAM( "ROS parameter 'camera_frame_id' is not set" );
            return false;
        }
        nh_.getParam("pc_width", pc_width_);
        if (pc_width_ == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'pc_width' is not set" );
            return false;
        }
        nh_.getParam("pc_height", pc_height_);
        if (pc_height_ == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'pc_height' is not set" );
            return false;
        }
        nh_.getParam("fov_x", fov_x_);
        if (fov_x_ == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'fov_x' is not set" );
            return false;
        }
        nh_.getParam("fov_y", fov_y_);
        if (fov_y_ == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'fov_y' is not set" );
            return false;
        }

        //
        // collision model
        //
        col_model_ = self_collision::CollisionModel::parseURDF(robot_description_str);
        if (!col_model_) {
            ROS_ERROR_STREAM( "Could not parse '/robot_description' ROS param" );
            return false;
        }

        links_fk_.resize(col_model_->getLinksCount());

        kin_model_.reset( new KinematicModel(robot_description_str, articulated_joint_names_) );

        const std::string package_uri_prefix("package://");

        for (int i = 0; i < col_model_->getLinksCount(); ++i) {
            KDL::Vector pt_min(1000000, 1000000, 1000000), pt_max(-1000000, -1000000, -1000000);
            boost::shared_ptr<self_collision::Link > link = col_model_->getLink(i);
            std::vector<std::pair<KDL::Frame, const aiScene* > > col_vec;
            for (int j = 0; j < link->visual_array_.size(); ++j) {
                if (link->visual_array_[j]->geom_->getType() == self_collision::VisualGeometry::MESH) {
                    boost::shared_ptr<self_collision::VisualMesh > mesh = boost::static_pointer_cast<self_collision::VisualMesh >(link->visual_array_[j]->geom_);
                    //if (!mesh) {
                    //    ROS_ERROR_STREAM( "Link '" << link->name << "' has wrong visual mesh." );
                    //    return false;
                    //}

                    std::string mesh_path;
                    if (mesh->filename_.size() > package_uri_prefix.size() && mesh->filename_.substr(0, package_uri_prefix.size()) == package_uri_prefix) {
                        size_t package_end_idx = mesh->filename_.find('/', package_uri_prefix.size());
                        if (package_end_idx == std::string::npos) {
                            ROS_ERROR_STREAM( "Link '" << link->name << "' has wrong visual mesh filename." );
                            return false;
                        }
                        const std::string package_name = mesh->filename_.substr(package_uri_prefix.size(), package_end_idx-package_uri_prefix.size());
                        const std::string sub_path = mesh->filename_.substr(package_end_idx);
                        const std::string package_path = ros::package::getPath(package_name);
                        mesh_path = package_path + sub_path;
                        //ROS_INFO_STREAM( "package: '" << package_name << "'" );
                        //ROS_INFO_STREAM( "sub_path: '" << sub_path << "'" );
                        //ROS_INFO_STREAM( "package_path: '" << package_path << "'" );
                        //ROS_INFO_STREAM( "Mesh path: '" << mesh_path << "'" );
                    }
                    else {
                        mesh_path = mesh->filename_;
                    }
                    const aiScene *sc = dart::dynamics::MeshShape::loadMesh(mesh_path);

                    const KDL::Frame &T_L_M = link->visual_array_[j]->origin_;

                    for (int m_idx = 0; m_idx < sc->mNumMeshes; ++m_idx) {
                        const aiMesh *mesh = sc->mMeshes[m_idx];
                        for (int v_idx = 0; v_idx < mesh->mNumVertices; ++v_idx) {
                            KDL::Vector v_M = KDL::Vector(mesh->mVertices[v_idx].x, mesh->mVertices[v_idx].y, mesh->mVertices[v_idx].z);
                            KDL::Vector v_L = T_L_M * v_M;
                            if (v_L.x() > pt_max.x()) {
                                pt_max.x(v_L.x());
                            }
                            if (v_L.y() > pt_max.y()) {
                                pt_max.y(v_L.y());
                            }
                            if (v_L.z() > pt_max.z()) {
                                pt_max.z(v_L.z());
                            }
                            if (v_L.x() < pt_min.x()) {
                                pt_min.x(v_L.x());
                            }
                            if (v_L.y() < pt_min.y()) {
                                pt_min.y(v_L.y());
                            }
                            if (v_L.z() < pt_min.z()) {
                                pt_min.z(v_L.z());
                            }
                        }
                    }

                    col_vec.push_back( std::pair<KDL::Frame, const aiScene* >(link->visual_array_[j]->origin_, sc) );
                }
            }
            collision_meshes_.push_back(col_vec);

            link_bounding_spheres_.push_back( std::pair<double, KDL::Vector >((pt_max-pt_min).Norm()/2, (pt_min+pt_max)/2) );

            ROS_INFO_STREAM( "link '" << link->name << "' has " << col_vec.size() << " visual meshes of " << link->visual_array_.size() << " visuals total" );
        }
        return true;
    }

    size_t getNumVertices(const aiScene *m) {
        size_t result = 0;
        for (int m_idx = 0; m_idx < m->mNumMeshes; ++m_idx) {
            result += m->mMeshes[m_idx]->mNumVertices;
        }
        return result;
    }

    size_t getNumFaces(const aiScene *m) {
        size_t result = 0;
        for (int m_idx = 0; m_idx < m->mNumMeshes; ++m_idx) {
            result += m->mMeshes[m_idx]->mNumFaces;
        }
        return result;
    }
/*
    bool rayMeshIntersect(const KDL::Vector &orig, const KDL::Vector &dir,
                                const aiScene *m,
                                double &t) {
//        ROS_WARN_STREAM( "rayMeshIntersect " << orig.x() << " " << orig.y() << " " << orig.z() << " " << dir.x() << " " << dir.y() << " " << dir.z() );
        bool hit = false;
        for (int m_idx = 0; m_idx < m->mNumMeshes; ++m_idx) {
            aiMesh *mesh = m->mMeshes[m_idx];
            for (int f_idx = 0; f_idx < mesh->mNumFaces; ++f_idx) {
                if (mesh->mFaces[f_idx].mNumIndices != 3) {
                    ROS_ERROR_STREAM( "Face is not triangle!" );
                }
                unsigned int i1 = mesh->mFaces[f_idx].mIndices[0];
                unsigned int i2 = mesh->mFaces[f_idx].mIndices[1];
                unsigned int i3 = mesh->mFaces[f_idx].mIndices[2];
                if (i1 >= mesh->mNumVertices || i2 >= mesh->mNumVertices || i3 >= mesh->mNumVertices) {
                    ROS_ERROR_STREAM( "Vertex index out of bounds!" );
                }
                const aiVector3D &v1 = mesh->mVertices[i1];
                const aiVector3D &v2 = mesh->mVertices[i2];
                const aiVector3D &v3 = mesh->mVertices[i3];

//                ROS_INFO_STREAM( "rayMeshIntersect " << v1.x << " " << v1.y << " " << v1.z << " " << v2.x << " " << v2.y << " " << v2.z << " " << v3.x << " " << v3.y << " " << v3.z  );

                double dist;
                if (rayTriangleIntersect(orig, dir, KDL::Vector(v1.x, v1.y, v1.z), KDL::Vector(v2.x, v2.y, v2.z), KDL::Vector(v3.x, v3.y, v3.z), dist) && dist > 0.01) {
                    if (!hit) {
                        t = dist;
                        hit = true;
                    }
                    else if (dist < t) {
                        t = dist;
                    }
                }
            }
        }
        return hit;
    }
*/
    std::vector<KDL::Vector > calculatePointCloud(const Eigen::VectorXd &q) {
        kin_model_->calculateFkAll(q);

        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
            links_fk_[l_idx] = kin_model_->getFrame(col_model_->getLinkName(l_idx));
        }

        const KDL::Frame &T_B_H = kin_model_->getFrame(camera_frame_id_);
        KDL::Frame T_H_B = T_B_H.Inverse();

        // frustum boundaries
        double rx, ry, rz;
        rz = 1.0;
        rx = tan(fov_x_) * rz;
        ry = tan(fov_y_) * rz;
        // boundary planes normals (normals point outside)
        KDL::Vector n1(rz, 0, -rx), n2(-rz, 0, -rx), n3(0, rz, -ry), n4(0, -rz, -ry);
        n1.Normalize();
        n2.Normalize();
        n3.Normalize();
        n4.Normalize();

        size_t vertex_idx_base = 0;
        std::vector<KDL::Vector > vec_v;
        std::vector<Eigen::Vector3i > vec_f;
        std::vector<KDL::Vector > vec_fn;

        size_t max_num_vertices = 0;
        size_t max_num_faces = 0;
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); ++l_idx) {
            for (int c_idx = 0; c_idx < collision_meshes_[l_idx].size(); ++c_idx) {
                const aiScene *m = collision_meshes_[l_idx][c_idx].second;
                max_num_vertices += getNumVertices(m);
                max_num_faces += getNumFaces(m);
            }
        }
        vec_v.reserve(max_num_vertices);
        vec_f.reserve(max_num_faces);
        vec_fn.reserve(max_num_faces);

        // create mesh of all visible object surfaces
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); ++l_idx) {
            const KDL::Frame &T_B_L = links_fk_[l_idx];
            KDL::Frame T_H_L = T_H_B * T_B_L;

            // broadphase check using bounding sphere
            double bs_radius = link_bounding_spheres_[l_idx].first;
            const KDL::Vector &bs_center_L = link_bounding_spheres_[l_idx].second;
            const KDL::Vector bs_center_H = T_H_L * bs_center_L;
            if (KDL::dot(bs_center_H, n1) > bs_radius || KDL::dot(bs_center_H, n2) > bs_radius ||
                    KDL::dot(bs_center_H, n3) > bs_radius || KDL::dot(bs_center_H, n4) > bs_radius) {
                continue;
            }

            for (int c_idx = 0; c_idx < collision_meshes_[l_idx].size(); ++c_idx) {
                const KDL::Frame &T_L_M = collision_meshes_[l_idx][c_idx].first;
                KDL::Frame T_H_M = T_H_L * T_L_M;
                const aiScene *m = collision_meshes_[l_idx][c_idx].second;

                for (int m_idx = 0; m_idx < m->mNumMeshes; ++m_idx) {
                    aiMesh *mesh = m->mMeshes[m_idx];
                    for (int v_idx = 0; v_idx < mesh->mNumVertices; ++v_idx) {
                        const aiVector3D &v = mesh->mVertices[v_idx];
                        vec_v.push_back(T_H_M * KDL::Vector(v.x, v.y, v.z));
                    }
                    for (int f_idx = 0; f_idx < mesh->mNumFaces; ++f_idx) {
                        if (mesh->mFaces[f_idx].mNumIndices != 3) {
                            ROS_ERROR_STREAM( "Face is not triangle!" );
                        }
                        unsigned int i1 = mesh->mFaces[f_idx].mIndices[0]+vertex_idx_base;
                        unsigned int i2 = mesh->mFaces[f_idx].mIndices[1]+vertex_idx_base;
                        unsigned int i3 = mesh->mFaces[f_idx].mIndices[2]+vertex_idx_base;
                        KDL::Vector v12 = vec_v[i2] - vec_v[i1];
                        KDL::Vector v13 = vec_v[i3] - vec_v[i1];
                        KDL::Vector n = v12 * v13;

//                        if (n.z() > 0) {    // the face is pointing outwards camera
//                            continue;
//                        }
                        if (n.Norm() < 0.0000001) {      // the face is too small
                            continue;
                        }
                        n.Normalize();

                        // the whole face is outside frustum
                        if ((KDL::dot(vec_v[i1], n1) > 0 && KDL::dot(vec_v[i2], n1) > 0 && KDL::dot(vec_v[i3], n1) > 0) ||
                                (KDL::dot(vec_v[i1], n2) > 0 && KDL::dot(vec_v[i2], n2) > 0 && KDL::dot(vec_v[i3], n2) > 0) ||
                                (KDL::dot(vec_v[i1], n3) > 0 && KDL::dot(vec_v[i2], n3) > 0 && KDL::dot(vec_v[i3], n3) > 0) ||
                                (KDL::dot(vec_v[i1], n4) > 0 && KDL::dot(vec_v[i2], n4) > 0 && KDL::dot(vec_v[i3], n4) > 0)) {
                            continue;
                        }
                        vec_f.push_back(Eigen::Vector3i(i1, i2, i3));
                        vec_fn.push_back(n);
                    }
                    vertex_idx_base += mesh->mNumVertices;
                }
            }
        }
        std::vector<KDL::Vector > pc_H;

        std::vector<double > rays(pc_width_*pc_height_, 1000000.0);
        for (int f_idx = 0; f_idx < vec_f.size(); ++f_idx) {
            const KDL::Vector &v1 = vec_v[vec_f[f_idx](0)];
            const KDL::Vector &v2 = vec_v[vec_f[f_idx](1)];
            const KDL::Vector &v3 = vec_v[vec_f[f_idx](2)];

            // calculate face angular extent
            double min_tg_x = std::min<double >(v1.x()/v1.z(), std::min<double >(v2.x()/v2.z(), v3.x()/v3.z()));
            double max_tg_x = std::max<double >(v1.x()/v1.z(), std::max<double >(v2.x()/v2.z(), v3.x()/v3.z()));
            double min_tg_y = std::min<double >(v1.y()/v1.z(), std::min<double >(v2.y()/v2.z(), v3.y()/v3.z()));
            double max_tg_y = std::max<double >(v1.y()/v1.z(), std::max<double >(v2.y()/v2.z(), v3.y()/v3.z()));

            double t;

            for (int x = 0; x < pc_width_; ++x) {
                double rx, ry, rz;
                rz = 1.0;
                rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
                double tg_x = rx/rz;
                if (tg_x < min_tg_x || tg_x > max_tg_x) {
                    continue;
                }
                for (int y = 0; y < pc_height_; ++y) {
                    ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                    double tg_y = ry/rz;
                    if (tg_y < min_tg_y || tg_y > max_tg_y) {
                        continue;
                    }
                    KDL::Vector dir_H(rx, ry, rz);
                    dir_H.Normalize();
                    if (KDL::dot(dir_H, vec_fn[f_idx]) < 0 && rayTriangleIntersect(KDL::Vector(), dir_H, v1, v2, v3, vec_fn[f_idx], t) && t > 0.01) {
                        if (rays[x+y*pc_width_] > t) {
                            rays[x+y*pc_width_] = t;
                        }
                    }
                }
            }
        }

        for (int x = 0; x < pc_width_; ++x) {
            double rx, ry, rz;
            rz = 1.0;
            rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
            for (int y = 0; y < pc_height_; ++y) {
                ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                KDL::Vector dir_H(rx, ry, rz);
                dir_H.Normalize();
                if (rays[x+y*pc_width_] != 1000000.0) {
                    pc_H.push_back(dir_H*rays[x+y*pc_width_]);
                }
            }
        }

/*
        for (int x = 0; x < pc_width_; ++x) {
            double rx, ry, rz;
            rz = 1.0;
            rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
            for (int y = 0; y < pc_height_; ++y) {
                ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                KDL::Vector dir_H(rx, ry, rz);
                dir_H.Normalize();
                double dist = 1000000;
                bool hit = false;

                for (int f_idx = 0; f_idx < vec_f.size(); ++f_idx) {
                    double t;
                    if (rayTriangleIntersect(KDL::Vector(), dir_H, vec_v[vec_f[f_idx](0)], vec_v[vec_f[f_idx](1)], vec_v[vec_f[f_idx](2)], vec_fn[f_idx], t) && t > 0.01) {
                        if (!hit) {
                            dist = t;
                            hit = true;
                        }
                        else if (dist > t) {
                            dist = t;
                        }
                    }
                }
                if (hit) {
                    pc_H.push_back(dir_H*dist);
                }
            }
        }
*/
        return pc_H;

/*
///////////////////////
//        std::vector<KDL::Vector > pc_H;
        for (int x = 0; x < pc_width_; ++x) {
            double rx, ry, rz;
            rz = 1.0;
            rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
            for (int y = 0; y < pc_height_; ++y) {
                ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                KDL::Vector dir_H(rx, ry, rz);
                dir_H.Normalize();
                double dist = 1000000;
                bool hit = false;

                for (int l_idx = 0; l_idx < col_model_->getLinksCount(); ++l_idx) {
                    const KDL::Frame &T_B_L = links_fk_[l_idx];
                    KDL::Frame T_L_B = T_B_L.Inverse();
                    KDL::Frame T_L_H = T_L_B * T_B_H;
                    KDL::Frame T_H_L = T_L_H.Inverse();

                    // broadphase check using bounding sphere
                    double bs_radius = link_bounding_spheres_[l_idx].first;
                    const KDL::Vector &bs_center_L = link_bounding_spheres_[l_idx].second;
                    const KDL::Vector bs_center_H = T_H_L * bs_center_L;
                    double circle_ray_dist = (dir_H*bs_center_H).Norm();
                    if (circle_ray_dist > bs_radius) {
                        continue;
                    }

                    const std::string &l_name = col_model_->getLink(l_idx)->name;
                    KDL::Vector orig_L = T_L_H * KDL::Vector();
//                    KDL::Vector dir_L = KDL::Frame(T_L_H.M) * dir_H;
                    KDL::Vector lookat_L = T_L_H * dir_H;
                    KDL::Vector dir_L = lookat_L - orig_L;
                    for (int c_idx = 0; c_idx < collision_meshes_[l_idx].size(); ++c_idx) {
                        const KDL::Frame &T_L_M = collision_meshes_[l_idx][c_idx].first;
                        KDL::Frame T_M_L = T_L_M.Inverse();
                        const aiScene *m = collision_meshes_[l_idx][c_idx].second;
                        double t;

//                        if (l_name == "right_gripper_mount_link") {
//                        if (l_name == "right_HandPalmLink") {

//                        if (l_name == "right_arm_5_link") {
//                            ROS_INFO_STREAM( "rayMeshIntersect " << orig_L.x() << " " << orig_L.y() << " " << orig_L.z() << " " << dir_L.x() << " " << dir_L.y() << " " << dir_L.z()  );

                            if (rayMeshIntersect(T_M_L * orig_L, KDL::Frame(T_M_L.M) * dir_L, m, t)) {
//                            if (rayMeshIntersect(orig_L, dir_L, m, t)) {
//                            if (rayMeshIntersect(KDL::Vector(0,0.01,0), dir_H, m, t)) {
                                hit = true;
                                if (t < dist) {
                                    dist = t;
                                }
//                                pc_H.push_back(dir_L*t);
                            }
//                        }

                    }
                }
                if (hit) {
                    pc_H.push_back(dir_H*dist);
                }
                else {
//                    pc_H.push_back(dir_H*1);
                }
                //break;
            }
            //break;
        }
        return pc_H;*/
    }
/*
    void spin() {
        return;

        boost::shared_ptr< self_collision::Collision > shpere = self_collision::createCollisionSphere(tolerance_, KDL::Frame());

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

                std::vector<KDL::Frame > links_tf(col_model_->getLinksCount());
                std::set<std::string > colliding_links;
                std::vector<bool > pt_col(pc_.points.size(), false);
                bool tf_error = false;
                for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
                    const boost::shared_ptr< self_collision::Link > plink = col_model_->getLink(l_idx);
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

                    for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                        if (pt_col[pidx]) {
                            continue;
                        }
                        KDL::Frame T_C_P(KDL::Vector(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z));
                        if (self_collision::checkCollision(shpere, T_C_P, plink, T_C_L)) {
                            pt_col[pidx] = true;
                            colliding_links.insert( plink->name );
                        }
                    }
                }
                if (tf_error) {
                    errors++;
                    continue;
                }

                PclPointCloud pc_out;
                for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                    if (pt_col[pidx]) {
                        continue;
                    }
                    pc_out.push_back( pc_.points[pidx] );
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
*/

    bool simVisiblePointCloud(pointcloud_utils_msgs::SimVisiblePointCloud::Request  &req,
             pointcloud_utils_msgs::SimVisiblePointCloud::Response &res)
    {
        if (req.q.size() != articulated_joint_names_.size()) {
            res.success = false;
            res.status_message = "wrong size of 'q'";
            return false;
        }

        Eigen::VectorXd q(req.q.size());
        for (int i = 0; i < req.q.size(); ++i) {
            q(i) = req.q[i];
        }

        std::vector<KDL::Vector > pc_H = calculatePointCloud(q);

        for (int i = 0; i < pc_H.size(); ++i) {
            geometry_msgs::Vector3 p;
            p.x = pc_H[i].x();
            p.y = pc_H[i].y();
            p.z = pc_H[i].z();
            res.pc.push_back(p);
        }
        
        res.success = true;
        return true;
    }

    typedef pcl::PointXYZ PointType;
//    typedef pcl::PointNormal PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::SHOT352 DescriptorType;

    void flipNormalsTowardsOrigin(pcl::PointCloud<PointType>::Ptr pc, pcl::PointCloud<NormalType>::Ptr normals) {
        for (int i = 0; i < pc->points.size(); ++i) {
            double dot = pc->points[i].x * normals->points[i].normal_x + 
                pc->points[i].y * normals->points[i].normal_y +
                pc->points[i].z * normals->points[i].normal_z;
            if (dot > 0) {
                normals->points[i].normal_x = -normals->points[i].normal_x;
                normals->points[i].normal_y = -normals->points[i].normal_y;
                normals->points[i].normal_z = -normals->points[i].normal_z;
            }
        }
    }

    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr calculatePrincipalCurvatures(pcl::PointCloud<PointType>::Ptr pc, pcl::PointCloud<PointType>::Ptr keypoints, pcl::PointCloud<NormalType>::Ptr normals) {
        pcl::search::KdTree<PointType>::Ptr tree2 (new pcl::search::KdTree<PointType>);

        // Setup the principal curvatures computation
        pcl::PrincipalCurvaturesEstimation<PointType, NormalType, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

        // Provide the original point cloud (without normals)
        principalCurvaturesEstimation.setInputCloud (keypoints);

        principalCurvaturesEstimation.setSearchSurface(pc);

        // Provide the point cloud with normals
        principalCurvaturesEstimation.setInputNormals(normals);

        // Use the same KdTree from the normal estimation
        principalCurvaturesEstimation.setSearchMethod (tree2);
        principalCurvaturesEstimation.setRadiusSearch(0.01);

        // Actually compute the principal curvatures
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
        principalCurvaturesEstimation.compute (*principalCurvatures);
        return principalCurvatures;
    }

    bool findPointCloudCorrespondence(pointcloud_utils_msgs::FindPointCloudCorrespondence::Request  &req,
             pointcloud_utils_msgs::FindPointCloudCorrespondence::Response &res) {

        //Algorithm params
        bool show_keypoints_ (false);
        bool show_correspondences_ (false);
        bool use_cloud_resolution_ (false);
        bool use_hough_ (true);
        float model_ss_ (0.005f);
        float scene_ss_ (0.005f);
        float rf_rad_ (0.025f);
//        float descr_rad_ (0.02f);
        float cg_size_ (0.03f);
        float cg_thresh_ (5.0f);

        pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
        pcl::PointCloud<NormalType>::Ptr model_normals_all (new pcl::PointCloud<NormalType> ());
        pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
        pcl::PointCloud<NormalType>::Ptr scene_normals_all (new pcl::PointCloud<NormalType> ());
        pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

        pcl::PointCloud<PointType>::Ptr model_raw (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr scene_raw (new pcl::PointCloud<PointType> ());

        std::vector<PointType > pc1(req.pc1.size());
        for (int i = 0; i < req.pc1.size(); ++i) {
            pc1.push_back( PointType(req.pc1[i].x, req.pc1[i].y, req.pc1[i].z) );
        }
        std::vector<PointType > pc2(req.pc2.size());
        for (int i = 0; i < req.pc2.size(); ++i) {
            pc2.push_back( PointType(req.pc2[i].x, req.pc2[i].y, req.pc2[i].z) );
        }

        model_raw->insert(model_raw->begin(), pc1.begin(), pc1.end());
        scene_raw->insert(scene_raw->begin(), pc2.begin(), pc2.end());

        pcl::VoxelGrid<PointType > sor;
        sor.setLeafSize (0.003f, 0.003f, 0.003f);
        sor.setInputCloud (model_raw);
        sor.filter (*model);

        sor.setInputCloud (scene_raw);
        sor.filter (*scene);

        //
        //  Compute Normals for all points
        //
        pcl::search::KdTree<PointType>::Ptr normals_search (new pcl::search::KdTree<PointType> ());

        pcl::NormalEstimation<PointType, NormalType> norm_est;
        norm_est.setSearchMethod(normals_search);
        norm_est.setRadiusSearch (0.01);
        norm_est.setInputCloud (model);
        norm_est.compute (*model_normals_all);

        norm_est.setInputCloud (scene);
        norm_est.compute (*scene_normals_all);

        flipNormalsTowardsOrigin(model, model_normals_all);
        flipNormalsTowardsOrigin(scene, scene_normals_all);

        //
        //  Downsample Clouds to Extract keypoints
        //
/*
        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud (model);
        uniform_sampling.setRadiusSearch (model_ss_);
        // workaround begin
//        uniform_sampling.filter (*model_keypoints);
        pcl::PointCloud<int> keypointIndices1;
        uniform_sampling.compute(keypointIndices1);
        pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints); 
        pcl::copyPointCloud(*model_normals_all, keypointIndices1.points, *model_normals); 
        // workaround end
        std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

        uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (scene_ss_);
        // workaround begin
//        uniform_sampling.filter (*scene_keypoints);
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute(keypointIndices2);
        pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints); 
        pcl::copyPointCloud(*scene_normals_all, keypointIndices1.points, *scene_normals); 
        // workaround end
        std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
*/
model_keypoints = model;
scene_keypoints = scene;
model_normals = model_normals_all;
scene_normals = scene_normals_all;

        // calculate principal curvatures for keypoints
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr model_pc = calculatePrincipalCurvatures(model, model_keypoints, model_normals_all);
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr scene_pc = calculatePrincipalCurvatures(scene, scene_keypoints, scene_normals_all);

        std::vector<int > scene_flat_indices;
        std::vector<int > scene_edges_indices;
        std::vector<int > scene_round_indices;
{
        pointcloud_utils_msgs::Segment s1, s2, s3;

        for (int i = 0; i < scene_pc->size(); ++i) {
            geometry_msgs::Vector3 v;
            v.x = (*scene_keypoints)[i].x;
            v.y = (*scene_keypoints)[i].y;
            v.z = (*scene_keypoints)[i].z;
            if ((*scene_pc)[i].pc1 < 0.06) {
                scene_flat_indices.push_back(i);
                s1.p.push_back(v);
            }
            else if ((*scene_pc)[i].pc2 < 0.05) {
                scene_edges_indices.push_back(i);
                s2.p.push_back(v);
            }
            else {
                scene_round_indices.push_back(i);
                s3.p.push_back(v);
            }
        }

        res.seg1.push_back(s1);
        res.seg1.push_back(s2);
        res.seg1.push_back(s3);
}

{
        pointcloud_utils_msgs::Segment s1, s2, s3;

        for (int i = 0; i < model_pc->size(); ++i) {
            geometry_msgs::Vector3 v;
            v.x = (*model_keypoints)[i].x;
            v.y = (*model_keypoints)[i].y;
            v.z = (*model_keypoints)[i].z;
            if ((*model_pc)[i].pc1 < 0.06) {
                s1.p.push_back(v);
            }
            else if ((*model_pc)[i].pc2 < 0.05) {
                s2.p.push_back(v);
            }
            else {
                s3.p.push_back(v);
            }
        }

        res.seg2.push_back(s1);
        res.seg2.push_back(s2);
        res.seg2.push_back(s3);
}

        res.success = true;
        return true;

/*        //
        //  Compute Descriptor for keypoints
        //
        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch (descr_rad_);

        descr_est.setInputCloud (model_keypoints);
        descr_est.setInputNormals (model_normals);
        descr_est.setSearchSurface (model);
        descr_est.compute (*model_descriptors);

        descr_est.setInputCloud (scene_keypoints);
        descr_est.setInputNormals (scene_normals);
        descr_est.setSearchSurface (scene);
        descr_est.compute (*scene_descriptors);
*/
/*        //
        //  Find Model-Scene Correspondences with KdTree
        //
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

        pcl::KdTreeFLANN<PointType> pos_search;
        pos_search.setInputCloud (model_keypoints);

//        pcl::KdTreeFLANN<NormalType> normal_search;
//        normal_search.setInputCloud (model_normals);


        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < scene_keypoints->size (); ++i)
        {
            std::vector<int> neigh_indices;
            std::vector<float> neigh_sqr_dists;

//            const pcl::PrincipalCurvatures &c1 = (*scene_pc)[i];
//            if (c1.pc1 != c1.pc1 || c1.pc2 != c1.pc2) {
//                continue;
//            }
//            std::cout << c1.pc1 << " " << c1.pc2 << std::endl;

//            int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 100, neigh_indices, neigh_sqr_dists);
            int found_neighs = pos_search.radiusSearch (scene_keypoints->at (i), 0.05, neigh_indices, neigh_sqr_dists);

            const NormalType &n1 = (*scene_normals)[i];

            KDL::Vector cor_mean = KDL::Vector();
            std::vector<KDL::Vector > cor;
            pcl::Correspondences cor_vec;
            for (int j = 0; j < found_neighs; ++j) {
                const NormalType &n2 = (*model_normals)[neigh_indices[j]];
                double f = (KDL::Vector(n2.normal_x, n2.normal_y, n2.normal_z) * KDL::Vector(n1.normal_x, n1.normal_y, n1.normal_z)).Norm();   // arcus sinus
//                const pcl::PrincipalCurvatures &c2 = (*model_pc)[neigh_indices[j]];
//                if (c2.pc1 != c2.pc1 || c2.pc2 != c2.pc2) {
//                    continue;
//                }
                if (f < 0.2) {// && fabs(c1.pc1-c2.pc1) < 0.05 && fabs(c1.pc2-c2.pc2) < 0.025) {
                    pcl::Correspondence corr (neigh_indices[j], static_cast<int> (i), 1.0-f);
                    cor_vec.push_back(corr);
                    const PointType &p1 = (*scene_keypoints)[i];
                    const PointType &p2 = (*model_keypoints)[neigh_indices[j]];
                    KDL::Vector v(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
                    cor_mean = cor_mean + v;
                    cor.push_back(v);
                }
            }

            double variance = 0;
            // calculate mean and variance for cor_vec
            for (int j = 0; j < cor.size(); ++j) {
                double norm = (cor[j]-cor_mean).Norm();
                variance += norm*norm;
            }
            if (variance < 0.05) {
                model_scene_corrs->insert (model_scene_corrs->end(), cor_vec.begin(), cor_vec.end());
            }

//            if(found_neighs == 1 && neigh_sqr_dists[0] < 0.5f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//            {
//                pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//                model_scene_corrs->push_back (corr);
//            }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

        pcl::PointCloud<PointType>::Ptr translations (new pcl::PointCloud<PointType> ());
        for (int i = 0; i < model_scene_corrs->size(); ++i) {
            PointType t;
            t.x = (*scene_keypoints)[(*model_scene_corrs)[i].index_match].x - (*model_keypoints)[(*model_scene_corrs)[i].index_query].x;
            t.y = (*scene_keypoints)[(*model_scene_corrs)[i].index_match].y - (*model_keypoints)[(*model_scene_corrs)[i].index_query].y;
            t.z = (*scene_keypoints)[(*model_scene_corrs)[i].index_match].z - (*model_keypoints)[(*model_scene_corrs)[i].index_query].z;
            translations->push_back(t);
        }

        //
        //  Actual Clustering
        //
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;

        //  Using Hough3D
        if (use_hough_)
        {
            //
            //  Compute (Keypoints) Reference Frames only for Hough
            //
            pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
            pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

            pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
            rf_est.setFindHoles (true);
            rf_est.setRadiusSearch (rf_rad_);

            rf_est.setInputCloud (model_keypoints);
            rf_est.setInputNormals (model_normals_all);
            rf_est.setSearchSurface (model);
            rf_est.compute (*model_rf);

            rf_est.setInputCloud (scene_keypoints);
            rf_est.setInputNormals (scene_normals_all);
            rf_est.setSearchSurface (scene);
            rf_est.compute (*scene_rf);

            //  Clustering
            pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
            clusterer.setHoughBinSize (cg_size_);
            clusterer.setHoughThreshold (cg_thresh_);
            clusterer.setUseInterpolation (true);
            clusterer.setUseDistanceWeight (false);

            clusterer.setInputCloud (model_keypoints);
            clusterer.setInputRf (model_rf);
            clusterer.setSceneCloud (scene_keypoints);
            clusterer.setSceneRf (scene_rf);
            clusterer.setModelSceneCorrespondences (model_scene_corrs);

            //clusterer.cluster (clustered_corrs);
            clusterer.recognize (rototranslations, clustered_corrs);
        }
        else // Using GeometricConsistency
        {
            pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
            gc_clusterer.setGCSize (cg_size_);
            gc_clusterer.setGCThreshold (cg_thresh_);

            gc_clusterer.setInputCloud (model_keypoints);
            gc_clusterer.setSceneCloud (scene_keypoints);
            gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

            //gc_clusterer.cluster (clustered_corrs);
            gc_clusterer.recognize (rototranslations, clustered_corrs);
        }

        if (rototranslations.size() != clustered_corrs.size()) {
            ROS_ERROR_STREAM( "rototranslations.size() != clustered_corrs.size()" );
            res.success = false;
            return false;
        }
        std::cout << "clusters: " << rototranslations.size() << std::endl;

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (translations);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02);
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (translations);
        ec.extract (cluster_indices);

        std::cout << "cluster_indices: " << cluster_indices.size() << std::endl;
        for (int i = 0; i < cluster_indices.size(); ++i) {
            KDL::Vector mean = KDL::Vector();
            for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
                const PointType &t = (*translations)[cluster_indices[i].indices[j]];
                mean = mean + KDL::Vector(t.x, t.y, t.z);
            }
            mean = mean * (1.0/cluster_indices[i].indices.size());
            if (i == 0) {
                res.translation.x = mean.x();
                res.translation.y = mean.y();
                res.translation.z = mean.z();
            }
            std::cout << "    " << cluster_indices[i].indices.size() << ": " << mean.x() << " " << mean.y() << " " << mean.z() << std::endl;
        }


        for (int i = 0; i < rototranslations.size(); ++i) {
            pointcloud_utils_msgs::Correspondences c;
            Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
            Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
            KDL::Frame T( KDL::Rotation(rotation(0,0), rotation(0,1), rotation(0,2),
                                        rotation(1,0), rotation(1,1), rotation(1,2),
                                        rotation(2,0), rotation(2,1), rotation(2,2)), KDL::Vector(translation(0), translation(1), translation(2)) );

            c.transform.position.x = T.p.x();
            c.transform.position.y = T.p.y();
            c.transform.position.z = T.p.z();
            double qx, qy, qz, qw;
            T.M.GetQuaternion(qx,qy,qz,qw);
            c.transform.orientation.x = qx;
            c.transform.orientation.y = qy;
            c.transform.orientation.z = qz;
            c.transform.orientation.w = qw;

            for (int j = 0; j < clustered_corrs[i].size(); ++j) {
                c.source_indices.push_back( clustered_corrs[i][j].index_query );
                c.target_indices.push_back( clustered_corrs[i][j].index_match );
            }
            res.corr.push_back(c);
        }

        for (int i = 0; i < model_keypoints->size(); ++i) {
            geometry_msgs::Vector3 v;
            v.x = (*model_keypoints)[i].x;
            v.y = (*model_keypoints)[i].y;
            v.z = (*model_keypoints)[i].z;
            res.pc1_keypoints.push_back(v);
        }

        for (int i = 0; i < scene_keypoints->size(); ++i) {
            geometry_msgs::Vector3 v;
            v.x = (*scene_keypoints)[i].x;
            v.y = (*scene_keypoints)[i].y;
            v.z = (*scene_keypoints)[i].z;
            res.pc2_keypoints.push_back(v);
        }


        // find all planes
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<PointType> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (5000);
        seg.setDistanceThreshold (0.005);

        // Create the filtering object
        pcl::ExtractIndices<PointType> extract;

        pcl::PointCloud<PointType>::Ptr cloud_p (new pcl::PointCloud<PointType>);
{
        int i = 0, nr_points = (int) model->points.size ();
        // While 30% of the original cloud is still there
        while (model->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (model);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud (model);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


            // segment the planar component
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_p);

            std::vector<pcl::PointIndices> cluster_indices;
            std::vector<pcl::PointIndices>::const_iterator biggest_cluster = cluster_indices.begin();
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_p);
            ec.extract (cluster_indices);

            int biggest_cluster_size = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
                if (it->indices.size() > biggest_cluster_size) {
                    biggest_cluster_size = it->indices.size();
                    biggest_cluster = it;
                }
            }

            // add the biggest planar cluster to result
            pointcloud_utils_msgs::Segment segment;
            for (std::vector<int>::const_iterator pit = biggest_cluster->indices.begin (); pit != biggest_cluster->indices.end (); ++pit) {
                geometry_msgs::Vector3 v;
                v.x = model->points[*pit].x;
                v.y = model->points[*pit].y;
                v.z = model->points[*pit].z;
                segment.p.push_back(v);
            }
            res.seg1.push_back(segment);

            pcl::PointCloud<PointType>::Ptr cloud_p (new pcl::PointCloud<PointType>), cloud_f (new pcl::PointCloud<PointType>);
            // add all other points to filtered point cloud
            for (int p_idx = 0; p_idx < model->size(); ++p_idx) {
                bool found = false;
                for (std::vector<int>::const_iterator pit = biggest_cluster->indices.begin (); pit != biggest_cluster->indices.end (); ++pit) {
                    if (p_idx == (*pit)) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    continue;
                }
                cloud_f->push_back((*model)[p_idx]);
            }

            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                pointcloud_utils_msgs::Segment segment;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                    cloud_cluster->points.push_back (model->points[*pit]);

                    geometry_msgs::Vector3 v;
                    v.x = model->points[*pit].x;
                    v.y = model->points[*pit].y;
                    v.z = model->points[*pit].z;
                    segment.p.push_back(v);
                }
                res.seg1.push_back(segment);



                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

    //            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    //            std::stringstream ss;
    //            ss << "cloud_cluster_" << j << ".pcd";
    //            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //            j++;
            }


            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);

            model = cloud_f;//.swap (cloud_f);
            i++;
        }


}

{
        int i = 0, nr_points = (int) scene->points.size ();
        // While 30% of the original cloud is still there
        while (scene->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (scene);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud (scene);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

            int size = cloud_p->points.size();
            double c_r = (double)(rand()%1000) / 1000.0;
            double c_g = (double)(rand()%1000) / 1000.0;
            double c_b = (double)(rand()%1000) / 1000.0;
            pointcloud_utils_msgs::Segment segment;
            for (int i = 0; i < cloud_p->points.size(); i++) {
                geometry_msgs::Vector3 v;
                v.x = cloud_p->points[i].x;
                v.y = cloud_p->points[i].y;
                v.z = cloud_p->points[i].z;
                segment.p.push_back(v);
            }
            res.seg2.push_back(segment);

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            scene.swap (cloud_f);
            i++;
        }
}*/
        res.success = true;
        return true;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_visible_pointcloud");
    SimVisiblePointCloud pc;
    pc.load();
    ros::spin();
    return 0;
}


